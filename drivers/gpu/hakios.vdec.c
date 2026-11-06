/*
 * The Amlogic box's video decoders (S905X5M "S7D": the DOS block, VDEC_1 for H.264 and HEVC for H.265/VP9/AV1),
 * step one: power, clocks and reset, then their registers read back. Boot-arg vdec=1; vdec_delay=N waits N seconds
 * first (in a thread of its own), so that the lines land late in the log.
 *
 * DOS registers are 32 bit, at 0xfe320000 + reg * 4 (Amlogic's dos_registers_v1.h numbers). Each core has a power
 * domain, switched by the SiP call 0x82000093 (domain, 1: on) and read by 0x82000095 (domain: 1 if off); VDEC is
 * domain 2, HEVC 1 (s7d-pd.h). Their clocks are CLKCTRL VDEC_CLK_CTRL (0x140, VDEC) and VDEC2_CLK_CTRL (0x144,
 * HEVC): bits 9-11 the source (1: fclk_div3, 667 MHz), 0-6 the divider, 8 the gate; bit 15 of VDEC3/VDEC4_CLK_CTRL
 * picks between those and a second copy (0: the first). The DOS bus clock is SYS_CLK_EN0_REG0 bit 1. The order is
 * Amlogic's media_modules (vdec_power_ctrl.c: pm_vdec_clock_on, the power domain, dos_local_config).
 *
 * The cores' microcode is encrypted and loaded by the secure world (hakios.optee.c); this file only proves that the
 * block is alive.
 */
#include <kern/clock.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/hakios.hdmitx.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void IOSleep(unsigned int milliseconds);
extern void IODelay(unsigned int microseconds);
extern kern_return_t kernel_thread_start(thread_continue_t continuation, void *parameter, thread_t *new_thread);
extern void thread_deallocate(thread_t thread);

#define DOS_BASE        0xfe320000UL
#define DOS_SIZE        0x10000UL
#define CLK_BASE        0xfe000000UL

#define SIP_PWRC_SET    0x82000093UL
#define SIP_PWRC_GET    0x82000095UL
#define PD_HEVC         1
#define PD_VDEC         2

/* CLKCTRL */
#define SYS_CLK_EN0_REG0 0x044
#define SYS_CLK_DOS     (1u << 1)
#define VDEC_CLK_CTRL   0x140
#define VDEC2_CLK_CTRL  0x144
#define VDEC3_CLK_CTRL  0x148
#define VDEC4_CLK_CTRL  0x14c
#define CLK_SRC_FCLK_DIV3 1u

/* DOS register numbers */
#define MPSR            0x0301
#define CPSR            0x0321
#define IMEM_DMA_CTRL   0x0340
#define AV_SCRATCH_0    0x09c0
#define HEVC_ASSIST_SCRATCH_0 0x30c0
#define HEVC_PARSER_VERSION 0x3100
#define HEVC_MPSR       0x3301
#define HEVC_CPSR       0x3321
#define DOS_SW_RESET0   0x3f00
#define DOS_GCLK_EN0    0x3f01
#define DOS_MEM_PD_VDEC 0x3f30
#define DOS_MEM_PD_HEVC 0x3f33
#define DOS_SW_RESET3   0x3f34
#define DOS_GCLK_EN3    0x3f35

static vm_offset_t dos, clk;

static uint64_t
sip(uint64_t function, uint64_t a1, uint64_t a2, uint64_t a3)
{
	register uint64_t x0 __asm__("x0") = function;
	register uint64_t x1 __asm__("x1") = a1;
	register uint64_t x2 __asm__("x2") = a2;
	register uint64_t x3 __asm__("x3") = a3;

	__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3) : : "x4", "x5", "x6", "x7", "x8", "x9",
	    "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return x0;
}

static uint32_t clk_rd(uint32_t off) { return *(volatile uint32_t *)(clk + off); }
static void clk_wr(uint32_t off, uint32_t v) { *(volatile uint32_t *)(clk + off) = v; }
static uint32_t dos_rd(uint32_t reg) { return *(volatile uint32_t *)(dos + reg * 4); }
static void dos_wr(uint32_t reg, uint32_t v) { *(volatile uint32_t *)(dos + reg * 4) = v; }

/* one core's clock: gate shut, source and divider, gate open, then its final mux on the first copy */
static void
core_clock(uint32_t ctrl, uint32_t mux)
{
	uint32_t v = clk_rd(ctrl) & ~0xffffu;

	clk_wr(ctrl, v | (CLK_SRC_FCLK_DIV3 << 9));
	IODelay(5);
	clk_wr(ctrl, v | (CLK_SRC_FCLK_DIV3 << 9) | (1u << 8));
	IODelay(5);
	clk_wr(mux, clk_rd(mux) & ~(1u << 15));
}

static bool
power_on(unsigned pd, const char *name)
{
	uint64_t before = sip(SIP_PWRC_GET, pd, 0, 0), ret, after;

	ret = sip(SIP_PWRC_SET, pd, 1, 0);
	IODelay(50);
	after = sip(SIP_PWRC_GET, pd, 0, 0);
	printf("vdec: %s power domain %u: %s before, set returned %#llx, %s after\n", name, pd,
	    before ? "off" : "on", ret, after ? "OFF" : "on");
	return after == 0;
}

/* the DOS block's own registers (0x3fxx): readable as soon as a domain is on */
static void
show_dos(const char *when)
{
	printf("vdec: %s: SW_RESET0 %#x GCLK_EN0 %#x MEM_PD_VDEC %#x SW_RESET3 %#x GCLK_EN3 %#x MEM_PD_HEVC %#x\n", when,
	    dos_rd(DOS_SW_RESET0), dos_rd(DOS_GCLK_EN0), dos_rd(DOS_MEM_PD_VDEC), dos_rd(DOS_SW_RESET3),
	    dos_rd(DOS_GCLK_EN3), dos_rd(DOS_MEM_PD_HEVC));
}

/* a core's own registers: an external abort unless its memories are on, its reset released and its clocks open;
 * one register per line, so that a fault names the register */
static void
show_core(const char *name, const uint32_t *regs, const char *const *names, unsigned n)
{
	for (unsigned i = 0; i < n; i++) {
		printf("vdec: %s: reading %s (%#x)\n", name, names[i], regs[i]);
		printf("vdec: %s: %s = %#x\n", name, names[i], dos_rd(regs[i]));
	}
}

static bool
scratch_test(uint32_t reg, const char *name)
{
	uint32_t old = dos_rd(reg), got;

	dos_wr(reg, 0x5a5aa5a5u);
	got = dos_rd(reg);
	dos_wr(reg, old);
	printf("vdec: %s scratch: wrote 0x5a5aa5a5, read %#x: %s\n", name, got, got == 0x5a5aa5a5u ? "works" : "FAILED");
	return got == 0x5a5aa5a5u;
}

static void
vdec_bringup(void)
{
	uint32_t gate;
	bool vdec_on, hevc_on;

	clk = ml_io_map(CLK_BASE, 0x1000);
	dos = ml_io_map(DOS_BASE, DOS_SIZE);
	if (clk == 0 || dos == 0) {
		printf("vdec: cannot map the registers\n");
		return;
	}
	gate = clk_rd(SYS_CLK_EN0_REG0);
	printf("vdec: DOS bus clock %s (SYS_CLK_EN0_REG0 %#x); clocks before: VDEC %#x VDEC2 %#x VDEC3 %#x VDEC4 %#x\n",
	    (gate & SYS_CLK_DOS) ? "open" : "shut", gate, clk_rd(VDEC_CLK_CTRL), clk_rd(VDEC2_CLK_CTRL),
	    clk_rd(VDEC3_CLK_CTRL), clk_rd(VDEC4_CLK_CTRL));
	if (!(gate & SYS_CLK_DOS)) {
		clk_wr(SYS_CLK_EN0_REG0, gate | SYS_CLK_DOS);
	}

	core_clock(VDEC_CLK_CTRL, VDEC3_CLK_CTRL);
	core_clock(VDEC2_CLK_CTRL, VDEC4_CLK_CTRL);
	printf("vdec: clocks after: VDEC %#x VDEC2 %#x VDEC3 %#x VDEC4 %#x\n", clk_rd(VDEC_CLK_CTRL),
	    clk_rd(VDEC2_CLK_CTRL), clk_rd(VDEC3_CLK_CTRL), clk_rd(VDEC4_CLK_CTRL));

	vdec_on = power_on(PD_VDEC, "VDEC");
	hevc_on = power_on(PD_HEVC, "HEVC");
	if (!vdec_on && !hevc_on) {
		printf("vdec: neither core powered up; the DOS registers are left alone\n");
		return;
	}
	show_dos("powered");

	/* dos_local_config(): memories on, each core held in reset for 20 us; then its clock gates opened */
	if (vdec_on) {
		dos_wr(DOS_MEM_PD_VDEC, 0);
		dos_wr(DOS_SW_RESET0, 0xfffffffcu);
		IODelay(20);
		dos_wr(DOS_SW_RESET0, 0);
		IODelay(10);
		dos_wr(DOS_MEM_PD_VDEC, 0);
		dos_wr(DOS_GCLK_EN0, 0xffffffffu);
	}
	if (hevc_on) {
		dos_wr(DOS_MEM_PD_HEVC, 0);
		dos_wr(DOS_SW_RESET3, 0xffffffffu);
		IODelay(20);
		dos_wr(DOS_SW_RESET3, 0);
		IODelay(10);
		dos_wr(DOS_GCLK_EN3, 0xffffffffu);
	}
	IODelay(10);
	show_dos("reset, clocks open");
	if (vdec_on) {
		static const uint32_t r[] = { MPSR, CPSR, IMEM_DMA_CTRL };
		static const char *const n[] = { "MPSR", "CPSR", "IMEM_DMA_CTRL" };

		show_core("VDEC", r, n, 3);
		scratch_test(AV_SCRATCH_0, "VDEC");
	}
	if (hevc_on) {
		static const uint32_t r[] = { HEVC_PARSER_VERSION, HEVC_MPSR, HEVC_CPSR };
		static const char *const n[] = { "HEVC_PARSER_VERSION", "HEVC_MPSR", "HEVC_CPSR" };

		show_core("HEVC", r, n, 3);
		scratch_test(HEVC_ASSIST_SCRATCH_0, "HEVC");
	}
	printf("vdec: bring-up done\n");
}

static void
vdec_later(void *arg, wait_result_t wr)
{
	(void)wr;
	IOSleep((unsigned int)(uintptr_t)arg * 1000);
	vdec_bringup();
}

void
hakios_vdec_init(void)
{
	unsigned int on = 0, delay = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("vdec", &on, sizeof(on)) || on == 0) {
		return;
	}
	(void)PE_parse_boot_argn("vdec_delay", &delay, sizeof(delay));
	if (delay == 0) {
		vdec_bringup();
		return;
	}
	if (kernel_thread_start(vdec_later, (void *)(uintptr_t)delay, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		printf("vdec: no thread for the delayed bring-up\n");
	}
}

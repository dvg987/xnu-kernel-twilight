/*
 * HDMI-CEC on the Amlogic box (S905X5M "S7D": the CEC-B controller at 0xfe044000): the TV's own remote as
 * the box's remote. Boot-arg cec=1. The box claims logical address 4 (playback device 1), answers what a TV
 * asks a playback device, and turns <User Control Pressed>/<Released> into key records of /dev/hakios-touch
 * (HID consumer page: Menu Pick/Up/Down/Left/Right, Home, Play/Pause; Back as a keyboard's Escape), the same queue as the USB keys
 * (hakios.crgudc.c) and backboardd's twilight keys take them (tvos/src/twlinput.inc).
 *
 * Polled from a kernel thread, no interrupt. The programming model is Amlogic's Linux driver
 * (common_drivers/drivers/media/cec: hdmi_ao_cec.c, hdmi_aocec_api.c): the controller's own registers are
 * reached one byte at a time through AO_CECB_RW_REG. The 32.768 kHz clock and the GPIOH_3 pin are set here
 * as Linux's clock and pinctrl drivers would, in case the firmware left them otherwise.
 *
 * The physical address (which of the TV's inputs the box is on) is boot-arg cec_pa (e.g. cec_pa=0x2000 for input
 * 2), else the TV's EDID as hakios.hdmitx.c read it (hdmitx=1), else the one Linux's driver last left in
 * SYSCTRL_STATUS_REG1. Without one the box never claims to be the active source: the TV would switch to
 * an input that is not the box's ("no signal").
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/hakios.input.h>
#include <sys/hakios.cec.h>
#include <sys/hakios.hdmitx.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void IOSleep(unsigned int milliseconds);
extern void IODelay(unsigned int microseconds);

#define CEC_AO_BASE     0xfe044000UL
#define AO_CECB_GEN_CNTL        0x40
#define AO_CECB_RW_REG          0x44
#define AO_CECB_INTR_MASKN      0x48
#define AO_CECB_INTR_CLR        0x4c
#define AO_CECB_INTR_STAT       0x50

#define CLK_BASE        0xfe000000UL
#define CLK_SYS_EN0     0x44            /* bit 16: the CEC block's bus clock */
#define CLK_CECB_CTRL0  0x90
#define CLK_CECB_CTRL1  0x94

#define PAD_BASE        0xfe004000UL
#define PIN_MUX_B       0x2c            /* GPIOH_3 = bits 15:12, function 2 = CEC-B */

/* the controller's own registers, through AO_CECB_RW_REG */
#define CEC_CTRL        0x00
#define CEC_CTRL2       0x01
#define CEC_ADDR_L      0x05
#define CEC_ADDR_H      0x06
#define CEC_TX_CNT      0x07
#define CEC_RX_CNT      0x08
#define CEC_TX_DATA     0x10
#define CEC_RX_DATA     0x20
#define CEC_LOCK        0x30
#define CEC_WKUPCTRL    0x31

#define RW_BUSY         (1u << 23)
#define RW_WRITE        (1u << 16)

#define I_TX_DONE       (1u << 0)
#define I_RX_EOM        (1u << 1)
#define I_TX_NACK       (1u << 2)
#define I_TX_ARB_LOST   (1u << 3)
#define I_TX_ERR        (1u << 4)
#define I_RX_ERR        (1u << 5)
#define I_RX_WAKEUP     (1u << 6)
#define I_ALL           0x7fu

#define SYSCTRL_STATUS_REG1     0xfe010284UL    /* [15:0]: the physical address Linux's driver saw last */
#define PHYS_ADDR       c.pa
#define BROADCAST       0xf
#define OSD_NAME        "Apple TV"
/* Apple's IEEE OUI, as an Apple TV reports it */
#define VENDOR_ID       0x0010fa

enum { TX_OK, TX_NACK, TX_FAIL };

static struct {
	vm_offset_t ao, clk, pad;
	int la;                 /* our logical address, 15 = none */
	uint32_t pa;            /* our physical address, 0xffff = not known */
	uint32_t pending;       /* status bits seen while transmitting, for the receiver */
	uint32_t held;          /* the key record of the button down now, 0 = none */
	unsigned frames;
} c;

static inline uint32_t
rd32(vm_offset_t base, uint32_t off)
{
	return *(volatile uint32_t *)(base + off);
}

static inline void
wr32(vm_offset_t base, uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(base + off) = v;
}

static uint8_t
cec_rd(uint8_t reg)
{
	uint32_t v;

	wr32(c.ao, AO_CECB_RW_REG, reg);
	for (int i = 0; i < 1000; i++) {
		v = rd32(c.ao, AO_CECB_RW_REG);
		if (!(v & RW_BUSY)) {
			break;
		}
		IODelay(1);
	}
	return (uint8_t)(v >> 24);
}

static void
cec_wr(uint8_t reg, uint8_t value)
{
	for (int i = 0; i < 1000 && (rd32(c.ao, AO_CECB_RW_REG) & RW_BUSY); i++) {
		IODelay(1);
	}
	wr32(c.ao, AO_CECB_RW_REG, RW_WRITE | (uint32_t)value << 8 | reg);
}

static void
set_address(int la)
{
	c.la = la;
	cec_wr(CEC_ADDR_L, la < 8 ? 1u << la : 0);
	cec_wr(CEC_ADDR_H, la >= 8 && la < 15 ? 1u << (la - 8) : 0);
}

static void
controller_reset(void)
{
	uint32_t g = rd32(c.ao, AO_CECB_GEN_CNTL);

	wr32(c.ao, AO_CECB_GEN_CNTL, g | 1);
	wr32(c.ao, AO_CECB_GEN_CNTL, g & ~1u);
	IODelay(10);
	wr32(c.ao, AO_CECB_GEN_CNTL, 0x7109);   /* filter 7, tick 1 us, system clock on, in reset */
	wr32(c.ao, AO_CECB_GEN_CNTL, 0x710b);   /* gated clock */
	wr32(c.ao, AO_CECB_GEN_CNTL, 0x710a);   /* out of reset */
	cec_wr(CEC_CTRL2, 0x08);                /* arbitration time */
	cec_wr(CEC_WKUPCTRL, 0x83);
	set_address(c.la);
	wr32(c.ao, AO_CECB_INTR_CLR, I_ALL);
	wr32(c.ao, AO_CECB_INTR_MASKN, 0x3f);
}

static void receive(void);

/* one frame (header first, up to 16 bytes); waits for the end of it */
static int
transmit(const uint8_t *msg, int len)
{
	for (int attempt = 0; attempt < 3; attempt++) {
		uint32_t st = 0;

		if (cec_rd(CEC_LOCK)) {
			receive();
		}
		for (int i = 0; i < 25 && (cec_rd(CEC_CTRL) & 1); i++) {
			IOSleep(20);
		}
		wr32(c.ao, AO_CECB_INTR_CLR, I_TX_DONE | I_TX_NACK | I_TX_ARB_LOST | I_TX_ERR);
		for (int i = 0; i < len; i++) {
			cec_wr(CEC_TX_DATA + i, msg[i]);
		}
		cec_wr(CEC_TX_CNT, (uint8_t)len);
		cec_wr(CEC_CTRL, (cec_rd(CEC_CTRL) & ~7u) | 3);         /* start, as a new initiator */
		for (int ms = 0; ms < 2000; ms += 2) {
			st = rd32(c.ao, AO_CECB_INTR_STAT);
			if (st & (I_TX_DONE | I_TX_NACK | I_TX_ARB_LOST | I_TX_ERR)) {
				break;
			}
			IOSleep(2);
		}
		wr32(c.ao, AO_CECB_INTR_CLR, st & (I_TX_DONE | I_TX_NACK | I_TX_ARB_LOST | I_TX_ERR));
		c.pending |= st & (I_RX_EOM | I_RX_ERR);
		if (st & I_TX_NACK) {
			return TX_NACK;
		}
		if (st & I_TX_DONE) {
			return TX_OK;
		}
		if (!(st & (I_TX_ARB_LOST | I_TX_ERR))) {
			printf("cec: transmit timed out (status %#x); resetting\n", st);
			controller_reset();
		} else {
			cec_wr(CEC_TX_CNT, 0);
			cec_wr(CEC_CTRL, cec_rd(CEC_CTRL) & ~7u);
		}
		IOSleep(100 + 30 * attempt);
	}
	return TX_FAIL;
}

static void
send(int to, const uint8_t *body, int n)
{
	uint8_t m[16];

	m[0] = (uint8_t)(c.la << 4 | to);
	for (int i = 0; i < n && i < 15; i++) {
		m[1 + i] = body[i];
	}
	transmit(m, n + 1);
}

static void
announce(void)
{
	const uint8_t report_pa[] = { 0x84, PHYS_ADDR >> 8, PHYS_ADDR & 0xff, 4 };
	const uint8_t vendor[] = { 0x87, VENDOR_ID >> 16, (VENDOR_ID >> 8) & 0xff, VENDOR_ID & 0xff };
	const uint8_t view_on[] = { 0x04 };
	const uint8_t active[] = { 0x82, PHYS_ADDR >> 8, PHYS_ADDR & 0xff };
	const uint8_t menu_on[] = { 0x8e, 0x00 };

	if (c.pa == 0xffff) {
		return;
	}
	send(BROADCAST, report_pa, sizeof(report_pa));
	send(BROADCAST, vendor, sizeof(vendor));
	send(0, view_on, sizeof(view_on));
	send(BROADCAST, active, sizeof(active));
	send(0, menu_on, sizeof(menu_on));
}

/*
 * <User Control Pressed> operand -> key record code, (HID usage page << 16) | usage. tvOS's "back" is a
 * keyboard's Escape; the consumer page's Menu is taken for Home.
 */
static uint32_t
key_of(uint8_t ui)
{
	switch (ui) {
	case 0x00: return 0x0c0041;     /* Select -> Menu Pick */
	case 0x01: return 0x0c0042;     /* Up */
	case 0x02: return 0x0c0043;     /* Down */
	case 0x03: return 0x0c0044;     /* Left */
	case 0x04: return 0x0c0045;     /* Right */
	case 0x0d: return 0x070029;     /* Exit (the remote's Back) -> Escape */
	case 0x09:                      /* Root Menu */
	case 0x0a:                      /* Setup Menu */
	case 0x0b: return 0x0c0223;     /* Contents Menu -> AC Home */
	case 0x44:                      /* Play */
	case 0x46:                      /* Pause */
	case 0x61: return 0x0c00cd;     /* Pause-Play -> Play/Pause */
	default: return 0;
	}
}

static void
key(uint32_t code, bool down)
{

	if (down) {
		if (c.held != 0 && c.held != code) {
			hakios_touch_post(HAKIOS_BUTTON_UP, c.held, 0, 0);
		}
		if (c.held != code) {
			hakios_touch_post(HAKIOS_BUTTON_DOWN, code, 0, 0);
		}
		c.held = code;
	} else if (c.held != 0) {
		hakios_touch_post(HAKIOS_BUTTON_UP, c.held, 0, 0);
		c.held = 0;
	}
}

static void
handle(const uint8_t *m, int n)
{
	int from = m[0] >> 4, to = m[0] & 0xf;
	uint8_t op = n > 1 ? m[1] : 0;
	uint8_t r[16];

	if (c.frames++ < 64) {
		printf("cec: %x -> %x:", from, to);
		for (int i = 1; i < n; i++) {
			printf(" %02x", m[i]);
		}
		printf("\n");
	}
	if (n < 2 || (to != c.la && to != BROADCAST)) {
		return;
	}
	switch (op) {
	case 0x44:              /* User Control Pressed */
		if (n > 2) {
			uint32_t code = key_of(m[2]);

			if (code != 0) {
				key(code, true);
			} else {
				printf("cec: key %#x not mapped\n", m[2]);
			}
		}
		return;
	case 0x45:              /* User Control Released */
		key(0, false);
		return;
	case 0x83:              /* Give Physical Address */
		r[0] = 0x84; r[1] = PHYS_ADDR >> 8; r[2] = PHYS_ADDR & 0xff; r[3] = 4;
		send(BROADCAST, r, 4);
		return;
	case 0x8c:              /* Give Device Vendor ID */
		r[0] = 0x87; r[1] = VENDOR_ID >> 16; r[2] = (VENDOR_ID >> 8) & 0xff; r[3] = VENDOR_ID & 0xff;
		send(BROADCAST, r, 4);
		return;
	case 0x85:              /* Request Active Source */
		if (c.pa == 0xffff) {
			return;
		}
		r[0] = 0x82; r[1] = PHYS_ADDR >> 8; r[2] = PHYS_ADDR & 0xff;
		send(BROADCAST, r, 3);
		return;
	case 0x86:              /* Set Stream Path */
		if (n >= 4 && c.pa != 0xffff && (uint32_t)(m[2] << 8 | m[3]) == PHYS_ADDR) {
			r[0] = 0x82; r[1] = PHYS_ADDR >> 8; r[2] = PHYS_ADDR & 0xff;
			send(BROADCAST, r, 3);
			r[0] = 0x8e; r[1] = 0x00;
			send(from, r, 2);
		}
		return;
	}
	if (to == BROADCAST) {
		return;
	}
	switch (op) {
	case 0x46:              /* Give OSD Name */
		r[0] = 0x47;
		memcpy(r + 1, OSD_NAME, sizeof(OSD_NAME) - 1);
		send(from, r, sizeof(OSD_NAME));
		return;
	case 0x8d:              /* Menu Request */
		r[0] = 0x8e; r[1] = 0x00;       /* activated */
		send(from, r, 2);
		return;
	case 0x8f:              /* Give Device Power Status */
		r[0] = 0x90; r[1] = 0x00;       /* on */
		send(from, r, 2);
		return;
	case 0x9f:              /* Get CEC Version */
		r[0] = 0x9e; r[1] = 0x05;       /* 1.4 */
		send(from, r, 2);
		return;
	case 0x1a:              /* Give Deck Status */
		r[0] = 0x1b; r[1] = 0x1a;       /* stop */
		send(from, r, 2);
		return;
	case 0x00:              /* Feature Abort: nothing to answer */
	case 0x36:              /* Standby */
		return;
	default:                /* Feature Abort: unrecognized opcode */
		r[0] = 0x00; r[1] = op; r[2] = 0x00;
		send(from, r, 3);
		return;
	}
}

static void
receive(void)
{
	uint8_t m[16];
	int n;

	if (!cec_rd(CEC_LOCK)) {
		return;
	}
	n = cec_rd(CEC_RX_CNT);
	if (n > 16) {
		n = 16;
	}
	for (int i = 0; i < n; i++) {
		m[i] = cec_rd(CEC_RX_DATA + i);
	}
	cec_wr(CEC_LOCK, 0);
	if (n > 0) {
		handle(m, n);
	}
}

static void
cec_thread(__unused void *arg, __unused wait_result_t wr_)
{
	static const int candidates[] = { 4, 8, 11 };
	uint8_t poll;

	/* claim a playback device's address: a poll nobody answers is ours */
	for (unsigned i = 0; i < sizeof(candidates) / sizeof(candidates[0]) && c.la == 15; i++) {
		int r;

		poll = (uint8_t)(candidates[i] << 4 | candidates[i]);
		r = transmit(&poll, 1);
		printf("cec: poll of address %d: %s\n", candidates[i], r == TX_NACK ? "free" : r == TX_OK ? "taken" : "failed");
		if (r == TX_NACK) {
			set_address(candidates[i]);
		}
	}
	if (c.la == 15) {
		printf("cec: no address of a playback device is free; listening only\n");
	} else {
		printf("cec: logical address %d, physical address %x.%x.%x.%x%s\n", c.la, PHYS_ADDR >> 12,
		    (PHYS_ADDR >> 8) & 0xf, (PHYS_ADDR >> 4) & 0xf, PHYS_ADDR & 0xf,
		    c.pa == 0xffff ? " (not known: not claiming the TV's input)" : "");
		announce();
	}
	for (;;) {
		uint32_t st = rd32(c.ao, AO_CECB_INTR_STAT) | c.pending;

		c.pending = 0;
		if (st & I_ALL) {
			wr32(c.ao, AO_CECB_INTR_CLR, st & I_ALL);
		}
		if ((st & I_RX_EOM) || cec_rd(CEC_LOCK)) {
			receive();
		}
		if (st & I_RX_ERR) {
			cec_wr(CEC_LOCK, 0);
		}
		if (st & I_RX_WAKEUP) {
			cec_wr(CEC_WKUPCTRL, 0x83);
		}
		IOSleep(5);
	}
}

void
hakios_cec_init(void)
{
	uint32_t on = 0, v;
	thread_t thread;

	if (!PE_parse_boot_argn("cec", &on, sizeof(on)) || !on) {
		return;
	}
	c.ao = ml_io_map(CEC_AO_BASE, 0x1000);
	c.clk = ml_io_map(CLK_BASE, 0x1000);
	c.pad = ml_io_map(PAD_BASE, 0x1000);
	c.la = 15;
	c.pa = 0xffff;
	if (!PE_parse_boot_argn("cec_pa", &c.pa, sizeof(c.pa)) && (c.pa = hakios_hdmitx_physical_address()) != 0xffff) {
		printf("cec: physical address from the TV's EDID\n");
	} else if (c.pa == 0xffff) {
		vm_offset_t sys = ml_io_map(SYSCTRL_STATUS_REG1 & ~0xfffUL, 0x1000);

		v = rd32(sys, SYSCTRL_STATUS_REG1 & 0xfff) & 0xffff;
		printf("cec: physical address Linux left: %#x\n", v);
		if (v != 0 && v != 0xffff) {
			c.pa = v;
		}
	}
	c.pa &= 0xffff;

	/* the bus clock, then 32768 Hz from the 24 MHz crystal: dual divider n1 733, n2 732, m1 8, m2 11 */
	wr32(c.clk, CLK_SYS_EN0, rd32(c.clk, CLK_SYS_EN0) | 1u << 16);
	if (rd32(c.clk, CLK_CECB_CTRL0) != 0xd02db2dc || rd32(c.clk, CLK_CECB_CTRL1) != 0x0000a007) {
		printf("cec: clock was %#x %#x\n", rd32(c.clk, CLK_CECB_CTRL0), rd32(c.clk, CLK_CECB_CTRL1));
		wr32(c.clk, CLK_CECB_CTRL1, 0x0000a007);
		wr32(c.clk, CLK_CECB_CTRL0, 0x102db2dc);
		wr32(c.clk, CLK_CECB_CTRL0, 0x902db2dc);
		wr32(c.clk, CLK_CECB_CTRL0, 0xd02db2dc);
	}
	/* GPIOH_3 as CEC-B */
	v = rd32(c.pad, PIN_MUX_B);
	if (((v >> 12) & 0xf) != 2) {
		printf("cec: pin GPIOH_3 had function %u\n", (v >> 12) & 0xf);
		wr32(c.pad, PIN_MUX_B, (v & ~(0xfu << 12)) | 2u << 12);
	}
	controller_reset();
	printf("cec: controller up (gen_cntl %#x)\n", rd32(c.ao, AO_CECB_GEN_CNTL));

	if (kernel_thread_start(cec_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_set_thread_name(thread, "cec");
		thread_deallocate(thread);
	}
}

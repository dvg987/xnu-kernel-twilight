/*
 * The Amlogic box's internal USB host (the Corigine "CRG" controller at 0xfe340000, an xHCI) and the one device on
 * it: the Amlogic W2, Wi-Fi and Bluetooth (1b8e:0641, high speed). Boot-arg w2=N: steps 1 to N.
 *
 *   1  the controller: clock, resets, the USB2 PHY, host mode, xHCI up; the W2 powered (GPIOX_7) and its port reset
 *   2  the W2 enumerated: address, descriptors, configuration; endpoint 1 OUT and 4 IN (commands, answers)
 *   3  a register of the chip read, by a vendor request on endpoint 0 and by a command on endpoints 1/4
 *   4  the Wi-Fi firmware loaded (fw/w2_iccm.bin, w2_dccm1.bin, w2_dccm2.bin of the small image, made by
 *      tools/w2-fw.py) and started, with the radio's AGC microcode (fw/w2_agc.bin) before it; the chip ready when it
 *      answers its vendor id at 0x00a10004
 *   5+ the Wi-Fi driver on top (hakios.w2.c), through the calls of sys/hakios.xhci.h
 *
 * One device, polled from a kernel thread, no interrupt: a ring of commands, one of events, one for each endpoint
 * used, contexts and buffers in one physically contiguous block below 4 GB (the controller has no 64-bit
 * addressing) touched only through an uncached mapping (it is not coherent). The bring-up of the controller and
 * the PHY is Amlogic's Linux one for S7D (common_drivers: usb/crg/crg_drd.c, usb/crgdrdphy/phy-aml-crg-drd-usb2.c);
 * the W2's commands are those of Amlogic's driver (amlogic-drivers_w2: aml_bus/w2_usb.c, aml_platform.c).
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/hakios.input.h>
#include <sys/hakios.xhci.h>
#include <kern/locks.h>
#include <vm/vm_kern.h>
#include <arm/caches_internal.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern uint64_t kvtophys(vm_offset_t va);
extern void IOSleep(unsigned int milliseconds);
extern void IODelay(unsigned int microseconds);

/* the SoC around the controller */
#define CLK_BASE        0xfe000000UL
#define CLK_SYS_EN0_REG1        0x48            /* bit 26: sys_usb */
#define RST_BASE        0xfe002000UL
#define RST_PULSE0      0x00
#define RST_LEVEL0      0x40                    /* 1 = out of reset: bit 4 USB1 (this host), bit 9 USBPHY21 */
#define PAD_BASE        0xfe004000UL
#define MUX_X0_7        0x0c                    /* GPIOX_7 = bits 31:28 */
#define MUX_H8          0x30                    /* GPIOH_8 = bits 3:0 */
#define GPIOX_OUT       0x104
#define GPIOX_DIR       0x108                   /* 1 = input */
#define GPIOH_OUT       0x144
#define GPIOH_DIR       0x148
#define TRIM_BASE       0xfe010000UL
#define TRIM_REG        0x330

#define CRG_BASE        0xfe340000UL
#define CRG_SIZE        0x10000                 /* the xHCI, its vendor registers, the PHY's (0x8000, 0xc000) */
#define CRG_MODE        0x20fc                  /* bit 0: 1 = device */
#define CRG_NAK_RETRY   0x2110                  /* bits 20:13 */
#define PHY_R0          0x8000
#define PHY_R1          0x8004                  /* bit 0: ready */
#define PHY_CFG         0xc000

/* xHCI */
#define CMD_RUN         (1u << 0)
#define CMD_HCRST       (1u << 1)
#define STS_HCH         (1u << 0)
#define STS_CNR         (1u << 11)
#define PORTSC_CCS      (1u << 0)
#define PORTSC_PED      (1u << 1)
#define PORTSC_PR       (1u << 4)
#define PORTSC_PP       (1u << 9)
#define PORTSC_SPEED(v) (((v) >> 10) & 0xf)
#define PORTSC_PRC      (1u << 21)
#define PORTSC_KEEP     0x4e00ffe9u             /* read-only and read-write bits: written back as they are */
#define PORTSC_CHANGES  (0x7fu << 17)

#define TRB_NORMAL      1
#define TRB_SETUP       2
#define TRB_DATA        3
#define TRB_STATUS      4
#define TRB_LINK        6
#define TRB_ENABLE_SLOT 9
#define TRB_ADDRESS     11
#define TRB_CONFIGURE   12
#define TRB_NOOP_CMD    23
#define EV_TRANSFER     32
#define EV_COMMAND      33
#define EV_PORT         34
#define TRB_CYCLE       (1u << 0)
#define TRB_TC          (1u << 1)
#define TRB_ISP         (1u << 2)
#define TRB_IOC         (1u << 5)
#define TRB_IDT         (1u << 6)
#define TRB_TYPE(t)     ((uint32_t)(t) << 10)
#define CC_SUCCESS      1
#define CC_SHORT        13

#define N_TRBS          256                     /* each ring: one page, the last TRB a link */

/* the block: offsets of its parts */
#define BLK_SIZE        0x60000
#define OFF_DCBAA       0x00000
#define OFF_CMD         0x01000
#define OFF_EVENTS      0x02000
#define OFF_ERST        0x03000
#define OFF_IN_CTX      0x04000
#define OFF_OUT_CTX     0x05000
#define OFF_EP0         0x06000
#define OFF_EP1_OUT     0x07000
#define OFF_EP4_IN      0x08000
#define OFF_SCRATCH_ARR 0x09000
#define OFF_CTRL_BUF    0x0a000
#define OFF_CBW         0x0b000
#define OFF_IN_BUF      0x0c000
#define OFF_EP5_IN      0x0d000
#define OFF_CFM         0x0e000                 /* 2 KiB: the TX confirmations */
#define OFF_SCRATCH     0x10000                 /* up to 32 pages */
#define OFF_DATA        0x30000                 /* 64 KiB: one transfer of the firmware, of an SRAM read or write */
#define OFF_TX          0x40000                 /* 64 KiB: the pages of one packet command */
#define OFF_EP2_OUT     0x50000                 /* Bluetooth's endpoints */
#define OFF_EP2_IN      0x51000
#define MAX_SCRATCH     32

#define DCI_EP0         1
#define DCI_EP1_OUT     2
#define DCI_EP2_OUT     4
#define DCI_EP2_IN      5
#define DCI_EP4_IN      9
#define DCI_EP5_IN      11

/* the W2's commands (aml_bus/usb_common.h) */
#define W2_SIG          0x43425355
#define W2_TAG          0x5da729a0
#define W2_DOWNLOAD     0xc1
#define W2_START        0xc2
#define W2_READ_REG     0xc4
#define W2_WRITE_REG    0xc5
#define W2_WRITE_PACKET 0xc7
#define W2_WRITE_SRAM   0xc8
#define W2_READ_SRAM    0xc9
#define W2_USB_IRQ      0xcd
#define W2_VENDOR_ID_REG 0x00a10004
#define W2_VENDOR_ID    0x1b8e

struct ring {
	uint32_t off;           /* in the block */
	uint32_t idx;
	uint32_t cycle;
};

static struct {
	vm_offset_t blk, regs, clk, rst, pad, trim;
	uint64_t pa;
	vm_offset_t op, rt, db;
	uint32_t ctx_size, ports, step;
	struct ring cmd, ep0, ep1, ep4, ep5, ep2o, ep2i;
	uint32_t ev_idx, ev_cycle;
	uint32_t slot, port, speed;
	uint32_t ev[4];         /* the last event taken */
	uint16_t irq_count;     /* wIndex of the interrupt poll: a counter */
	bool ready;             /* the firmware is up: the calls of sys/hakios.xhci.h may be made */
} x;

static LCK_GRP_DECLARE(xhci_lck_grp, "hakios-xhci");
static LCK_MTX_DECLARE(xhci_lock, &xhci_lck_grp);       /* all traffic on the bus: one transfer at a time */

static inline uint32_t rd(vm_offset_t base, uint32_t off) { return *(volatile uint32_t *)(base + off); }
static inline void wr(vm_offset_t base, uint32_t off, uint32_t v) { *(volatile uint32_t *)(base + off) = v; }
static inline void set(vm_offset_t base, uint32_t off, uint32_t bits) { wr(base, off, rd(base, off) | bits); }
static inline void clr(vm_offset_t base, uint32_t off, uint32_t bits) { wr(base, off, rd(base, off) & ~bits); }
static inline volatile uint32_t *mem32(uint32_t off) { return (volatile uint32_t *)(x.blk + off); }
static inline uint32_t pa32(uint32_t off) { return (uint32_t)(x.pa + off); }

static void
wr64(vm_offset_t base, uint32_t off, uint64_t v)
{
	wr(base, off, (uint32_t)v);
	wr(base, off + 4, (uint32_t)(v >> 32));
}

/* ---- the controller's surroundings ---- */

static void
phy_init(void)
{
	uint32_t t, cali, v;
	int i;

	set(x.clk, CLK_SYS_EN0_REG1, 1u << 26);
	/* VBUS of the port: GPIOH_8, output, high */
	clr(x.pad, MUX_H8, 0xfu);
	set(x.pad, GPIOH_OUT, 1u << 8);
	clr(x.pad, GPIOH_DIR, 1u << 8);
	/* the PHY out of reset, then as a host: POR, host, ID pull-up, drive VBUS */
	set(x.rst, RST_LEVEL0, 1u << 9);
	set(x.regs, PHY_R0, 0x39);
	IODelay(10);
	clr(x.rst, RST_LEVEL0, 1u << 9);
	IODelay(100);
	set(x.rst, RST_LEVEL0, 1u << 9);
	IODelay(50);
	/* the trim of the chip, from its fuses */
	t = rd(x.trim, TRIM_REG);
	cali = (t & (1u << 12)) ? ((t >> 8) & 0xf) + 2 : 9;
	if (cali > 12) {
		cali = 12;
	}
	wr(x.regs, PHY_CFG + 0x10, (rd(x.regs, PHY_CFG + 0x10) & ~0xfffu) | ((1u << cali) - 1));
	clr(x.regs, PHY_CFG + 0x54, 1u << 2);
	for (i = 0; i < 200 && !(rd(x.regs, PHY_R1) & 1); i++) {
		IODelay(5);
	}
	printf("xhci: phy %s (trim %#x, cali %u)\n", (rd(x.regs, PHY_R1) & 1) ? "ready" : "not ready", t, cali);
	/* its PLL (pll-version 3) */
	set(x.regs, PHY_CFG + 0x40, 1u << 27);
	IODelay(10);
	for (i = 0; i < 5; i++) {
		wr(x.regs, PHY_CFG + 0x40, 0x549540 | 1u << 27);
		IODelay(10);
		set(x.regs, PHY_CFG + 0x40, 1u << 26);
		IODelay(20);
		set(x.regs, PHY_CFG + 0x40, 1u << 25);
		IODelay(20);
		set(x.regs, PHY_CFG + 0x40, 1u << 24);
		for (int j = 0; j < 5 && !(rd(x.regs, PHY_CFG + 0x40) & (1u << 31)); j++) {
			IODelay(20);
		}
		if (rd(x.regs, PHY_CFG + 0x40) & (1u << 31)) {
			break;
		}
	}
	v = rd(x.regs, PHY_CFG + 0x40);
	printf("xhci: phy pll %s (%#x, %d retries)\n", (v & (1u << 31)) ? "locked" : "NOT locked", v, i);
	wr(x.regs, PHY_CFG + 0x0c, 0x39);
	clr(x.regs, PHY_CFG + 0x38, 1u << 27);
	wr(x.regs, PHY_CFG + 0x50, 0x7e18);
	/* the controller out of reset, as a host */
	set(x.rst, RST_LEVEL0, 1u << 4);
	IODelay(100);
	clr(x.regs, CRG_MODE, 1u << 0);
}

/* the W2's supply: GPIOX_7, off for 200 ms, then on */
static void
w2_power_cycle(void)
{
	clr(x.pad, MUX_X0_7, 0xfu << 28);
	clr(x.pad, GPIOX_OUT, 1u << 7);
	clr(x.pad, GPIOX_DIR, 1u << 7);
	IOSleep(200);
	set(x.pad, GPIOX_OUT, 1u << 7);
	IOSleep(200);
}

/* ---- rings and events ---- */

static void
ring_init(struct ring *r, uint32_t off)
{
	volatile uint32_t *t = mem32(off);

	for (uint32_t i = 0; i < N_TRBS * 4; i++) {
		t[i] = 0;
	}
	t[(N_TRBS - 1) * 4 + 0] = pa32(off);
	t[(N_TRBS - 1) * 4 + 1] = 0;
	t[(N_TRBS - 1) * 4 + 2] = 0;
	t[(N_TRBS - 1) * 4 + 3] = TRB_TYPE(TRB_LINK) | TRB_TC;    /* cycle bit set when the ring wraps */
	r->off = off;
	r->idx = 0;
	r->cycle = 1;
}

static void
ring_put(struct ring *r, uint32_t d0, uint32_t d1, uint32_t d2, uint32_t d3)
{
	volatile uint32_t *t = mem32(r->off + r->idx * 16);

	t[0] = d0;
	t[1] = d1;
	t[2] = d2;
	__builtin_arm_dsb(0xf);
	t[3] = (d3 & ~TRB_CYCLE) | r->cycle;
	if (++r->idx == N_TRBS - 1) {
		volatile uint32_t *link = mem32(r->off + (N_TRBS - 1) * 16);

		link[3] = (link[3] & ~TRB_CYCLE) | r->cycle;
		r->idx = 0;
		r->cycle ^= 1;
	}
	__builtin_arm_dsb(0xf);
}

/* the next event, or false after `ms' milliseconds */
static bool
next_event(unsigned ms)
{
	for (unsigned waited = 0;; waited++) {
		volatile uint32_t *e = mem32(OFF_EVENTS + x.ev_idx * 16);

		if ((e[3] & TRB_CYCLE) == x.ev_cycle) {
			for (int i = 0; i < 4; i++) {
				x.ev[i] = e[i];
			}
			if (++x.ev_idx == N_TRBS) {
				x.ev_idx = 0;
				x.ev_cycle ^= 1;
			}
			wr64(x.rt, 0x38, (uint64_t)pa32(OFF_EVENTS + x.ev_idx * 16) | (1u << 3));
			return true;
		}
		if (waited >= ms * 4) {
			return false;
		}
		IODelay(250);
	}
}

static uint32_t
event_type(void)
{
	return (x.ev[3] >> 10) & 0x3f;
}

/* waits for an event of `type', setting the others aside (port changes are only logged) */
static bool
wait_event(uint32_t type, unsigned ms)
{
	uint64_t deadline, now;

	clock_interval_to_deadline(ms, NSEC_PER_MSEC, &deadline);
	for (;;) {
		clock_get_uptime(&now);
		if (now > deadline || !next_event(ms)) {
			return false;
		}
		if (event_type() == type) {
			return true;
		}
		if (event_type() == EV_PORT) {
			continue;
		}
		printf("xhci: event %u set aside (%08x %08x %08x %08x)\n", event_type(), x.ev[0], x.ev[1], x.ev[2], x.ev[3]);
	}
}

/* a command; its completion code, or 0 for no answer */
static uint32_t
command(uint32_t d0, uint32_t d1, uint32_t d2, uint32_t d3)
{
	ring_put(&x.cmd, d0, d1, d2, d3);
	wr(x.db, 0, 0);
	if (!wait_event(EV_COMMAND, 1000)) {
		printf("xhci: command %u: no answer\n", (d3 >> 10) & 0x3f);
		return 0;
	}
	return x.ev[2] >> 24;
}

/* ---- transfers ---- */

/* waits for the end of a transfer on `dci'; the bytes not moved in *residual. Its completion code */
static uint32_t
transfer_done(uint32_t dci, unsigned ms, uint32_t *residual)
{
	for (;;) {
		if (!wait_event(EV_TRANSFER, ms)) {
			return 0;
		}
		if (((x.ev[3] >> 16) & 0x1f) == dci && (x.ev[3] >> 24) == x.slot) {
			break;
		}
	}
	if (residual != NULL) {
		*residual = x.ev[2] & 0xffffff;
	}
	return x.ev[2] >> 24;
}

/* a request on endpoint 0, with up to 4 KiB of data in or out through the control buffer. Bytes moved, or -1 */
static int
control(uint8_t type, uint8_t request, uint16_t value, uint16_t index, uint16_t length)
{
	bool in = (type & 0x80) != 0;
	uint32_t trt = length == 0 ? 0 : in ? 3 : 2, cc, residual = 0;

	ring_put(&x.ep0, (uint32_t)type | (uint32_t)request << 8 | (uint32_t)value << 16, (uint32_t)index | (uint32_t)length << 16,
	    8, TRB_TYPE(TRB_SETUP) | TRB_IDT | trt << 16);
	if (length != 0) {
		ring_put(&x.ep0, pa32(OFF_CTRL_BUF), 0, length, TRB_TYPE(TRB_DATA) | (in ? 1u << 16 : 0) | TRB_ISP);
	}
	ring_put(&x.ep0, 0, 0, 0, TRB_TYPE(TRB_STATUS) | TRB_IOC | ((length == 0 || !in) ? 1u << 16 : 0));
	wr(x.db, x.slot * 4, DCI_EP0);
	for (;;) {
		cc = transfer_done(DCI_EP0, 1000, &residual);
		if (cc != CC_SHORT) {
			break;
		}
		/* a short data stage: the status stage's event follows */
	}
	if (cc != CC_SUCCESS) {
		printf("xhci: control %02x %02x %04x %04x %u: completion %u\n", type, request, value, index, length, cc);
		return -1;
	}
	return length;
}

/* one bulk transfer of the block's part at `off' on `dci'; bytes moved, or -1 */
static int
bulk(uint32_t dci, struct ring *r, uint32_t off, uint32_t length, unsigned ms)
{
	uint32_t cc, residual = 0;

	ring_put(r, pa32(off), 0, length, TRB_TYPE(TRB_NORMAL) | TRB_IOC | TRB_ISP);
	wr(x.db, x.slot * 4, dci);
	cc = transfer_done(dci, ms, &residual);
	if (cc != CC_SUCCESS && cc != CC_SHORT) {
		printf("xhci: bulk on %u, %u bytes: completion %u\n", dci, length, cc);
		return -1;
	}
	return (int)(length - residual);
}

/* ---- the controller ---- */

static bool
controller_up(void)
{
	uint32_t caplen = rd(x.regs, 0) & 0xff, hcs1 = rd(x.regs, 0x04), hcs2 = rd(x.regs, 0x08), hcc1 = rd(x.regs, 0x10);
	uint32_t scratch = ((hcs2 >> 21) & 0x1f) << 5 | ((hcs2 >> 27) & 0x1f);

	x.op = x.regs + caplen;
	x.rt = x.regs + (rd(x.regs, 0x18) & ~0x1fu);
	x.db = x.regs + (rd(x.regs, 0x14) & ~0x3u);
	x.ctx_size = (hcc1 & (1u << 2)) ? 64 : 32;
	x.ports = hcs1 >> 24;
	printf("xhci: version %#x, %u slots, %u ports, %u scratchpad pages, %u-byte contexts, 64-bit %s\n",
	    rd(x.regs, 0) >> 16, hcs1 & 0xff, x.ports, scratch, x.ctx_size, (hcc1 & 1) ? "yes" : "no");
	if (scratch > MAX_SCRATCH) {
		printf("xhci: too many scratchpad pages\n");
		return false;
	}

	clr(x.op, 0x00, CMD_RUN);
	for (int i = 0; i < 100 && !(rd(x.op, 0x04) & STS_HCH); i++) {
		IOSleep(1);
	}
	set(x.op, 0x00, CMD_HCRST);
	for (int i = 0; i < 1000 && (rd(x.op, 0x00) & CMD_HCRST); i++) {
		IOSleep(1);
	}
	for (int i = 0; i < 1000 && (rd(x.op, 0x04) & STS_CNR); i++) {
		IOSleep(1);
	}
	if (rd(x.op, 0x04) & STS_CNR) {
		printf("xhci: not ready after reset (status %#x)\n", rd(x.op, 0x04));
		return false;
	}
	wr(x.regs, CRG_NAK_RETRY, (rd(x.regs, CRG_NAK_RETRY) & ~(0xffu << 13)) | 5u << 13);

	for (uint32_t i = 0; i < BLK_SIZE / 4; i++) {
		mem32(0)[i] = 0;
	}
	for (uint32_t i = 0; i < scratch; i++) {
		mem32(OFF_SCRATCH_ARR)[i * 2] = pa32(OFF_SCRATCH + i * 0x1000);
	}
	mem32(OFF_DCBAA)[0] = scratch ? pa32(OFF_SCRATCH_ARR) : 0;
	wr(x.op, 0x38, 1);                                      /* one slot */
	wr64(x.op, 0x30, pa32(OFF_DCBAA));
	ring_init(&x.cmd, OFF_CMD);
	wr64(x.op, 0x18, (uint64_t)pa32(OFF_CMD) | 1);
	/* interrupter 0: its event ring, one segment */
	mem32(OFF_ERST)[0] = pa32(OFF_EVENTS);
	mem32(OFF_ERST)[1] = 0;
	mem32(OFF_ERST)[2] = N_TRBS;
	x.ev_idx = 0;
	x.ev_cycle = 1;
	__builtin_arm_dsb(0xf);
	wr(x.rt, 0x28, 1);
	wr64(x.rt, 0x38, pa32(OFF_EVENTS));
	wr64(x.rt, 0x30, pa32(OFF_ERST));
	wr(x.rt, 0x20, 0);                                      /* no interrupts: polled */
	set(x.op, 0x00, CMD_RUN);
	for (int i = 0; i < 100 && (rd(x.op, 0x04) & STS_HCH); i++) {
		IOSleep(1);
	}
	if (command(0, 0, 0, TRB_TYPE(TRB_NOOP_CMD)) != CC_SUCCESS) {
		printf("xhci: the command ring does not answer (status %#x)\n", rd(x.op, 0x04));
		return false;
	}
	printf("xhci: running\n");
	return true;
}

static vm_offset_t
portsc(uint32_t port)
{
	return x.op + 0x400 + 0x10 * (port - 1);
}

/* the port the W2 is on, reset and enabled; false if none comes up */
static bool
port_up(void)
{
	uint32_t v;

	for (uint32_t p = 1; p <= x.ports; p++) {
		v = rd(portsc(p), 0);
		if (!(v & PORTSC_PP)) {
			wr(portsc(p), 0, (v & PORTSC_KEEP) | PORTSC_PP);
		}
	}
	for (int tries = 0; tries < 100 && x.port == 0; tries++) {
		IOSleep(20);
		for (uint32_t p = 1; p <= x.ports; p++) {
			if (rd(portsc(p), 0) & PORTSC_CCS) {
				x.port = p;
				break;
			}
		}
	}
	for (uint32_t p = 1; p <= x.ports; p++) {
		printf("xhci: port %u: %#x\n", p, rd(portsc(p), 0));
	}
	if (x.port == 0) {
		printf("xhci: nothing connected\n");
		return false;
	}
	v = rd(portsc(x.port), 0);
	wr(portsc(x.port), 0, (v & PORTSC_KEEP) | PORTSC_PR);
	for (int i = 0; i < 100 && !(rd(portsc(x.port), 0) & PORTSC_PRC); i++) {
		IOSleep(2);
	}
	v = rd(portsc(x.port), 0);
	wr(portsc(x.port), 0, (v & PORTSC_KEEP) | (v & PORTSC_CHANGES));
	IOSleep(20);
	v = rd(portsc(x.port), 0);
	x.speed = PORTSC_SPEED(v);
	printf("xhci: port %u after reset: %#x (%s, speed %u)\n", x.port, v, (v & PORTSC_PED) ? "enabled" : "NOT enabled", x.speed);
	return (v & PORTSC_PED) != 0;
}

static volatile uint32_t *
in_ctx(uint32_t index)          /* 0 control, 1 slot, 1 + dci endpoints */
{
	return mem32(OFF_IN_CTX + index * x.ctx_size);
}

static void
ep_ctx(uint32_t dci, uint32_t type, uint32_t mps, struct ring *r, uint32_t avg)
{
	volatile uint32_t *e = in_ctx(1 + dci);

	e[0] = 0;
	e[1] = 3u << 1 | type << 3 | mps << 16;                 /* 3 errors allowed */
	e[2] = pa32(r->off) | 1;
	e[3] = 0;
	e[4] = avg;
}

/* step 2 */
static bool
enumerate(void)
{
	uint32_t cc, mps0 = x.speed == 3 ? 64 : 8;
	volatile uint8_t *buf = (volatile uint8_t *)(x.blk + OFF_CTRL_BUF);
	uint8_t dev[18];
	uint16_t total;

	cc = command(0, 0, 0, TRB_TYPE(TRB_ENABLE_SLOT));
	if (cc != CC_SUCCESS) {
		printf("xhci: enable slot: completion %u\n", cc);
		return false;
	}
	x.slot = x.ev[3] >> 24;
	mem32(OFF_DCBAA)[x.slot * 2] = pa32(OFF_OUT_CTX);
	for (uint32_t i = 0; i < 0x1000 / 4; i++) {
		mem32(OFF_IN_CTX)[i] = 0;
	}
	in_ctx(0)[1] = 1u << 0 | 1u << 1;                       /* add: the slot, endpoint 0 */
	in_ctx(1)[0] = x.speed << 20 | 1u << 27;
	in_ctx(1)[1] = x.port << 16;
	ring_init(&x.ep0, OFF_EP0);
	ep_ctx(DCI_EP0, 4, mps0, &x.ep0, 8);
	__builtin_arm_dsb(0xf);
	cc = command(pa32(OFF_IN_CTX), 0, 0, TRB_TYPE(TRB_ADDRESS) | x.slot << 24);
	if (cc != CC_SUCCESS) {
		printf("xhci: address device: completion %u\n", cc);
		return false;
	}
	printf("xhci: slot %u, address %u\n", x.slot, mem32(OFF_OUT_CTX)[3] & 0xff);

	if (control(0x80, 6, 0x0100, 0, 18) < 0) {
		return false;
	}
	for (int i = 0; i < 18; i++) {
		dev[i] = buf[i];
	}
	printf("xhci: device %04x:%04x, usb %x.%02x, class %u, endpoint 0 takes %u\n", dev[8] | dev[9] << 8,
	    dev[10] | dev[11] << 8, dev[3], dev[2], dev[4], dev[7]);
	if (control(0x80, 6, 0x0200, 0, 9) < 0) {
		return false;
	}
	total = buf[2] | buf[3] << 8;
	if (total > 0x1000 || control(0x80, 6, 0x0200, 0, total) < 0) {
		return false;
	}
	for (uint32_t i = 0; i + 2 <= total && buf[i] != 0; i += buf[i]) {
		if (buf[i + 1] == 5) {
			printf("xhci:   endpoint %#04x type %u, %u bytes\n", buf[i + 2], buf[i + 3] & 3, buf[i + 4] | buf[i + 5] << 8);
		}
	}
	if (control(0x00, 9, buf[5], 0, 0) < 0) {                 /* SET_CONFIGURATION */
		return false;
	}

	/* endpoint 1 OUT, endpoint 4 IN: bulk, 512 bytes */
	for (uint32_t i = 0; i < 0x1000 / 4; i++) {
		mem32(OFF_IN_CTX)[i] = 0;
	}
	in_ctx(0)[1] = 1u << 0 | 1u << DCI_EP1_OUT | 1u << DCI_EP2_OUT | 1u << DCI_EP2_IN | 1u << DCI_EP4_IN | 1u << DCI_EP5_IN;
	in_ctx(1)[0] = x.speed << 20 | (uint32_t)DCI_EP5_IN << 27;
	in_ctx(1)[1] = x.port << 16;
	ring_init(&x.ep1, OFF_EP1_OUT);
	ring_init(&x.ep4, OFF_EP4_IN);
	ring_init(&x.ep5, OFF_EP5_IN);
	ring_init(&x.ep2o, OFF_EP2_OUT);
	ring_init(&x.ep2i, OFF_EP2_IN);
	ep_ctx(DCI_EP1_OUT, 2, 512, &x.ep1, 1024);
	ep_ctx(DCI_EP4_IN, 6, 512, &x.ep4, 1024);
	ep_ctx(DCI_EP5_IN, 6, 512, &x.ep5, 1024);
	ep_ctx(DCI_EP2_OUT, 2, 512, &x.ep2o, 1024);
	ep_ctx(DCI_EP2_IN, 6, 512, &x.ep2i, 1024);
	__builtin_arm_dsb(0xf);
	cc = command(pa32(OFF_IN_CTX), 0, 0, TRB_TYPE(TRB_CONFIGURE) | x.slot << 24);
	if (cc != CC_SUCCESS) {
		printf("xhci: configure endpoint: completion %u\n", cc);
		return false;
	}
	printf("xhci: configured; endpoints 1 OUT, 4 IN, 5 IN (Wi-Fi) and 2 OUT/IN (Bluetooth) ready\n");
	return true;
}

/* ---- the W2 ---- */

/* its 512-byte command block (struct crg_msc_cbw, packed: the four words of cdb are not aligned) */
static void
w2_cbw(bool in, uint32_t length, uint32_t cmd, uint32_t addr, uint32_t flag, uint32_t data_len)
{
	volatile uint8_t *b = (volatile uint8_t *)(x.blk + OFF_CBW);
	uint32_t f[3] = { W2_SIG, W2_TAG, length };

	for (int i = 0; i < 512; i++) {
		b[i] = 0;
	}
	for (int i = 0; i < 4; i++) {
		b[0 + i] = (uint8_t)(f[0] >> (8 * i));
		b[4 + i] = (uint8_t)(f[1] >> (8 * i));
		b[8 + i] = (uint8_t)(f[2] >> (8 * i));
		b[15 + i] = (uint8_t)(cmd >> (8 * i));
		b[19 + i] = (uint8_t)(addr >> (8 * i));
		b[23 + i] = (uint8_t)(flag >> (8 * i));
		b[27 + i] = (uint8_t)(data_len >> (8 * i));
	}
	b[12] = in ? 0x80 : 0;
	b[13] = 0;
	b[14] = 16;
	__builtin_arm_dsb(0xf);
}

/*
 * Which endpoints the commands go over: Wi-Fi's (1 OUT, answers on 4 IN) or Bluetooth's (2 OUT, answers on 2 IN).
 * Set for one whole exchange, under the lock.
 */
struct pipe {
	uint32_t out_dci, in_dci;
	struct ring *out, *in;
};
static struct pipe pipe_wifi = { DCI_EP1_OUT, DCI_EP4_IN, &x.ep1, &x.ep4 };
static struct pipe pipe_bt = { DCI_EP2_OUT, DCI_EP2_IN, &x.ep2o, &x.ep2i };
static struct pipe *pp = &pipe_wifi;

static bool
w2_read(uint32_t addr, uint32_t *value)
{
	volatile uint8_t *in = (volatile uint8_t *)(x.blk + OFF_IN_BUF);

	w2_cbw(true, 4, W2_READ_REG, addr, 0, 4);
	if (bulk(pp->out_dci, pp->out, OFF_CBW, 512, 3000) != 512) {
		return false;
	}
	if (bulk(pp->in_dci, pp->in, OFF_IN_BUF, 4, 3000) < 4) {
		return false;
	}
	*value = in[0] | in[1] << 8 | in[2] << 16 | (uint32_t)in[3] << 24;
	return true;
}

static bool
w2_write(uint32_t addr, uint32_t value)
{
	w2_cbw(false, 0, W2_WRITE_REG, addr, value, 4);
	return bulk(pp->out_dci, pp->out, OFF_CBW, 512, 3000) == 512;
}

/* between the block (device memory to us: no memcpy, whose wide or unaligned accesses may fault) and memory */
static void
dma_put(uint32_t off, const uint8_t *src, uint32_t n)
{
	uint32_t i = 0;

	for (; i + 4 <= n; i += 4) {
		mem32(off)[i / 4] = src[i] | src[i + 1] << 8 | src[i + 2] << 16 | (uint32_t)src[i + 3] << 24;
	}
	for (; i < n; i++) {
		((volatile uint8_t *)(x.blk + off))[i] = src[i];
	}
	__builtin_arm_dsb(0xf);
}

static void
dma_get(uint32_t off, uint8_t *dst, uint32_t n)
{
	uint32_t i = 0;

	for (; i + 4 <= n; i += 4) {
		uint32_t w = mem32(off)[i / 4];

		dst[i] = (uint8_t)w;
		dst[i + 1] = (uint8_t)(w >> 8);
		dst[i + 2] = (uint8_t)(w >> 16);
		dst[i + 3] = (uint8_t)(w >> 24);
	}
	for (; i < n; i++) {
		dst[i] = ((volatile uint8_t *)(x.blk + off))[i];
	}
}

/* `n' bytes into the chip's memory at `addr', in pieces of 64 KiB: a write-SRAM command, then the data */
static bool
sram_write(uint32_t addr, const uint8_t *src, uint32_t n)
{
	for (uint32_t done = 0; done < n;) {
		uint32_t piece = n - done > 0x10000 ? 0x10000 : n - done;

		w2_cbw(false, piece, W2_WRITE_SRAM, addr + done, 0, piece);
		if (bulk(pp->out_dci, pp->out, OFF_CBW, 512, 3000) != 512) {
			return false;
		}
		dma_put(OFF_DATA, src + done, piece);
		if (bulk(pp->out_dci, pp->out, OFF_DATA, piece, 3000) != (int)piece) {
			return false;
		}
		done += piece;
	}
	return true;
}

static bool
sram_read(uint32_t addr, uint8_t *dst, uint32_t n)
{
	for (uint32_t done = 0; done < n;) {
		uint32_t piece = n - done > 0x10000 ? 0x10000 : n - done;
		int got;

		w2_cbw(true, piece, W2_READ_SRAM, addr + done, 0, piece);
		if (bulk(pp->out_dci, pp->out, OFF_CBW, 512, 3000) != 512) {
			return false;
		}
		got = bulk(pp->in_dci, pp->in, OFF_DATA, piece, 3000);
		if (got != (int)piece) {
			return false;
		}
		dma_get(OFF_DATA, dst + done, piece);
		done += piece;
	}
	return true;
}

/* ---- for the Wi-Fi driver (sys/hakios.xhci.h): each a whole exchange, under the lock ---- */

bool
hakios_w2_read(uint32_t addr, uint32_t *value)
{
	bool ok;

	lck_mtx_lock(&xhci_lock);
	ok = x.ready && w2_read(addr, value);
	lck_mtx_unlock(&xhci_lock);
	return ok;
}

bool
hakios_w2_write(uint32_t addr, uint32_t value)
{
	bool ok;

	lck_mtx_lock(&xhci_lock);
	ok = x.ready && w2_write(addr, value);
	lck_mtx_unlock(&xhci_lock);
	return ok;
}

bool
hakios_w2_sram_write(uint32_t addr, const void *data, uint32_t length)
{
	bool ok;

	lck_mtx_lock(&xhci_lock);
	ok = x.ready && sram_write(addr, data, length);
	lck_mtx_unlock(&xhci_lock);
	return ok;
}

bool
hakios_w2_sram_read(uint32_t addr, void *data, uint32_t length)
{
	bool ok;

	lck_mtx_lock(&xhci_lock);
	ok = x.ready && sram_read(addr, data, length);
	lck_mtx_unlock(&xhci_lock);
	return ok;
}

/* the chip's "interrupt": a vendor request on endpoint 0 whose index counts up; four words back */
bool
hakios_w2_irq(uint32_t words[4])
{
	uint8_t b[16];
	bool ok;

	lck_mtx_lock(&xhci_lock);
	ok = x.ready && control(0xc2, W2_USB_IRQ, 0, x.irq_count, 16) == 16;
	if (ok) {
		dma_get(OFF_CTRL_BUF, b, 16);
		for (int i = 0; i < 4; i++) {
			words[i] = b[4 * i] | b[4 * i + 1] << 8 | b[4 * i + 2] << 16 | (uint32_t)b[4 * i + 3] << 24;
		}
	}
	x.irq_count++;
	lck_mtx_unlock(&xhci_lock);
	return ok;
}

/* pages of frames (txdesc_host + payload each, padded to 4), `count' <= 25: the packet command, then all the pages */
bool
hakios_w2_packets(const void *pages, const uint16_t *sizes, unsigned count)
{
	volatile uint8_t *b = (volatile uint8_t *)(x.blk + OFF_CBW);
	uint32_t total = 0;
	bool ok;

	if (count == 0 || count > 25) {
		return false;
	}
	for (unsigned i = 0; i < count; i++) {
		total += sizes[i];
	}
	if (total > 0x10000) {
		return false;
	}
	lck_mtx_lock(&xhci_lock);
	for (int i = 0; i < 512; i++) {
		b[i] = 0;
	}
	/* the sizes are laid over the command block where its fields are not looked at (w2_usb.c, auc_send_frame) */
	for (unsigned i = 0; i < count; i++) {
		uint32_t at = i < 6 ? 2 * i : i < 12 ? 19 + 2 * (i - 6) : i == 12 ? 17 : i == 13 ? 13 : 31 + 2 * (i - 14);

		if (i == 13) {
			b[13] = (uint8_t)(sizes[i] >> 8);
			b[14] = (uint8_t)sizes[i];
		} else {
			b[at] = (uint8_t)sizes[i];
			b[at + 1] = (uint8_t)(sizes[i] >> 8);
		}
	}
	b[12] = (uint8_t)count;
	b[15] = W2_WRITE_PACKET;
	__builtin_arm_dsb(0xf);
	ok = x.ready && bulk(DCI_EP1_OUT, &x.ep1, OFF_CBW, 512, 3000) == 512;
	if (ok) {
		dma_put(OFF_TX, pages, total);
		ok = bulk(DCI_EP1_OUT, &x.ep1, OFF_TX, total, 3000) == (int)total;
	}
	lck_mtx_unlock(&xhci_lock);
	return ok;
}

/* the ring of TX confirmations, as it is now (endpoint 5, no command) */
bool
hakios_w2_confirmations(void *data, uint32_t length)
{
	int got = -1;

	if (length > 2048) {
		return false;
	}
	lck_mtx_lock(&xhci_lock);
	if (x.ready) {
		got = bulk(DCI_EP5_IN, &x.ep5, OFF_CFM, length, 3000);
		if (got > 0) {
			dma_get(OFF_CFM, data, (uint32_t)got);
		}
	}
	lck_mtx_unlock(&xhci_lock);
	return got == (int)length;
}

/* ---- the same, for Bluetooth: endpoint 2 ---- */

bool
hakios_w2bt_read(uint32_t addr, uint32_t *value)
{
	bool ok;

	lck_mtx_lock(&xhci_lock);
	pp = &pipe_bt;
	ok = x.ready && w2_read(addr, value);
	pp = &pipe_wifi;
	lck_mtx_unlock(&xhci_lock);
	return ok;
}

bool
hakios_w2bt_write(uint32_t addr, uint32_t value)
{
	bool ok;

	lck_mtx_lock(&xhci_lock);
	pp = &pipe_bt;
	ok = x.ready && w2_write(addr, value);
	pp = &pipe_wifi;
	lck_mtx_unlock(&xhci_lock);
	return ok;
}

bool
hakios_w2bt_sram_write(uint32_t addr, const void *data, uint32_t length)
{
	bool ok;

	lck_mtx_lock(&xhci_lock);
	pp = &pipe_bt;
	ok = x.ready && sram_write(addr, data, length);
	pp = &pipe_wifi;
	lck_mtx_unlock(&xhci_lock);
	return ok;
}

bool
hakios_w2bt_sram_read(uint32_t addr, void *data, uint32_t length)
{
	bool ok;

	lck_mtx_lock(&xhci_lock);
	pp = &pipe_bt;
	ok = x.ready && sram_read(addr, data, length);
	pp = &pipe_wifi;
	lck_mtx_unlock(&xhci_lock);
	return ok;
}

/* step 3 */
static bool
w2_registers(void)
{
	volatile uint8_t *buf = (volatile uint8_t *)(x.blk + OFF_CTRL_BUF);
	uint32_t chip, dpll, vid;

	if (control(0xc2, W2_READ_REG, 0x00f0, 0x0004, 4) < 0) {
		return false;
	}
	chip = buf[0] | buf[1] << 8 | buf[2] << 16 | (uint32_t)buf[3] << 24;
	if (!w2_read(0x00f0101c, &dpll) || !w2_read(W2_VENDOR_ID_REG, &vid)) {
		printf("xhci: w2: chip id %#x; a register read over endpoints 1/4 failed\n", chip);
		return false;
	}
	printf("xhci: w2: chip id %#x, bbpll %s (%#x), vendor id register %#x\n", chip, (dpll >> 31) ? "running" : "not running",
	    dpll, vid);
	return true;
}

/* one piece of the firmware to `addr': a download command, then the data in 64 KiB transfers */
static bool
w2_download(const char *name, uint32_t addr)
{
	uint64_t size = 0;
	const uint8_t *data = hakios_image_file("fw", name, &size, NULL);

	if (data == NULL) {
		printf("xhci: w2: no fw/%s in the small image (tools/w2-fw.py)\n", name);
		return false;
	}
	w2_cbw(false, (uint32_t)size, W2_DOWNLOAD, addr, 0, (uint32_t)size);
	if (bulk(DCI_EP1_OUT, &x.ep1, OFF_CBW, 512, 3000) != 512) {
		return false;
	}
	for (uint64_t off = 0; off < size;) {
		uint32_t n = size - off > 0x10000 ? 0x10000 : (uint32_t)(size - off);
		int moved;

		/* word by word: the block is device memory to us, where memcpy's wide or unaligned stores may fault */
		for (uint32_t i = 0; i < n; i += 4) {
			const uint8_t *s = data + off + i;

			mem32(OFF_DATA)[i / 4] = s[0] | s[1] << 8 | s[2] << 16 | (uint32_t)s[3] << 24;
		}
		__builtin_arm_dsb(0xf);
		moved = bulk(DCI_EP1_OUT, &x.ep1, OFF_DATA, n, 3000);
		if (moved <= 0) {
			printf("xhci: w2: %s: stopped at %llu of %llu bytes\n", name, off, size);
			return false;
		}
		off += (uint32_t)moved;
	}
	printf("xhci: w2: %s: %llu bytes to %#x\n", name, size, addr);
	return true;
}

/* the chip's baseband PLL, which its MAC and system registers run from (bbpll_init, bbpll_start) */
static bool
w2_bbpll(void)
{
	static const uint32_t init[][2] = {
		{ 0x00f01000, 0x00800060 }, { 0x00f01004, 0x00000c02 }, { 0x00f01008, 0x00021f1f }, { 0x00f0100c, 0x00000020 },
		{ 0x00f01010, 0x0000000a }, { 0x00f01014, 0x000000c0 }, { 0x00f0101c, 0x00000000 },
	};
	uint32_t v = 0;

	if (w2_read(0x00f0101c, &v) && (v >> 31)) {
		return true;
	}
	for (unsigned i = 0; i < sizeof(init) / sizeof(init[0]); i++) {
		w2_write(init[i][0], init[i][1]);
	}
	w2_read(0x00f01004, &v);
	w2_write(0x00f01004, v | 1u << 0);                      /* enable */
	IODelay(20);
	w2_write(0x00f01004, (v | 1u << 0) & ~(1u << 1));        /* out of reset */
	IODelay(20);
	w2_read(0x00f0100c, &v);
	w2_write(0x00f0100c, v & ~(1u << 5));                   /* lock detector on */
	w2_read(0x00f0101c, &v);
	printf("xhci: w2: bbpll %s (%#x)\n", (v >> 31) ? "locked" : "NOT locked", v);
	return (v >> 31) != 0;
}

/* the bus between the MAC and the PHY, and the radio's AGC microcode (aml_plat_mpif_sel, aml_plat_agc_load) */
static bool
w2_mpif_agc(void)
{
	uint32_t sig = 0, cfg = 0, rf, agc, clk, ver = 0, agcctl = 0, memclk = 0, v = 0;
	uint64_t size = 0;
	const uint8_t *ucode = hakios_image_file("fw", "w2_agc.bin", &size, NULL);

	w2_read(0x60801000, &sig);
	if ((sig >> 16) != 0xcafe && (sig >> 16) != 0xc0ca && (sig & 0xf) < 3) {
		w2_write(0x60c10030, 0x3);
	}
	w2_read(0x60c00000, &cfg);
	rf = (cfg >> 16) & 0xf;
	if (rf > 2) {
		printf("xhci: w2: unknown radio type %u (%#x)\n", rf, cfg);
		return false;
	}
	agc = rf == 0 ? 0x60c02060 : 0x60c0b390;
	clk = rf == 0 ? 0x60c00848 : (sig >> 16) == 0xc0ca ? 0x60805010 : 0x60c00874;
	if (rf != 0) {
		w2_read(0x60c0b000, &v);
		ver = (v >> 22) & 3;
	}
	if (ucode == NULL || size != 2048) {
		printf("xhci: w2: no fw/w2_agc.bin of 2048 bytes in the small image\n");
		return false;
	}
	w2_read(clk, &memclk);
	w2_read(agc, &agcctl);
	w2_write(agc, agcctl | 1u << 12);
	w2_write(clk, ver > 0 ? memclk | 1u << 29 : memclk & ~(1u << 3));
	if (!sram_write(0x60c0a000, ucode, 2048)) {
		return false;
	}
	if (ver == 1) {
		w2_write(0x60c0b008, 1u << 28);
		for (int i = 0; i < 1000 && w2_read(0x60c0b008, &v) && (v & (1u << 28)); i++) {
			IODelay(10);
		}
		w2_read(0x60c0b238, &v);
		if (!(v & 1)) {
			w2_read(0x60c0b23c, &v);
			printf("xhci: w2: the AGC memory does not check (signature %#x)\n", v);
		}
	}
	w2_write(clk, ver > 0 ? memclk & ~(1u << 29) : memclk | 1u << 3);
	w2_write(agc, agcctl & ~(1u << 12));
	printf("xhci: w2: radio type %u, AGC microcode (version %u) loaded\n", rf, ver);
	return true;
}

/* step 4 */
static bool
w2_firmware(void)
{
	uint32_t vid = 0, misc;

	if (!w2_bbpll()) {
		return false;
	}
	w2_write(0x00a0d090, 0x4f530033);                       /* its CPU at 240 MHz */
	w2_read(0x00a0d080, &misc);
	w2_write(0x00a0d080, misc | 0x30000);                   /* its MAC at 240 MHz */

	/* soft reset of the chip's MAC side (aml_platform_reset, for USB) */
	w2_read(0x608010e0, &misc);
	w2_write(0x608010e0, 0x3);
	w2_read(0x00f02040, &misc);
	w2_write(0x00f02040, misc & ~(3u << 30));
	IOSleep(10);
	w2_read(0x608010e0, &misc);
	if (misc & 1) {
		printf("xhci: w2: soft reset did not finish (%#x)\n", misc);
	}
	w2_write(0x608010e0, misc & ~2u);
	IOSleep(10);

	if (!w2_mpif_agc()) {
		return false;
	}
	if (!w2_download("w2_iccm.bin", 0x00040000) || !w2_download("w2_dccm1.bin", 0x00d00000) ||
	    !w2_download("w2_dccm2.bin", 0x00d28000)) {
		return false;
	}
	w2_cbw(false, 0, W2_START, 0, 0, 0);
	if (bulk(DCI_EP1_OUT, &x.ep1, OFF_CBW, 512, 3000) != 512) {
		return false;
	}
	w2_write(0x608010e0, 1u << 4);                          /* boot ROM on */
	w2_write(0x00f02058, 0x00070000);                       /* the firmware's processor out of reset */
	for (int i = 0; i < 400; i++) {
		if (w2_read(W2_VENDOR_ID_REG, &vid) && (vid & 0xffff) == W2_VENDOR_ID) {
			printf("xhci: w2: the firmware is up after %d ms (vendor id %#x)\n", i * 5, vid);
			return true;
		}
		IOSleep(5);
	}
	printf("xhci: w2: the firmware did not come up (vendor id register %#x)\n", vid);
	return false;
}

static void
xhci_thread(__unused void *arg, __unused wait_result_t wr_)
{
	phy_init();
	w2_power_cycle();
	if (!controller_up() || !port_up() || x.step < 2) {
		return;
	}
	if (!enumerate() || x.step < 3) {
		return;
	}
	if (!w2_registers() || x.step < 4) {
		return;
	}
	if (!w2_firmware() || x.step < 5) {
		return;
	}
	x.ready = true;
	hakios_w2_run(x.step);          /* hakios.w2.c: does not return */
}

void
hakios_xhci_init(void)
{
	vm_offset_t va;
	thread_t thread;

	if (!PE_parse_boot_argn("w2", &x.step, sizeof(x.step)) || x.step == 0) {
		return;
	}
	/* 64 KiB aligned: a TRB's buffer may not cross a 64 KiB boundary, and OFF_DATA holds one of 64 KiB */
	if (kmem_alloc_contig(kernel_map, &va, BLK_SIZE, 0xffff, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("xhci: no memory\n");
		return;
	}
	FlushPoC_DcacheRegion(va, BLK_SIZE);
	x.pa = kvtophys(va);
	if (x.pa >> 32) {
		printf("xhci: block above 4 GB\n");
		return;
	}
	x.blk = ml_io_map(x.pa, BLK_SIZE);
	x.regs = ml_io_map(CRG_BASE, CRG_SIZE);
	x.clk = ml_io_map(CLK_BASE, 0x1000);
	x.rst = ml_io_map(RST_BASE, 0x1000);
	x.pad = ml_io_map(PAD_BASE, 0x1000);
	x.trim = ml_io_map(TRIM_BASE, 0x1000);
	printf("xhci: w2=%u, block at %#llx\n", x.step, x.pa);
	if (kernel_thread_start(xhci_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_set_thread_name(thread, "xhci");
		thread_deallocate(thread);
	}
}

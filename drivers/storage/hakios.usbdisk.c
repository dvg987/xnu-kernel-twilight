/*
 * A USB disk on the Amlogic box's outside port as block devices for the partitions of its MBR: the disk the box
 * boots from when it is not installed on the eMMC.
 *
 *   /dev/usb_boot          partition 1, FAT: what U-Boot loads (read only)
 *   /dev/usb_darwin        partition 2, the APFS system container (read only)
 *   /dev/usb_darwin_data   partition 3, the HFS+ data volume
 *
 * Boot-arg usbdisk=1: the box's port (the Corigine "CRG" dual-role controller at 0xfe350000 as an xHCI host);
 * usbdisk=2: the first xHCI of QEMU's PCI bus (-M virt,highmem-ecam=off), for trying the rest without the box.
 * usbdisk_wait=N: seconds to wait for a disk on the port (10).
 *
 * One device straight on the port (no hub), high or super speed, mass storage class, bulk-only transport, SCSI
 * with 512-byte blocks. Polled, no interrupt, one command at a time: the rings and contexts in one block below
 * 4 GB touched through an uncached mapping, the data through a 64 KiB bounce buffer. The bring-up of the PHY and
 * the controller is that of the inside port (drivers/wifi/hakios.xhci.c), with this port's resets.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <miscfs/devfs/devfs.h>
#include <pexpert/pexpert.h>
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/disk.h>
#include <sys/errno.h>
#include <sys/systm.h>
#include <sys/hakios.usbdisk.h>
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
#define RST_LEVEL0      0x40                    /* 1 = out of reset: bit 5 this controller, bit 8 its PHY */
#define RST_CTRL        (1u << 5)
#define RST_PHY         (1u << 8)
#define PAD_BASE        0xfe004000UL
#define MUX_H8          0x30                    /* GPIOH_8 = bits 3:0: the port's VBUS switch */
#define GPIOH_OUT       0x144
#define GPIOH_DIR       0x148
#define TRIM_BASE       0xfe010000UL
#define TRIM_REG        0x330

#define CRG_BASE        0xfe350000UL
#define CRG_SIZE        0x10000                 /* the xHCI, its vendor registers, the PHY's (0x8000, 0xc000) */
#define CRG_MODE        0x20fc                  /* bit 0: 1 = device */
#define CRG_NAK_RETRY   0x2110                  /* bits 20:13 */
#define PHY_R0          0x8000
#define PHY_R1          0x8004                  /* bit 0: ready */
#define PHY_CFG         0xc000

/* QEMU's virt machine with highmem-ecam=off */
#define PCI_ECAM        0x3f000000UL
#define PCI_MMIO        0x10000000UL

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
#define TRB_RESET_EP    14
#define TRB_STOP_EP     15
#define TRB_SET_DEQ     16
#define TRB_NOOP_CMD    23
#define EV_TRANSFER     32
#define EV_COMMAND      33
#define TRB_CYCLE       (1u << 0)
#define TRB_TC          (1u << 1)
#define TRB_ISP         (1u << 2)
#define TRB_IOC         (1u << 5)
#define TRB_IDT         (1u << 6)
#define TRB_TYPE(t)     ((uint32_t)(t) << 10)
#define CC_SUCCESS      1
#define CC_STALL        6
#define CC_SHORT        13

#define N_TRBS          256                     /* each ring: one page, the last TRB a link */

/* the block: offsets of its parts */
#define BLK_SIZE        0x30000
#define OFF_DCBAA       0x00000
#define OFF_CMD         0x01000
#define OFF_EVENTS      0x02000
#define OFF_ERST        0x03000
#define OFF_IN_CTX      0x04000
#define OFF_OUT_CTX     0x05000
#define OFF_EP0         0x06000
#define OFF_EP_OUT      0x07000
#define OFF_EP_IN       0x08000
#define OFF_SCRATCH_ARR 0x09000
#define OFF_CTRL_BUF    0x0a000
#define OFF_CBW         0x0b000
#define OFF_CSW         0x0b400
#define OFF_SMALL       0x0b800                 /* sense data, inquiry, capacity */
#define OFF_SCRATCH     0x10000                 /* up to 32 pages */
#define MAX_SCRATCH     32

#define DCI_EP0         1

#define SECTOR          512u
#define MAX_XFER        (64u * 1024)            /* one TRB: its buffer may not cross a 64 KiB boundary */
#define MAX_BLOCKS      (MAX_XFER / SECTOR)
#define SPIN_US         2000                    /* an answer is waited for busily this long, then asleep */

#define CBW_SIG         0x43425355
#define CSW_SIG         0x53425355

struct ring {
	uint32_t off;           /* in the block */
	uint32_t idx;
	uint32_t cycle;
};

struct usb_part {
	const char *name;
	uint64_t first, count;  /* in sectors */
	bool writable;
};

static struct {
	vm_offset_t blk, regs, clk, rst, pad, trim;
	uint64_t pa;
	vm_offset_t bounce;
	uint64_t bounce_pa;
	vm_offset_t op, rt, db;
	uint32_t mode, ctx_size, ports;
	struct ring cmd, ep0, out, in;
	uint32_t ev_idx, ev_cycle;
	uint32_t slot, port, speed;
	uint32_t ev[4];         /* the last event taken */
	uint32_t dci_out, dci_in, mps_out, mps_in;
	uint8_t ep_out, ep_in, iface;
	uint32_t tag;
	uint64_t sectors;
	bool ready, gone;
	struct usb_part part[3];
	int bmajor, cmajor;
	uint64_t reads, writes, errors;
} u = {
	.bmajor = -1, .cmajor = -1,
	.part = { { "boot", 0, 0, false }, { "darwin", 0, 0, false }, { "darwin_data", 0, 0, true } },
};

static LCK_GRP_DECLARE(usbdisk_lck_grp, "hakios-usbdisk");
static LCK_MTX_DECLARE(usbdisk_lock, &usbdisk_lck_grp);         /* all traffic on the bus: one command at a time */

static inline uint32_t rd(vm_offset_t base, uint32_t off) { return *(volatile uint32_t *)(base + off); }
static inline void wr(vm_offset_t base, uint32_t off, uint32_t v) { *(volatile uint32_t *)(base + off) = v; }
static inline void set(vm_offset_t base, uint32_t off, uint32_t bits) { wr(base, off, rd(base, off) | bits); }
static inline void clr(vm_offset_t base, uint32_t off, uint32_t bits) { wr(base, off, rd(base, off) & ~bits); }
static inline volatile uint32_t *mem32(uint32_t off) { return (volatile uint32_t *)(u.blk + off); }
static inline volatile uint8_t *mem8(uint32_t off) { return (volatile uint8_t *)(u.blk + off); }
static inline uint32_t pa32(uint32_t off) { return (uint32_t)(u.pa + off); }

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

	set(u.clk, CLK_SYS_EN0_REG1, 1u << 26);
	clr(u.pad, MUX_H8, 0xfu);
	set(u.pad, GPIOH_OUT, 1u << 8);
	clr(u.pad, GPIOH_DIR, 1u << 8);
	/* the PHY out of reset, then as a host: POR, host, ID pull-up, drive VBUS */
	set(u.rst, RST_LEVEL0, RST_PHY);
	set(u.regs, PHY_R0, 0x39);
	IODelay(10);
	clr(u.rst, RST_LEVEL0, RST_PHY);
	IODelay(100);
	set(u.rst, RST_LEVEL0, RST_PHY);
	IODelay(50);
	/* the trim of the chip, from its fuses */
	t = rd(u.trim, TRIM_REG);
	cali = (t & (1u << 12)) ? ((t >> 8) & 0xf) + 2 : 9;
	if (cali > 12) {
		cali = 12;
	}
	wr(u.regs, PHY_CFG + 0x10, (rd(u.regs, PHY_CFG + 0x10) & ~0xfffu) | ((1u << cali) - 1));
	clr(u.regs, PHY_CFG + 0x54, 1u << 2);
	for (i = 0; i < 200 && !(rd(u.regs, PHY_R1) & 1); i++) {
		IODelay(5);
	}
	printf("usbdisk: phy %s (trim %#x, cali %u)\n", (rd(u.regs, PHY_R1) & 1) ? "ready" : "not ready", t, cali);
	/* its PLL (pll-version 3) */
	set(u.regs, PHY_CFG + 0x40, 1u << 27);
	IODelay(10);
	for (i = 0; i < 5; i++) {
		wr(u.regs, PHY_CFG + 0x40, 0x549540 | 1u << 27);
		IODelay(10);
		set(u.regs, PHY_CFG + 0x40, 1u << 26);
		IODelay(20);
		set(u.regs, PHY_CFG + 0x40, 1u << 25);
		IODelay(20);
		set(u.regs, PHY_CFG + 0x40, 1u << 24);
		for (int j = 0; j < 5 && !(rd(u.regs, PHY_CFG + 0x40) & (1u << 31)); j++) {
			IODelay(20);
		}
		if (rd(u.regs, PHY_CFG + 0x40) & (1u << 31)) {
			break;
		}
	}
	v = rd(u.regs, PHY_CFG + 0x40);
	printf("usbdisk: phy pll %s (%#x, %d retries)\n", (v & (1u << 31)) ? "locked" : "NOT locked", v, i);
	wr(u.regs, PHY_CFG + 0x0c, 0x39);
	clr(u.regs, PHY_CFG + 0x38, 1u << 27);
	wr(u.regs, PHY_CFG + 0x50, 0x7e18);
	/* the controller out of reset, as a host */
	set(u.rst, RST_LEVEL0, RST_CTRL);
	IODelay(100);
	clr(u.regs, CRG_MODE, 1u << 0);
}

/* the first xHCI of bus 0, given a place in the bus's memory window and switched on */
static bool
pci_find(void)
{
	vm_offset_t ecam = ml_io_map(PCI_ECAM, 0x100000);

	for (uint32_t dev = 0; dev < 32; dev++) {
		vm_offset_t cfg = ecam + (dev << 15);

		if (rd(cfg, 0) == 0xffffffffu || (rd(cfg, 0x08) >> 8) != 0x0c0330) {
			continue;
		}
		wr(cfg, 0x10, (uint32_t)PCI_MMIO);
		wr(cfg, 0x14, 0);
		wr(cfg, 0x04, rd(cfg, 0x04) | 0x6);                 /* memory, bus master */
		u.regs = ml_io_map(PCI_MMIO, 0x10000);
		printf("usbdisk: xHCI %04x:%04x at PCI device %u\n", rd(cfg, 0) & 0xffff, rd(cfg, 0) >> 16, dev);
		return true;
	}
	printf("usbdisk: no xHCI on the PCI bus\n");
	return false;
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
	uint64_t deadline = 0, now;
	unsigned spun = 0;

	for (;;) {
		volatile uint32_t *e = mem32(OFF_EVENTS + u.ev_idx * 16);

		if ((e[3] & TRB_CYCLE) == u.ev_cycle) {
			for (int i = 0; i < 4; i++) {
				u.ev[i] = e[i];
			}
			if (++u.ev_idx == N_TRBS) {
				u.ev_idx = 0;
				u.ev_cycle ^= 1;
			}
			wr64(u.rt, 0x38, (uint64_t)pa32(OFF_EVENTS + u.ev_idx * 16) | (1u << 3));
			return true;
		}
		if (deadline == 0) {
			clock_interval_to_deadline(ms, NSEC_PER_MSEC, &deadline);
		} else {
			clock_get_uptime(&now);
			if (now > deadline) {
				return false;
			}
		}
		if (spun < SPIN_US) {
			IODelay(10);
			spun += 10;
		} else {
			IOSleep(1);
		}
	}
}

/* waits for an event of `type'; the others are let go */
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
		if (((u.ev[3] >> 10) & 0x3f) == type) {
			return true;
		}
	}
}

/* a command; its completion code, or 0 for no answer */
static uint32_t
command(uint32_t d0, uint32_t d1, uint32_t d2, uint32_t d3)
{
	ring_put(&u.cmd, d0, d1, d2, d3);
	wr(u.db, 0, 0);
	if (!wait_event(EV_COMMAND, 1000)) {
		printf("usbdisk: command %u: no answer\n", (d3 >> 10) & 0x3f);
		return 0;
	}
	return u.ev[2] >> 24;
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
		if (((u.ev[3] >> 16) & 0x1f) == dci && (u.ev[3] >> 24) == u.slot) {
			break;
		}
	}
	*residual = u.ev[2] & 0xffffff;
	return u.ev[2] >> 24;
}

/* an endpoint that stalled or never answered, emptied and ready for the next transfer */
static void
ep_recover(uint32_t dci, struct ring *r)
{
	uint32_t state = mem32(OFF_OUT_CTX + dci * u.ctx_size)[0] & 7;

	if (state == 2) {
		command(0, 0, 0, TRB_TYPE(TRB_RESET_EP) | dci << 16 | u.slot << 24);
	} else if (state == 1) {
		command(0, 0, 0, TRB_TYPE(TRB_STOP_EP) | dci << 16 | u.slot << 24);
	}
	command(pa32(r->off + r->idx * 16) | r->cycle, 0, 0, TRB_TYPE(TRB_SET_DEQ) | dci << 16 | u.slot << 24);
}

/* a request on endpoint 0, with up to 4 KiB of data in or out through the control buffer. Bytes asked, or -1 */
static int
control(uint8_t type, uint8_t request, uint16_t value, uint16_t index, uint16_t length)
{
	bool in = (type & 0x80) != 0;
	uint32_t trt = length == 0 ? 0 : in ? 3 : 2, cc, residual = 0;

	ring_put(&u.ep0, (uint32_t)type | (uint32_t)request << 8 | (uint32_t)value << 16, (uint32_t)index | (uint32_t)length << 16,
	    8, TRB_TYPE(TRB_SETUP) | TRB_IDT | trt << 16);
	if (length != 0) {
		ring_put(&u.ep0, pa32(OFF_CTRL_BUF), 0, length, TRB_TYPE(TRB_DATA) | (in ? 1u << 16 : 0) | TRB_ISP);
	}
	ring_put(&u.ep0, 0, 0, 0, TRB_TYPE(TRB_STATUS) | TRB_IOC | ((length == 0 || !in) ? 1u << 16 : 0));
	wr(u.db, u.slot * 4, DCI_EP0);
	do {
		cc = transfer_done(DCI_EP0, 2000, &residual);
	} while (cc == CC_SHORT);               /* a short data stage: the status stage's event follows */
	if (cc != CC_SUCCESS) {
		printf("usbdisk: control %02x %02x %04x %04x %u: completion %u\n", type, request, value, index, length, cc);
		ep_recover(DCI_EP0, &u.ep0);
		return -1;
	}
	return length;
}

/* one bulk transfer on `dci' to or from physical `pa'; bytes moved, or -1 with the completion code in *cc */
static int
bulk(uint32_t dci, struct ring *r, uint32_t pa, uint32_t length, unsigned ms, uint32_t *cc)
{
	uint32_t residual = 0;

	ring_put(r, pa, 0, length, TRB_TYPE(TRB_NORMAL) | TRB_IOC | TRB_ISP);
	wr(u.db, u.slot * 4, dci);
	*cc = transfer_done(dci, ms, &residual);
	if (*cc != CC_SUCCESS && *cc != CC_SHORT) {
		return -1;
	}
	return (int)(length - residual);
}

/* ---- the controller ---- */

static bool
controller_up(void)
{
	uint32_t caplen = rd(u.regs, 0) & 0xff, hcs1 = rd(u.regs, 0x04), hcs2 = rd(u.regs, 0x08), hcc1 = rd(u.regs, 0x10);
	uint32_t scratch = ((hcs2 >> 21) & 0x1f) << 5 | ((hcs2 >> 27) & 0x1f);

	u.op = u.regs + caplen;
	u.rt = u.regs + (rd(u.regs, 0x18) & ~0x1fu);
	u.db = u.regs + (rd(u.regs, 0x14) & ~0x3u);
	u.ctx_size = (hcc1 & (1u << 2)) ? 64 : 32;
	u.ports = hcs1 >> 24;
	printf("usbdisk: xHCI %#x, %u ports, %u scratchpad pages, %u-byte contexts\n", rd(u.regs, 0) >> 16, u.ports,
	    scratch, u.ctx_size);
	if (scratch > MAX_SCRATCH) {
		printf("usbdisk: too many scratchpad pages\n");
		return false;
	}

	clr(u.op, 0x00, CMD_RUN);
	for (int i = 0; i < 100 && !(rd(u.op, 0x04) & STS_HCH); i++) {
		IOSleep(1);
	}
	set(u.op, 0x00, CMD_HCRST);
	for (int i = 0; i < 1000 && (rd(u.op, 0x00) & CMD_HCRST); i++) {
		IOSleep(1);
	}
	for (int i = 0; i < 1000 && (rd(u.op, 0x04) & STS_CNR); i++) {
		IOSleep(1);
	}
	if (rd(u.op, 0x04) & STS_CNR) {
		printf("usbdisk: the controller is not ready after its reset (status %#x)\n", rd(u.op, 0x04));
		return false;
	}
	if (u.mode == 1) {
		wr(u.regs, CRG_NAK_RETRY, (rd(u.regs, CRG_NAK_RETRY) & ~(0xffu << 13)) | 5u << 13);
	}

	for (uint32_t i = 0; i < BLK_SIZE / 4; i++) {
		mem32(0)[i] = 0;
	}
	for (uint32_t i = 0; i < scratch; i++) {
		mem32(OFF_SCRATCH_ARR)[i * 2] = pa32(OFF_SCRATCH + i * 0x1000);
	}
	mem32(OFF_DCBAA)[0] = scratch ? pa32(OFF_SCRATCH_ARR) : 0;
	wr(u.op, 0x38, 1);                                      /* one slot */
	wr64(u.op, 0x30, pa32(OFF_DCBAA));
	ring_init(&u.cmd, OFF_CMD);
	wr64(u.op, 0x18, (uint64_t)pa32(OFF_CMD) | 1);
	/* interrupter 0: its event ring, one segment */
	mem32(OFF_ERST)[0] = pa32(OFF_EVENTS);
	mem32(OFF_ERST)[1] = 0;
	mem32(OFF_ERST)[2] = N_TRBS;
	u.ev_idx = 0;
	u.ev_cycle = 1;
	__builtin_arm_dsb(0xf);
	wr(u.rt, 0x28, 1);
	wr64(u.rt, 0x38, pa32(OFF_EVENTS));
	wr64(u.rt, 0x30, pa32(OFF_ERST));
	wr(u.rt, 0x20, 0);                                      /* no interrupts: polled */
	set(u.op, 0x00, CMD_RUN);
	for (int i = 0; i < 100 && (rd(u.op, 0x04) & STS_HCH); i++) {
		IOSleep(1);
	}
	if (command(0, 0, 0, TRB_TYPE(TRB_NOOP_CMD)) != CC_SUCCESS) {
		printf("usbdisk: the command ring does not answer (status %#x)\n", rd(u.op, 0x04));
		return false;
	}
	return true;
}

static vm_offset_t
portsc(uint32_t port)
{
	return u.op + 0x400 + 0x10 * (port - 1);
}

/* the port with something on it, reset and enabled; false if nothing comes within `wait' seconds */
static bool
port_up(uint32_t wait)
{
	uint32_t v;

	for (uint32_t p = 1; p <= u.ports; p++) {
		v = rd(portsc(p), 0);
		if (!(v & PORTSC_PP)) {
			wr(portsc(p), 0, (v & PORTSC_KEEP) | PORTSC_PP);
		}
	}
	for (uint32_t tries = 0; tries < wait * 50 && u.port == 0; tries++) {
		IOSleep(20);
		for (uint32_t p = 1; p <= u.ports; p++) {
			if (rd(portsc(p), 0) & PORTSC_CCS) {
				u.port = p;
				break;
			}
		}
	}
	if (u.port == 0) {
		printf("usbdisk: nothing on the port after %u s (portsc %#x)\n", wait, rd(portsc(1), 0));
		return false;
	}
	IOSleep(100);                                           /* the device's supply settles */
	v = rd(portsc(u.port), 0);
	wr(portsc(u.port), 0, (v & PORTSC_KEEP) | PORTSC_PR);
	for (int i = 0; i < 150 && !(rd(portsc(u.port), 0) & PORTSC_PRC); i++) {
		IOSleep(2);
	}
	v = rd(portsc(u.port), 0);
	wr(portsc(u.port), 0, (v & PORTSC_KEEP) | (v & PORTSC_CHANGES));
	IOSleep(20);
	v = rd(portsc(u.port), 0);
	u.speed = PORTSC_SPEED(v);
	printf("usbdisk: port %u after its reset: %#x (%s, speed %u)\n", u.port, v, (v & PORTSC_PED) ? "enabled" : "NOT enabled",
	    u.speed);
	if (u.speed != 3 && u.speed != 4) {
		printf("usbdisk: not a high speed device: not handled\n");
		return false;
	}
	return (v & PORTSC_PED) != 0;
}

static volatile uint32_t *
in_ctx(uint32_t index)          /* 0 control, 1 slot, 1 + dci endpoints */
{
	return mem32(OFF_IN_CTX + index * u.ctx_size);
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

/* address, descriptors, configuration; the mass storage interface's two bulk endpoints */
static bool
enumerate(void)
{
	uint32_t cc, last;
	volatile uint8_t *buf = mem8(OFF_CTRL_BUF);
	uint16_t total, vendor, product;
	uint8_t config;
	bool ours = false;

	cc = command(0, 0, 0, TRB_TYPE(TRB_ENABLE_SLOT));
	if (cc != CC_SUCCESS) {
		printf("usbdisk: enable slot: completion %u\n", cc);
		return false;
	}
	u.slot = u.ev[3] >> 24;
	mem32(OFF_DCBAA)[u.slot * 2] = pa32(OFF_OUT_CTX);
	in_ctx(0)[1] = 1u << 0 | 1u << 1;                       /* add: the slot, endpoint 0 */
	in_ctx(1)[0] = u.speed << 20 | 1u << 27;
	in_ctx(1)[1] = u.port << 16;
	ring_init(&u.ep0, OFF_EP0);
	ep_ctx(DCI_EP0, 4, u.speed == 4 ? 512 : 64, &u.ep0, 8);
	__builtin_arm_dsb(0xf);
	cc = command(pa32(OFF_IN_CTX), 0, 0, TRB_TYPE(TRB_ADDRESS) | u.slot << 24);
	if (cc != CC_SUCCESS) {
		printf("usbdisk: address device: completion %u\n", cc);
		return false;
	}
	IOSleep(10);
	if (control(0x80, 6, 0x0100, 0, 18) < 0) {
		return false;
	}
	vendor = buf[8] | buf[9] << 8;
	product = buf[10] | buf[11] << 8;
	if (control(0x80, 6, 0x0200, 0, 9) < 0) {
		return false;
	}
	total = buf[2] | buf[3] << 8;
	if (total > 0x1000 || control(0x80, 6, 0x0200, 0, total) < 0) {
		return false;
	}
	config = buf[5];
	for (uint32_t i = 0; i + 2 <= total && buf[i] >= 2; i += buf[i]) {
		if (buf[i + 1] == 4) {                          /* an interface: mass storage, SCSI, bulk only? */
			if (u.ep_in != 0 && u.ep_out != 0) {
				break;
			}
			ours = buf[i + 5] == 8 && buf[i + 6] == 6 && buf[i + 7] == 0x50;
			u.iface = buf[i + 2];
			u.ep_in = u.ep_out = 0;
		} else if (buf[i + 1] == 5 && ours && (buf[i + 3] & 3) == 2) {
			if (buf[i + 2] & 0x80) {
				u.ep_in = buf[i + 2];
				u.mps_in = buf[i + 4] | buf[i + 5] << 8;
			} else {
				u.ep_out = buf[i + 2];
				u.mps_out = buf[i + 4] | buf[i + 5] << 8;
			}
		}
	}
	if (u.ep_in == 0 || u.ep_out == 0) {
		printf("usbdisk: device %04x:%04x is not a mass storage disk\n", vendor, product);
		return false;
	}
	if (control(0x00, 9, config, 0, 0) < 0) {               /* SET_CONFIGURATION */
		return false;
	}
	u.dci_out = (u.ep_out & 0xf) * 2;
	u.dci_in = (u.ep_in & 0xf) * 2 + 1;
	last = u.dci_out > u.dci_in ? u.dci_out : u.dci_in;
	for (uint32_t i = 0; i < 0x1000 / 4; i++) {
		mem32(OFF_IN_CTX)[i] = 0;
	}
	in_ctx(0)[1] = 1u << 0 | 1u << u.dci_out | 1u << u.dci_in;
	in_ctx(1)[0] = u.speed << 20 | last << 27;
	in_ctx(1)[1] = u.port << 16;
	ring_init(&u.out, OFF_EP_OUT);
	ring_init(&u.in, OFF_EP_IN);
	ep_ctx(u.dci_out, 2, u.mps_out, &u.out, 1024);
	ep_ctx(u.dci_in, 6, u.mps_in, &u.in, 1024);
	__builtin_arm_dsb(0xf);
	cc = command(pa32(OFF_IN_CTX), 0, 0, TRB_TYPE(TRB_CONFIGURE) | u.slot << 24);
	if (cc != CC_SUCCESS) {
		printf("usbdisk: configure endpoint: completion %u\n", cc);
		return false;
	}
	printf("usbdisk: device %04x:%04x, mass storage on endpoints %#04x and %#04x (%u bytes)\n", vendor, product, u.ep_out,
	    u.ep_in, u.mps_in);
	return true;
}

/* ---- bulk-only transport, SCSI ---- */

static void
clear_halt(uint32_t dci, struct ring *r, uint8_t ep)
{
	ep_recover(dci, r);
	control(0x02, 1, 0, ep, 0);                             /* CLEAR_FEATURE(ENDPOINT_HALT) */
}

/* the transport back to where a command may be sent; the disk given up if it is not there any more */
static void
reset_recovery(void)
{
	if (!(rd(portsc(u.port), 0) & PORTSC_CCS) || control(0x21, 0xff, 0, u.iface, 0) < 0) {
		if (!u.gone) {
			printf("usbdisk: the disk is gone (portsc %#x)\n", rd(portsc(u.port), 0));
		}
		u.gone = true;
		return;
	}
	clear_halt(u.dci_in, &u.in, u.ep_in);
	clear_halt(u.dci_out, &u.out, u.ep_out);
}

/*
 * One command, with `length' bytes of data in or out at physical `pa'. 0: done; 1: the disk says it failed (its
 * sense data says why); -1: the transport failed, and was reset.
 */
static int
bot(const uint8_t *cdb, uint8_t cdb_length, bool in, uint32_t pa, uint32_t length, unsigned ms)
{
	volatile uint8_t *cbw = mem8(OFF_CBW), *csw = mem8(OFF_CSW);
	uint32_t f[3] = { CBW_SIG, ++u.tag, length }, cc = 0, residue = 0, sig = 0, tag = 0;
	int n;

	for (int i = 0; i < 31; i++) {
		cbw[i] = 0;
	}
	for (int i = 0; i < 12; i++) {
		cbw[i] = (uint8_t)(f[i / 4] >> (8 * (i % 4)));
	}
	cbw[12] = in ? 0x80 : 0;
	cbw[14] = cdb_length;
	for (int i = 0; i < cdb_length; i++) {
		cbw[15 + i] = cdb[i];
	}
	__builtin_arm_dsb(0xf);
	if (bulk(u.dci_out, &u.out, pa32(OFF_CBW), 31, 5000, &cc) != 31) {
		printf("usbdisk: command %#04x not taken (completion %u)\n", cdb[0], cc);
		reset_recovery();
		return -1;
	}
	if (length != 0) {
		n = in ? bulk(u.dci_in, &u.in, pa, length, ms, &cc) : bulk(u.dci_out, &u.out, pa, length, ms, &cc);
		if (n < 0 && cc == CC_STALL) {
			if (in) {
				clear_halt(u.dci_in, &u.in, u.ep_in);
			} else {
				clear_halt(u.dci_out, &u.out, u.ep_out);
			}
		} else if (n < 0) {
			printf("usbdisk: command %#04x: data, completion %u\n", cdb[0], cc);
			reset_recovery();
			return -1;
		}
	}
	n = bulk(u.dci_in, &u.in, pa32(OFF_CSW), 13, ms, &cc);
	if (n < 0 && cc == CC_STALL) {
		clear_halt(u.dci_in, &u.in, u.ep_in);
		n = bulk(u.dci_in, &u.in, pa32(OFF_CSW), 13, ms, &cc);
	}
	for (int i = 0; i < 4; i++) {
		sig |= (uint32_t)csw[i] << (8 * i);
		tag |= (uint32_t)csw[4 + i] << (8 * i);
		residue |= (uint32_t)csw[8 + i] << (8 * i);
	}
	if (n != 13 || sig != CSW_SIG || tag != u.tag || csw[12] > 1) {
		printf("usbdisk: command %#04x: status of %d bytes, completion %u, signature %#x, tag %#x/%#x, %u\n", cdb[0], n,
		    cc, sig, tag, u.tag, csw[12]);
		reset_recovery();
		return -1;
	}
	if (csw[12] == 0 && residue != 0 && (cdb[0] == 0x28 || cdb[0] == 0x2a)) {
		return -1;                                      /* blocks not moved, without a word */
	}
	return csw[12];
}

/* a command, asked again when the disk has something to say first (it was reset, it is spinning up). 0 or EIO */
static int
scsi(const uint8_t *cdb, uint8_t cdb_length, bool in, uint32_t pa, uint32_t length, unsigned ms)
{
	static const uint8_t sense[6] = { 0x03, 0, 0, 0, 18, 0 };
	volatile uint8_t *s = mem8(OFF_SMALL + 0x200);
	int r = -1;

	for (int try = 0; try < 4 && !u.gone; try++) {
		r = bot(cdb, cdb_length, in, pa, length, ms);
		if (r == 0) {
			return 0;
		}
		if (r == 1 && bot(sense, 6, true, pa32(OFF_SMALL + 0x200), 18, 5000) == 0) {
			uint8_t key = s[2] & 0xf;

			if (cdb[0] != 0x00 && cdb[0] != 0x35) {
				printf("usbdisk: command %#04x: sense %x/%02x/%02x\n", cdb[0], key, s[12], s[13]);
			}
			if (key == 5 || key == 7) {
				break;                          /* not a command of this disk, or it is write protected */
			}
			IOSleep(key == 2 ? 200 : 20);
		}
	}
	return EIO;
}

/* sectors [lba, lba + count) of the disk, through the bounce buffer. Lock held */
static int
usbdisk_xfer(bool write, uint64_t lba, uint32_t count, void *data)
{
	uint8_t cdb[10] = { write ? 0x2a : 0x28, 0, (uint8_t)(lba >> 24), (uint8_t)(lba >> 16), (uint8_t)(lba >> 8), (uint8_t)lba,
		            0, (uint8_t)(count >> 8), (uint8_t)count, 0 };
	size_t size = (size_t)count * SECTOR;
	int error;

	if (count == 0 || count > MAX_BLOCKS || lba + count > u.sectors) {
		return EINVAL;
	}
	if (u.gone) {
		return EIO;
	}
	if (write) {
		memcpy((void *)u.bounce, data, size);
	}
	FlushPoC_DcacheRegion(u.bounce, size);
	error = scsi(cdb, 10, !write, (uint32_t)u.bounce_pa, (uint32_t)size, write ? 30000 : 10000);
	FlushPoC_DcacheRegion(u.bounce, size);
	if (error == 0 && !write) {
		memcpy(data, (const void *)u.bounce, size);
	}
	if (error != 0) {
		u.errors++;
	} else if (write) {
		u.writes++;
	} else {
		u.reads++;
	}
	return error;
}

/* what the disk is, that it is ready, how large it is */
static bool
disk_up(void)
{
	static const uint8_t inquiry[6] = { 0x12, 0, 0, 0, 36, 0 }, ready[6] = { 0x00 }, capacity[10] = { 0x25 };
	volatile uint8_t *b = mem8(OFF_SMALL);
	char name[25];
	uint32_t last, size;
	int i;

	if (scsi(inquiry, 6, true, pa32(OFF_SMALL), 36, 5000) != 0) {
		return false;
	}
	for (i = 0; i < 24; i++) {
		name[i] = (b[8 + i] >= 0x20 && b[8 + i] < 0x7f) ? (char)b[8 + i] : ' ';
	}
	name[24] = 0;
	for (i = 0; i < 50 && scsi(ready, 6, false, 0, 0, 5000) != 0 && !u.gone; i++) {
		IOSleep(100);
	}
	if (scsi(capacity, 10, true, pa32(OFF_SMALL), 8, 5000) != 0) {
		printf("usbdisk: \"%s\" does not say how large it is\n", name);
		return false;
	}
	last = (uint32_t)b[0] << 24 | b[1] << 16 | b[2] << 8 | b[3];
	size = (uint32_t)b[4] << 24 | b[5] << 16 | b[6] << 8 | b[7];
	printf("usbdisk: \"%s\", %llu MiB in blocks of %u bytes\n", name, ((uint64_t)last + 1) * size >> 20, size);
	if (size != SECTOR || last == 0xffffffffu) {
		printf("usbdisk: only disks under 2 TiB with 512-byte blocks are handled\n");
		return false;
	}
	u.sectors = (uint64_t)last + 1;
	return true;
}

static bool
read_mbr(void)
{
	static uint8_t mbr[SECTOR];
	int found = 0;

	if (usbdisk_xfer(false, 0, 1, mbr) != 0) {
		return false;
	}
	if (mbr[510] != 0x55 || mbr[511] != 0xaa) {
		printf("usbdisk: no partition table\n");
		return false;
	}
	for (int i = 0; i < 3; i++) {
		const uint8_t *e = &mbr[446 + 16 * i];
		uint64_t first = (uint32_t)(e[8] | e[9] << 8 | e[10] << 16 | (uint32_t)e[11] << 24);
		uint64_t count = (uint32_t)(e[12] | e[13] << 8 | e[14] << 16 | (uint32_t)e[15] << 24);

		if (e[4] == 0 || count == 0 || first + count > u.sectors) {
			continue;
		}
		u.part[i].first = first;
		u.part[i].count = count;
		found++;
		printf("usbdisk: partition %d (type %#04x) %llu MiB at sector %llu: /dev/usb_%s\n", i + 1, e[4], count >> 11, first,
		    u.part[i].name);
	}
	return found != 0;
}

/* ---- the block devices ---- */

static struct usb_part *
usbdisk_part_of(dev_t dev)
{
	int unit = minor(dev);

	return (u.ready && unit >= 0 && unit < 3 && u.part[unit].count != 0) ? &u.part[unit] : NULL;
}

bool
hakios_usbdisk_dev(dev_t dev)
{
	return (u.bmajor >= 0 && major(dev) == u.bmajor) || (u.cmajor >= 0 && major(dev) == u.cmajor);
}

static int
usbdisk_open(dev_t dev, int flags, __unused int devtype, __unused proc_t p)
{
	struct usb_part *pt = usbdisk_part_of(dev);

	if (pt == NULL) {
		return ENXIO;
	}
	if ((flags & FWRITE) && !pt->writable) {
		return EROFS;
	}
	return 0;
}

static int
usbdisk_close(__unused dev_t dev, __unused int flags, __unused int devtype, __unused proc_t p)
{
	return 0;
}

static void
usbdisk_strategy(buf_t bp)
{
	struct usb_part *pt = usbdisk_part_of(buf_device(bp));
	bool write = !(buf_flags(bp) & B_READ);
	uint64_t first, len, done = 0;
	caddr_t mapped;
	int error = 0;

	if (pt == NULL || (write && !pt->writable)) {
		buf_seterror(bp, pt == NULL ? ENXIO : EROFS);
		buf_biodone(bp);
		return;
	}
	first = (uint64_t)buf_blkno(bp);
	len = buf_count(bp);
	if ((len % SECTOR) != 0 || first > pt->count || len / SECTOR > pt->count - first) {
		buf_seterror(bp, first == pt->count ? 0 : EINVAL);
		buf_setresid(bp, buf_count(bp));
		buf_biodone(bp);
		return;
	}
	if ((error = buf_map(bp, &mapped)) != 0) {
		buf_seterror(bp, error);
		buf_biodone(bp);
		return;
	}
	lck_mtx_lock(&usbdisk_lock);
	while (done < len && error == 0) {
		uint32_t n = (uint32_t)MIN((len - done) / SECTOR, MAX_BLOCKS);

		error = usbdisk_xfer(write, pt->first + first + done / SECTOR, n, (uint8_t *)mapped + done);
		if (error == 0) {
			done += (uint64_t)n * SECTOR;
		}
	}
	lck_mtx_unlock(&usbdisk_lock);
	buf_unmap(bp);
	buf_setresid(bp, (uint32_t)(len - done));
	if (error) {
		buf_seterror(bp, error);
	}
	buf_biodone(bp);
}

static int
usbdisk_ioctl(dev_t dev, u_long cmd, caddr_t data, __unused int fflag, __unused proc_t p)
{
	static const uint8_t sync[10] = { 0x35 };
	struct usb_part *pt = usbdisk_part_of(dev);

	if (pt == NULL) {
		return ENXIO;
	}
	switch (cmd) {
	case DKIOCGETBLOCKSIZE:
	case DKIOCGETPHYSICALBLOCKSIZE:
		*(uint32_t *)data = SECTOR;
		return 0;
	case DKIOCSETBLOCKSIZE:
		return *(uint32_t *)data == SECTOR ? 0 : EINVAL;
	case DKIOCGETBLOCKCOUNT:
		*(uint64_t *)data = pt->count;
		return 0;
	case DKIOCISWRITABLE:
		*(uint32_t *)data = pt->writable;
		return 0;
	case DKIOCGETMAXBLOCKCOUNTREAD:
	case DKIOCGETMAXBLOCKCOUNTWRITE:
		*(uint64_t *)data = MAX_BLOCKS;
		return 0;
	case DKIOCSYNCHRONIZE:
	case DKIOCSYNCHRONIZECACHE:
		if (pt->writable) {                             /* a disk that has no cache says so: no error */
			lck_mtx_lock(&usbdisk_lock);
			(void)scsi(sync, 10, false, 0, 0, 30000);
			lck_mtx_unlock(&usbdisk_lock);
		}
		return 0;
	default:
		return ENOTTY;
	}
}

static int
usbdisk_size(dev_t dev)
{
	return usbdisk_part_of(dev) ? (int)SECTOR : -1;
}

static int
usbdisk_read(dev_t dev, struct uio *uio, __unused int ioflag)
{
	return physio(usbdisk_strategy, NULL, dev, B_READ, (u_int (*)(buf_t))usbdisk_size, uio, SECTOR);
}

static int
usbdisk_write(dev_t dev, struct uio *uio, __unused int ioflag)
{
	return physio(usbdisk_strategy, NULL, dev, B_WRITE, (u_int (*)(buf_t))usbdisk_size, uio, SECTOR);
}

static const struct bdevsw usbdisk_bdevsw = {
	.d_open     = usbdisk_open,
	.d_close    = usbdisk_close,
	.d_strategy = usbdisk_strategy,
	.d_ioctl    = usbdisk_ioctl,
	.d_dump     = eno_dump,
	.d_psize    = usbdisk_size,
	.d_type     = D_DISK,
};

static const struct cdevsw usbdisk_cdevsw = {
	.d_open       = usbdisk_open,
	.d_close      = usbdisk_close,
	.d_read       = usbdisk_read,
	.d_write      = usbdisk_write,
	.d_ioctl      = usbdisk_ioctl,
	.d_stop       = eno_stop,
	.d_reset      = eno_reset,
	.d_ttys       = NULL,
	.d_select     = eno_select,
	.d_mmap       = eno_mmap,
	.d_strategy   = eno_strat,
	.d_reserved_1 = eno_getc,
	.d_reserved_2 = eno_putc,
	.d_type       = D_DISK,
};

void
hakios_usbdisk_init(void)
{
	uint32_t wait = 10;
	vm_offset_t va;
	bool up;

	if (!PE_parse_boot_argn("usbdisk", &u.mode, sizeof(u.mode)) || u.mode == 0) {
		return;
	}
	PE_parse_boot_argn("usbdisk_wait", &wait, sizeof(wait));
	if (kmem_alloc_contig(kernel_map, &va, BLK_SIZE, PAGE_MASK, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS ||
	    kmem_alloc_contig(kernel_map, &u.bounce, MAX_XFER, 0xffff, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("usbdisk: no memory\n");
		return;
	}
	FlushPoC_DcacheRegion(va, BLK_SIZE);
	u.pa = kvtophys(va);
	u.bounce_pa = kvtophys(u.bounce);
	if ((u.pa >> 32) || (u.bounce_pa >> 32)) {
		printf("usbdisk: memory above 4 GB: the controller cannot reach it\n");
		return;
	}
	u.blk = ml_io_map(u.pa, BLK_SIZE);
	if (u.mode == 1) {
		u.regs = ml_io_map(CRG_BASE, CRG_SIZE);
		u.clk = ml_io_map(CLK_BASE, 0x1000);
		u.rst = ml_io_map(RST_BASE, 0x1000);
		u.pad = ml_io_map(PAD_BASE, 0x1000);
		u.trim = ml_io_map(TRIM_BASE, 0x1000);
		phy_init();
	} else if (!pci_find()) {
		return;
	}
	lck_mtx_lock(&usbdisk_lock);
	up = controller_up() && port_up(wait) && enumerate() && disk_up() && read_mbr();
	lck_mtx_unlock(&usbdisk_lock);
	if (!up) {
		return;
	}
	u.bmajor = bdevsw_add(-1, &usbdisk_bdevsw);
	if (u.bmajor < 0) {
		printf("usbdisk: bdevsw_add failed\n");
		return;
	}
	u.cmajor = cdevsw_add_with_bdev(-1, &usbdisk_cdevsw, u.bmajor);
	if (u.cmajor < 0) {
		printf("usbdisk: cdevsw_add failed\n");
		return;
	}
	u.ready = true;
	for (int i = 0; i < 3; i++) {
		if (u.part[i].count == 0) {
			continue;
		}
		devfs_make_node(makedev(u.bmajor, i), DEVFS_BLOCK, UID_ROOT, GID_OPERATOR, 0640, "usb_%s", u.part[i].name);
		devfs_make_node(makedev(u.cmajor, i), DEVFS_CHAR, UID_ROOT, GID_OPERATOR, 0640, "rusb_%s", u.part[i].name);
	}
}

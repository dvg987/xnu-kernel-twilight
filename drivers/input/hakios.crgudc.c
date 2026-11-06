/*
 * The Amlogic box's USB port as a device for the host it is plugged into (the Corigine "CRG" device controller
 * at 0xfe350000): one vendor interface with one bulk OUT endpoint, over which the host sends keys.
 * Boot-arg usbkeys=1. A record is 8 bytes, { type, code } as in sys/hakios.input.h: type 4 (down) or 5 (up),
 * code (HID usage page << 16) | usage; each goes to /dev/hakios-touch, where backboardd's twilight keys take
 * it (tvos/src/twlinput.inc). The host side is tools/twl-keys.py. Type 9 with code 'STOP' (0x504f5453) ends the
 * run: control goes back to the loader, as when loader_return_after expires (and from there to fastboot).
 *
 * Polled from a kernel thread, no interrupt. The controller is used as U-Boot's fastboot leaves it (clock and
 * PHY up); it is stopped, reset and set up again here. Rings, contexts and buffers are one physically
 * contiguous block below 4 GB that is only touched through an uncached mapping (the controller is not
 * coherent). The programming model is Amlogic's Linux crg_udc.c (see tvos/docs/crg-udc.md).
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/hakios.input.h>
#include <sys/hakios.crgudc.h>
#include <vm/vm_kern.h>
#include <arm/caches_internal.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern uint64_t kvtophys(vm_offset_t va);
extern void IOSleep(unsigned int milliseconds);
extern void IODelay(unsigned int microseconds);
extern kern_return_t kernel_thread_start_priority(thread_continue_t continuation, void *parameter, integer_t priority, thread_t *new_thread);
extern void loader_return_now(void);

#define RECORD_STOP     9
#define STOP_CODE       0x504f5453      /* "STOP" */

#define CRG_BASE        0xfe350000UL
#define R_ROLE          0x20fc
#define R_VBUS          0x2110
#define R_CONFIG0       0x2410
#define R_CONFIG1       0x2414
#define R_CONTROL       0x2420
#define R_STATUS        0x2424
#define R_DCBAPLO       0x2428
#define R_DCBAPHI       0x242c
#define R_PORTSC        0x2430
#define R_U3PORTPM      0x2434
#define R_DOORBELL      0x2440
#define R_EP_ENABLE     0x2460
#define R_CMD_PARAM0    0x2470
#define R_CMD_PARAM1    0x2474
#define R_CMD_CONTROL   0x2478
#define R_IMAN          0x2500
#define R_IMOD          0x2504
#define R_ERSTSZ        0x2508
#define R_ERSTBALO      0x2510
#define R_ERSTBAHI      0x2514
#define R_ERDPLO        0x2518
#define R_ERDPHI        0x251c

#define CTRL_RUN        (1u << 0)
#define CTRL_SWRST      (1u << 1)
#define CTRL_INT_EN     (1u << 2)
#define CTRL_SYSERR_EN  (1u << 3)

#define PORTSC_CCS      (1u << 0)
#define PORTSC_PED      (1u << 1)
#define PORTSC_PP       (1u << 3)
#define PORTSC_PR       (1u << 4)
#define PORTSC_PLS(v)   (((v) >> 5) & 0xf)
#define PORTSC_SPEED(v) (((v) >> 10) & 0xf)
#define PORTSC_CSC      (1u << 17)
#define PORTSC_PRC      (1u << 21)
#define PORTSC_PLC      (1u << 22)
#define PORTSC_W1C      ((1u << 17) | (1u << 18) | (1u << 20) | (1u << 21) | (1u << 22) | (1u << 23))

#define CMD_INIT_EP0    0
#define CMD_UPDATE_EP0  1
#define CMD_SET_ADDR    2
#define CMD_CONFIG_EP   4

#define TRB_NORMAL      1
#define TRB_DATA        3
#define TRB_STATUS      4
#define TRB_LINK        6
#define TRB_CYCLE       (1u << 0)
#define TRB_TC          (1u << 1)
#define TRB_ISP         (1u << 2)
#define TRB_IOC         (1u << 5)
#define TRB_DIR         (1u << 16)
#define TRB_TAG(t)      ((uint32_t)(t) << 17)
#define TRB_STALL       (1u << 19)
#define TRB_SET_ADDR    (1u << 20)

#define EVT_TRANSFER    32
#define EVT_PORT        34
#define EVT_SETUP       40

/* the block: offsets in it */
#define BLK_SIZE        0x10000
#define OFF_EVENTS      0x0000          /* 256 event TRBs */
#define OFF_ERST        0x1000
#define OFF_DCBA        0x1100          /* 30 endpoint contexts */
#define OFF_EP0_RING    0x1400          /* 16 TRBs */
#define OFF_OUT_RING    0x1600          /* 16 TRBs */
#define OFF_EP0_BUF     0x2000          /* 512 bytes */
#define OFF_OUT_BUF     0x3000          /* 4096 bytes */
#define N_EVENTS        256
#define N_RING          16
#define DCI_OUT         3               /* EP1 OUT */

struct ring {
	uint32_t off, enq, pcs;
};

static struct {
	vm_offset_t regs;
	vm_offset_t blk;                /* the uncached mapping */
	uint64_t    pa;
	uint32_t    ev_idx, ev_ccs;
	struct ring ep0, out;
	uint32_t    setup_tag;
	int         ep0_stage;          /* 0 idle, 1 data in flight then status, 2 status in flight */
	uint32_t    ep0_status_dir;
	int         addressed, configured;
	uint64_t    keys, events;
} u;

static inline uint32_t rd(uint32_t r) { return *(volatile uint32_t *)(u.regs + r); }
static inline void wr(uint32_t r, uint32_t v) { *(volatile uint32_t *)(u.regs + r) = v; }
static inline volatile uint32_t *mem32(uint32_t off) { return (volatile uint32_t *)(u.blk + off); }
static inline uint32_t pa32(uint32_t off) { return (uint32_t)(u.pa + off); }

static int
command(uint32_t type, uint32_t p0, uint32_t p1)
{
	wr(R_CMD_PARAM0, p0);
	wr(R_CMD_PARAM1, p1);
	wr(R_CMD_CONTROL, 1u | type << 4);
	for (int i = 0; i < 4000; i++) {
		uint32_t c = rd(R_CMD_CONTROL);
		if (!(c & 1)) {
			return (c >> 16) & 0xf;
		}
		IODelay(5);
	}
	printf("usbkeys: command %u timed out\n", type);
	return -1;
}

static void
ring_init(struct ring *r, uint32_t off)
{
	r->off = off;
	r->enq = 0;
	r->pcs = 1;
	for (uint32_t i = 0; i < N_RING * 4; i++) {
		mem32(off)[i] = 0;
	}
	volatile uint32_t *link = mem32(off + (N_RING - 1) * 16);
	link[0] = pa32(off);
	link[1] = 0;
	link[2] = 0;
	link[3] = TRB_LINK << 10 | TRB_TC;
}

static void
ring_put(struct ring *r, uint32_t dw0, uint32_t dw2, uint32_t dw3)
{
	volatile uint32_t *t = mem32(r->off + r->enq * 16);

	t[0] = dw0;
	t[1] = 0;
	t[2] = dw2;
	__builtin_arm_dmb(0xf);
	t[3] = dw3 | r->pcs;
	__builtin_arm_dsb(0xf);
	if (++r->enq == N_RING - 1) {
		volatile uint32_t *link = mem32(r->off + (N_RING - 1) * 16);
		link[3] = TRB_LINK << 10 | TRB_TC | r->pcs;
		r->pcs ^= 1;
		r->enq = 0;
	}
}

/* ---- descriptors ---- */

static const uint8_t dev_desc[18] = {
	18, 1, 0x00, 0x02, 0xff, 0x00, 0x00, 64, 0xd1, 0x18, 0xe0, 0x4e, 0x00, 0x01, 1, 2, 0, 1,
};
static const uint8_t cfg_desc[25] = {
	9, 2, 25, 0, 1, 1, 0, 0x80, 50,
	9, 4, 0, 0, 1, 0xff, 0x00, 0x00, 0,
	7, 5, 0x01, 0x02, 0x00, 0x02, 0,
};

static uint32_t
string_desc(int index, volatile uint8_t *out)
{
	static const char *strings[] = { NULL, "twilight", "tvOS keys" };
	uint32_t n = 2;

	if (index == 0) {
		out[0] = 4; out[1] = 3; out[2] = 0x09; out[3] = 0x04;
		return 4;
	}
	if (index >= 3) {
		return 0;
	}
	for (const char *c = strings[index]; *c; c++) {
		out[n++] = (uint8_t)*c;
		out[n++] = 0;
	}
	out[0] = (uint8_t)n;
	out[1] = 3;
	return n;
}

/* ---- control transfers ---- */

static void
ep0_status(uint32_t dir, uint32_t extra)
{
	ring_put(&u.ep0, 0, 0, TRB_STATUS << 10 | TRB_IOC | dir | TRB_TAG(u.setup_tag) | extra);
	wr(R_DOORBELL, 0);
	u.ep0_stage = 2;
}

static void
ep0_data_in(uint32_t len)
{
	ring_put(&u.ep0, pa32(OFF_EP0_BUF), len, TRB_DATA << 10 | TRB_IOC | TRB_ISP | TRB_TAG(u.setup_tag));
	wr(R_DOORBELL, 0);
	u.ep0_stage = 1;
	u.ep0_status_dir = TRB_DIR;     /* after an IN data stage the host sends an OUT zero-length status */
}

static void
ep0_stall(void)
{
	ring_put(&u.ep0, 0, 0, TRB_STATUS << 10 | TRB_IOC | TRB_DIR | TRB_TAG(u.setup_tag) | TRB_STALL);
	wr(R_DOORBELL, 0);
	u.ep0_stage = 2;
}

static void
out_queue(void)
{
	ring_put(&u.out, pa32(OFF_OUT_BUF), 512, TRB_NORMAL << 10 | TRB_IOC | TRB_ISP);
	wr(R_DOORBELL, DCI_OUT);
}

static void
configure(void)
{
	volatile uint32_t *cx = mem32(OFF_DCBA + (DCI_OUT - 2) * 16);

	ring_init(&u.out, OFF_OUT_RING);
	cx[0] = (DCI_OUT / 2) << 3;
	cx[1] = 512u << 16 | 6u << 3;   /* bulk OUT */
	cx[2] = pa32(OFF_OUT_RING) | 1;
	cx[3] = 0;
	__builtin_arm_dsb(0xf);
	if (command(CMD_CONFIG_EP, 1u << DCI_OUT, 0) != 0) {
		printf("usbkeys: CONFIG_EP failed\n");
	}
	out_queue();
	u.configured = 1;
	printf("usbkeys: configured by the host\n");
}

static void
handle_setup(const volatile uint32_t *e)
{
	uint32_t w0 = e[0], w1 = e[1];
	uint8_t type = w0 & 0xff, req = (w0 >> 8) & 0xff;
	uint16_t value = w0 >> 16, index = w1 & 0xffff, length = w1 >> 16;
	volatile uint8_t *buf = (volatile uint8_t *)(u.blk + OFF_EP0_BUF);
	uint32_t n = 0;

	(void)index;
	u.setup_tag = (e[3] >> 21) & 3;
	if (type == 0x80 && req == 6) {                 /* GET_DESCRIPTOR */
		switch (value >> 8) {
		case 1:
			for (n = 0; n < sizeof(dev_desc); n++) buf[n] = dev_desc[n];
			break;
		case 2:
			for (n = 0; n < sizeof(cfg_desc); n++) buf[n] = cfg_desc[n];
			break;
		case 3:
			n = string_desc(value & 0xff, buf);
			break;
		default:
			ep0_stall();
			return;
		}
		if (n == 0) {
			ep0_stall();
			return;
		}
		ep0_data_in(n < length ? n : length);
	} else if (type == 0x00 && req == 5) {          /* SET_ADDRESS */
		command(CMD_SET_ADDR, value & 0x7f, 0);
		ep0_status(0, TRB_SET_ADDR);
		u.addressed = 1;
	} else if (type == 0x00 && req == 9) {          /* SET_CONFIGURATION */
		if (value == 1 && !u.configured) {
			configure();
		}
		ep0_status(0, 0);
	} else if ((type & 0x80) && req == 0) {         /* GET_STATUS */
		buf[0] = 0;
		buf[1] = 0;
		ep0_data_in(2 < length ? 2 : length);
	} else if (type == 0x80 && req == 8) {          /* GET_CONFIGURATION */
		buf[0] = (uint8_t)u.configured;
		ep0_data_in(1);
	} else if ((type & 0x60) == 0 && (req == 11 || req == 1 || req == 3)) {   /* SET_INTERFACE, CLEAR/SET_FEATURE */
		ep0_status(0, 0);
	} else {
		ep0_stall();
	}
}

static void
handle_out(const volatile uint32_t *e)
{
	uint32_t residual = e[2] & 0x1ffff, len = 512 - (residual <= 512 ? residual : 512);
	const volatile uint8_t *b = (const volatile uint8_t *)(u.blk + OFF_OUT_BUF);

	for (uint32_t i = 0; i + 8 <= len; i += 8) {
		uint32_t type = b[i] | b[i + 1] << 8 | b[i + 2] << 16 | (uint32_t)b[i + 3] << 24;
		uint32_t code = b[i + 4] | b[i + 5] << 8 | b[i + 6] << 16 | (uint32_t)b[i + 7] << 24;

		if (type == RECORD_STOP && code == STOP_CODE) {
			printf("\n--- usbkeys: the host says stop: back to the loader ---\n");
			loader_return_now();
		}
		if (type == HAKIOS_BUTTON_DOWN || type == HAKIOS_BUTTON_UP) {
			hakios_touch_post(type, code, 0, 0);
			if (u.keys++ < 32) {
				printf("usbkeys: key %#x %s\n", code, type == HAKIOS_BUTTON_DOWN ? "down" : "up");
			}
		}
	}
	out_queue();
}

static void
handle_port(void)
{
	uint32_t ps = rd(R_PORTSC);

	wr(R_PORTSC, ps);                               /* the change bits are write-one-to-clear */
	if ((ps & PORTSC_PRC) || ((ps & PORTSC_CSC) && (ps & PORTSC_CCS))) {
		IODelay(3000);
		ps = rd(R_PORTSC);
		if (!(ps & PORTSC_PR) && (ps & PORTSC_PED) && PORTSC_PLS(ps) == 0) {
			command(CMD_UPDATE_EP0, 64u << 16, 0);
			wr(R_CONFIG1, rd(R_CONFIG1) | 1u << 16);        /* SETUP events */
			printf("usbkeys: bus reset, speed %u\n", PORTSC_SPEED(ps));
			u.ep0_stage = 0;
			u.configured = 0;
		}
	}
	if ((ps & PORTSC_PLC) && PORTSC_PLS(ps) == 0xf) {
		wr(R_PORTSC, ps & ~PORTSC_W1C & ~(0xfu << 5));
	}
}

static void
poll_events(void)
{
	uint32_t handled = 0;

	for (;;) {
		volatile uint32_t *e = mem32(OFF_EVENTS + u.ev_idx * 16);
		uint32_t dw3 = e[3];

		if ((dw3 & 1) != u.ev_ccs) {
			break;
		}
		__builtin_arm_dmb(0xf);
		uint32_t type = (dw3 >> 10) & 0x3f, dci = (dw3 >> 16) & 0x1f, code = e[2] >> 24;
		u.events++;
		if (type == EVT_SETUP) {
			handle_setup(e);
		} else if (type == EVT_PORT) {
			handle_port();
		} else if (type == EVT_TRANSFER && dci == 0) {
			if (u.ep0_stage == 1 && (code == 1 || code == 13)) {
				ep0_status(u.ep0_status_dir, 0);
			} else {
				u.ep0_stage = 0;
			}
		} else if (type == EVT_TRANSFER && dci == DCI_OUT) {
			if (code == 1 || code == 13) {
				handle_out(e);
			} else {
				printf("usbkeys: bulk OUT completion %u\n", code);
			}
		}
		if (++u.ev_idx == N_EVENTS) {
			u.ev_idx = 0;
			u.ev_ccs ^= 1;
		}
		handled++;
	}
	if (handled) {
		uint32_t next = pa32(OFF_EVENTS + u.ev_idx * 16);
		wr(R_IMAN, rd(R_IMAN) | 1);
		wr(R_ERDPHI, 0);
		wr(R_ERDPLO, next | 8);
	}
	if (rd(R_STATUS) & 0xc) {
		wr(R_STATUS, rd(R_STATUS) & 0xc);
	}
}

static void
usbkeys_thread(__unused void *arg, __unused wait_result_t wr_)
{
	for (;;) {
		poll_events();
		IOSleep(u.configured ? 4 : 2);
	}
}

void
hakios_crgudc_init(void)
{
	uint32_t on = 0;
	vm_offset_t va;
	thread_t thread;

	if (!PE_parse_boot_argn("usbkeys", &on, sizeof(on)) || !on) {
		return;
	}
	if (kmem_alloc_contig(kernel_map, &va, BLK_SIZE, PAGE_MASK, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("usbkeys: no memory\n");
		return;
	}
	FlushPoC_DcacheRegion(va, BLK_SIZE);
	u.pa = kvtophys(va);
	u.blk = ml_io_map(u.pa, BLK_SIZE);
	u.regs = ml_io_map(CRG_BASE, 0x8000);
	if (u.pa >> 32) {
		printf("usbkeys: block above 4 GB\n");
		return;
	}

	/* stop what U-Boot left, device role, no VBUS detection, reset */
	wr(R_CONTROL, rd(R_CONTROL) & ~(CTRL_RUN | CTRL_INT_EN));
	IOSleep(5);
	wr(R_ROLE, rd(R_ROLE) | 1);
	wr(R_VBUS, rd(R_VBUS) & ~0x1fu);
	wr(R_CONTROL, rd(R_CONTROL) | CTRL_SWRST);
	for (int i = 0; i < 50 && (rd(R_CONTROL) & CTRL_SWRST); i++) {
		IOSleep(10);
	}
	wr(R_U3PORTPM, rd(R_U3PORTPM) & ~((1u << 20) | (1u << 21) | 0xffffu));
	wr(R_CONTROL, rd(R_CONTROL) & ~(CTRL_INT_EN | CTRL_RUN));
	wr(R_CONFIG0, 3);                               /* high speed */

	/* the event ring */
	for (uint32_t i = 0; i < N_EVENTS * 4; i++) {
		mem32(OFF_EVENTS)[i] = 0;
	}
	mem32(OFF_ERST)[0] = pa32(OFF_EVENTS);
	mem32(OFF_ERST)[1] = 0;
	mem32(OFF_ERST)[2] = N_EVENTS;
	mem32(OFF_ERST)[3] = 0;
	__builtin_arm_dsb(0xf);
	u.ev_idx = 0;
	u.ev_ccs = 1;
	wr(R_ERSTSZ, 1);
	wr(R_ERSTBALO, pa32(OFF_ERST));
	wr(R_ERSTBAHI, 0);
	wr(R_ERDPLO, pa32(OFF_EVENTS) | 8);
	wr(R_ERDPHI, 0);
	wr(R_IMAN, 3);
	wr(R_IMOD, 500);
	wr(R_DCBAPLO, pa32(OFF_DCBA));
	wr(R_DCBAPHI, 0);

	/* EP0 */
	ring_init(&u.ep0, OFF_EP0_RING);
	command(CMD_INIT_EP0, pa32(OFF_EP0_RING) | 1, 0);

	/* connect */
	wr(R_CONFIG0, rd(R_CONFIG0) | 0xf0);
	IOSleep(3);
	wr(R_CONFIG1, rd(R_CONFIG1) | 0x7b);
	wr(R_CONTROL, rd(R_CONTROL) | CTRL_SYSERR_EN | CTRL_INT_EN);
	wr(R_CONTROL, rd(R_CONTROL) | CTRL_RUN);
	printf("usbkeys: controller running (portsc %#x, block at %#llx)\n", rd(R_PORTSC), u.pa);

	if (kernel_thread_start_priority(usbkeys_thread, NULL, HAKIOS_POLL_PRI, &thread) == KERN_SUCCESS) {   /* (HAKIOS_POLL_PRI: below the window server's render threads) */
		thread_set_thread_name(thread, "usb keys");
		thread_deallocate(thread);
	}
}

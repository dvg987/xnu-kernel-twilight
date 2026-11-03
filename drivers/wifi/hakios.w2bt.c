/*
 * The Bluetooth of the Amlogic W2, and on it the box's own remote (Xiaomi's "MI RC", 2717:32b9, a BLE HID device).
 * Boot-arg w2bt=1: the controller up (firmware, HCI) and a scan, for the log; w2bt=2: and the remote found, paired
 * (legacy "Just Works": the controller's LE Encrypt does the AES), its HID service discovered, its reports turned into
 * keys of /dev/hakios-touch -- where backboardd's twilight remote, which says it is a Siri Remote, takes them.
 *
 * The controller: the chip's BT core, its firmware loaded by us, HCI through rings in its shared memory read and
 * written over the chip's Bluetooth endpoints (hakios.xhci.c; tvos/docs/w2-bt.md). The host: one thread, polling,
 * with the least of L2CAP (signalling, ATT, SMP), ATT as a client and SMP as the initiator, for one peripheral.
 * The remote's key is kept for this boot only: it is paired again after every start (Home + "Xiaomi TV", 5 s).
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/random.h>
#include <sys/systm.h>
#include <sys/hakios.input.h>
#include <sys/hakios.xhci.h>

extern void IOSleep(unsigned int milliseconds);
extern void IODelay(unsigned int microseconds);

/* the controller's shared memory */
#define BT_STATUS       0x005101fcu
#define CMD_RING        0x00518000u
#define CMD_RING_SIZE   0x1000u
#define CMD_RD          0x00510000u
#define CMD_WR          0x00510004u
#define TX_TABLE        0x00510018u
#define TX_BUF          0x00508000u
#define TX_BUF_STRIDE   0x408u
#define RX_BLOCK        0x00514000u
#define RX_BLOCK_SIZE   0x93cu
#define EVT_RING_SIZE   0x800u
#define ACL_RING        0x00500000u
#define ACL_RING_SIZE   0x1020u

#define EV_DISCONNECT   0x05
#define EV_ENC_CHANGE   0x08
#define EV_CMD_COMPLETE 0x0e
#define EV_CMD_STATUS   0x0f
#define EV_LE_META      0x3e

#define CID_ATT         4
#define CID_SIGNAL      5
#define CID_SMP         6

static struct {
	unsigned step, stop_at;             /* stop_at (boot-arg w2bt_stop): 1 before the firmware, 2 before its CPU runs,
	                                     * 3 before HCI: then only a line every 5 s (for finding what stops the box) */
	uint32_t type_r, evt_r, acl_r, cmd_w;
	bool active;
	uint8_t rx[RX_BLOCK_SIZE];
	uint8_t acl_ring[ACL_RING_SIZE];
	/* the last command's answer */
	uint16_t waiting_op;
	bool answered;
	uint8_t answer[64];
	unsigned answer_length;
	uint8_t own[6];
	/* the remote */
	enum { BT_IDLE, BT_SCANNING, BT_CONNECTING, BT_CONNECTED } state;
	bool found;
	uint8_t peer[6], peer_type;
	uint16_t handle;
	bool encrypted;
	unsigned enc_status;
	bool enc_done;
	/* SMP, as the initiator */
	uint8_t preq[7], pres[7], mrand[16], sconfirm[16], srand[16], ltk[16], rand[8];
	uint16_t ediv;
	bool have_pres, have_sconfirm, have_srand, have_ltk, have_ediv, smp_failed, bonded;
	/* ATT, as a client: one request at a time */
	uint8_t att_waiting;
	bool att_answered;
	uint8_t att[256];
	unsigned att_length;
	/* the reports of the HID service: handle -> report id */
	struct { uint16_t handle, cccd; uint8_t id, type; } reports[16];
	unsigned report_count;
	uint16_t pressed[4];                /* the consumer usages down now */
	bool update_pending;                /* the remote asked for other connection parameters */
	uint8_t update[8];
	uint32_t keyboard_down;             /* the keyboard usage down now (one) */
	unsigned keys;
} b;

static inline uint16_t le16(const uint8_t *p) { return p[0] | p[1] << 8; }
static inline uint32_t le32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static inline void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }

static uint64_t
now_ms(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / 1000000ull;
}

/* ---- the controller's start ---- */

static bool
rmw(uint32_t addr, uint32_t clear, uint32_t set)
{
	uint32_t v;

	return hakios_w2bt_read(addr, &v) && hakios_w2bt_write(addr, (v & ~clear) | set);
}

static void
rings_init(void)
{
	static const uint32_t zero_at[] = { CMD_RD, CMD_WR, RX_BLOCK + 0x00, RX_BLOCK + 0x04, RX_BLOCK + 0x08, RX_BLOCK + 0x0c,
		                            RX_BLOCK + 0x10, RX_BLOCK + 0x20, RX_BLOCK + 0x24 };
	uint8_t table[0x80];
	uint32_t v = 0;

	hakios_w2bt_read(BT_STATUS, &v);
	hakios_w2bt_write(BT_STATUS, v | 2);
	for (unsigned i = 0; i < sizeof(zero_at) / sizeof(zero_at[0]); i++) {
		hakios_w2bt_write(zero_at[i], 0);
	}
	bzero(table, sizeof(table));
	for (unsigned i = 0; i < 8; i++) {
		put32(table + 16 * i, 0xffffffffu);
	}
	hakios_w2bt_sram_write(TX_TABLE, table, sizeof(table));
	hakios_w2bt_write(BT_STATUS, 0);
	b.type_r = b.evt_r = b.acl_r = b.cmd_w = 0;
}

static bool
controller_start(void)
{
	uint64_t size = 0;
	const uint8_t *fw = hakios_image_file("fw", "w2_bt_fw_usb.bin", &size, NULL);
	uint32_t v = 0, iccm, pc[3] = { 0, 0, 0 };
	uint8_t manf[48];

	if (fw == NULL || size < 8) {
		printf("w2bt: no fw/w2_bt_fw_usb.bin in the small image\n");
		return false;
	}
	iccm = le32(fw);
	if (iccm != 0x80000 || le32(fw + 4) != 0x20000 || size < 8 + iccm + 0x20000) {
		printf("w2bt: fw/w2_bt_fw_usb.bin is not laid out as expected (%#x, %#x, %llu bytes)\n", iccm, le32(fw + 4), size);
		return false;
	}
	if (b.stop_at == 1) {
		return false;
	}
	rmw(0x00f00060, (1u << 27) | (1u << 26), 1u << 24);
	hakios_w2bt_read(0x00f0303c, &v);
	if ((v & 0xf) == 7 || (v & 0xf) == 8) {
		IOSleep(1);
		rmw(0x00f03040, 3, 2);
	}
	for (int i = 0; i < 51 && hakios_w2bt_read(0x00f00044, &v) && (v & (1u << 29)); i++) {
		IOSleep(20);
	}
	hakios_w2bt_write(0x00f03058, 0x00050005);
	IOSleep(1);
	hakios_w2bt_write(0x00f03058, 0x00040004);              /* its CPU held in reset */
	rings_init();
	hakios_w2bt_write(0x00f03050, 0);                       /* its RAM powered */
	for (uint32_t off = 0; off < 0x40000; off += 0x1000) {
		if (!hakios_w2bt_sram_write(0x00340000 + off, fw + 8 + 0x40000 + off, 0x1000)) {
			printf("w2bt: firmware (ICCM) at %#x: the bus failed\n", off);
			return false;
		}
	}
	for (uint32_t off = 0; off < 0x20000; off += 0x1000) {
		if (!hakios_w2bt_sram_write(0x00400000 + off, fw + 8 + iccm + off, 0x1000)) {
			printf("w2bt: firmware (DCCM) at %#x: the bus failed\n", off);
			return false;
		}
	}
	{
		/* a page of each read back, to know that what went there is there */
		static uint8_t check[0x1000];

		if (!hakios_w2bt_sram_read(0x00340000, check, sizeof(check)) || memcmp(check, fw + 8 + 0x40000, sizeof(check)) != 0) {
			printf("w2bt: the firmware did not stay in the BT core's memory\n");
			return false;
		}
	}
	rmw(0x00f06308, 3, 0);                                  /* isolation 0 */
	rmw(0x00f03040, ~0xedffffffu, 2u << 28);                /* antenna 2, sink 0 */
	rmw(0x00f000d4, ~0xff4f0000u, 66 | 98 << 8);            /* BR and EDR digital gains */
	rmw(0x00f000ec, 3, 0);                                  /* no firmware log */
	bzero(manf, sizeof(manf));
	hakios_w2bt_sram_write(0x00514970, manf, sizeof(manf));
	if (b.stop_at == 2) {
		printf("w2bt: firmware loaded; its CPU left in reset (w2bt_stop=2)\n");
		return false;
	}
	rmw(0x00f000d0, 0, 1);
	hakios_w2bt_write(0x00f03058, 0);                       /* its CPU runs */
	for (int i = 0; i < 3; i++) {
		IOSleep(10);
		hakios_w2bt_read(0x00200034, &pc[i]);
	}
	hakios_w2bt_read(0x00f0303c, &v);
	printf("w2bt: firmware loaded (%u KiB); the BT core runs (PMU state %u, pc %#x %#x %#x)\n", (0x40000 + 0x20000) / 1024,
	    v & 0xf, pc[0], pc[1], pc[2]);
	return true;
}

/* ---- HCI ---- */

static void le_meta(const uint8_t *p, unsigned n);
static void acl_received(uint16_t handle, const uint8_t *p, unsigned n);

static void
event(uint8_t code, const uint8_t *p, unsigned n)
{
	switch (code) {
	case EV_CMD_COMPLETE:
		if (n >= 3 && le16(p + 1) == b.waiting_op) {
			b.answer_length = n - 3 < sizeof(b.answer) ? n - 3 : sizeof(b.answer);
			memcpy(b.answer, p + 3, b.answer_length);
			b.answered = true;
		}
		break;
	case EV_CMD_STATUS:
		if (n >= 4 && le16(p + 2) == b.waiting_op) {
			b.answer[0] = p[0];
			b.answer_length = 1;
			b.answered = true;
		}
		break;
	case EV_LE_META:
		le_meta(p, n);
		break;
	case EV_ENC_CHANGE:
		if (n >= 4 && le16(p + 1) == b.handle) {
			b.enc_status = p[0];
			b.encrypted = p[0] == 0 && p[3] != 0;
			b.enc_done = true;
			printf("w2bt: encryption %s (status %#x)\n", b.encrypted ? "on" : "NOT on", p[0]);
		}
		break;
	case EV_DISCONNECT:
		if (n >= 4 && le16(p + 1) == b.handle && b.state != BT_IDLE) {
			printf("w2bt: the remote is gone (reason %#x)\n", p[3]);
			b.state = BT_IDLE;
			b.encrypted = false;
			/* whatever was held down is let go */
			for (int i = 0; i < 4; i++) {
				b.pressed[i] = 0;
			}
		}
		break;
	case 0x13:                                              /* number of completed packets */
		break;
	default:
		if (b.step >= 1 && code != 0x13) {
			printf("w2bt: event %#x (%u bytes)\n", code, n);
		}
		break;
	}
}

/* what the controller has for us: events and ACL data out of its rings */
static void
poll_controller(void)
{
	static uint8_t lin[EVT_RING_SIZE];
	uint32_t type_w, evt_w, acl_w;
	uint8_t ack[16];

	if (!hakios_w2bt_sram_read(RX_BLOCK, b.rx, RX_BLOCK_SIZE)) {
		return;
	}
	type_w = le32(b.rx + 0x20) & 0xff;
	evt_w = le32(b.rx + 0x24) & 0x1fff;
	acl_w = le32(b.rx + 0x10) & 0x1fff;
	b.active = le32(b.rx + 0x14) != 0;
	if (type_w == b.type_r) {
		return;
	}
	b.type_r = type_w;
	if (evt_w != b.evt_r && evt_w < EVT_RING_SIZE) {
		unsigned n = (evt_w - b.evt_r + EVT_RING_SIZE) % EVT_RING_SIZE;

		for (unsigned i = 0; i < n; i++) {
			lin[i] = b.rx[0x13c + (b.evt_r + i) % EVT_RING_SIZE];
		}
		for (unsigned o = 0; o + 3 <= n;) {
			unsigned plen = lin[o + 2], len = (plen + 6) & ~3u;

			if (lin[o] != 0x04 || o + 3 + plen > n) {
				break;
			}
			event(lin[o + 1], lin + o + 3, plen);
			o += len;
		}
		b.evt_r = evt_w;
	}
	if (acl_w != b.acl_r && acl_w < ACL_RING_SIZE && hakios_w2bt_sram_read(ACL_RING, b.acl_ring, ACL_RING_SIZE)) {
		static uint8_t lacl[ACL_RING_SIZE];
		unsigned n = (acl_w - b.acl_r + ACL_RING_SIZE) % ACL_RING_SIZE;

		for (unsigned i = 0; i < n; i++) {
			lacl[i] = b.acl_ring[(b.acl_r + i) % ACL_RING_SIZE];
		}
		for (unsigned o = 0; o + 8 <= n;) {
			unsigned len = le16(lacl + o + 6);

			if (lacl[o] != 0x02 || len >= 0x405 || o + 8 + len > n) {
				break;
			}
			acl_received(le16(lacl + o + 4), lacl + o + 8, len);
			o += (len + 11) & ~3u;
		}
		b.acl_r = acl_w;
	}
	bzero(ack, sizeof(ack));
	put32(ack, b.type_r & 0xff);
	put32(ack + 4, b.evt_r & 0x1fff);
	put32(ack + 12, b.acl_r & 0x1fff);
	hakios_w2bt_sram_write(RX_BLOCK, ack, sizeof(ack));
}

/* a command, and its answer (Command Complete's return parameters, or Command Status's status); -1: none */
static int
cmd(uint16_t op, const void *params, uint8_t plen, void *out, unsigned out_length)
{
	uint8_t c[260];
	unsigned len = 3 + plen, l4 = (len + 3) & ~3u;
	uint32_t r = 0, used;
	uint64_t began = now_ms();

	put16(c, op);
	c[2] = plen;
	if (plen != 0) {
		memcpy(c + 3, params, plen);
	}
	bzero(c + len, l4 - len);
	hakios_w2bt_read(CMD_RD, &r);
	used = (b.cmd_w - (r & 0xfff) + CMD_RING_SIZE) % CMD_RING_SIZE;
	if (CMD_RING_SIZE - used - 4 < l4) {
		printf("w2bt: command %#x: the ring is full\n", op);
		return -1;
	}
	if (b.cmd_w + l4 <= CMD_RING_SIZE) {
		hakios_w2bt_sram_write(CMD_RING + b.cmd_w, c, l4);
	} else {
		unsigned first = CMD_RING_SIZE - b.cmd_w;

		hakios_w2bt_sram_write(CMD_RING + b.cmd_w, c, first);
		hakios_w2bt_sram_write(CMD_RING, c + first, l4 - first);
	}
	b.cmd_w = (b.cmd_w + l4) % CMD_RING_SIZE;
	b.waiting_op = op;
	b.answered = false;
	hakios_w2bt_write(CMD_WR, b.cmd_w);
	while (!b.answered && now_ms() - began < 2000) {
		poll_controller();
		if (!b.answered) {
			IOSleep(1);
		}
	}
	b.waiting_op = 0;
	if (!b.answered) {
		printf("w2bt: command %#x: no answer\n", op);
		return -1;
	}
	if (out != NULL) {
		memcpy(out, b.answer, out_length < b.answer_length ? out_length : b.answer_length);
	}
	return b.answer[0];
}

/* one ACL packet (an L2CAP frame on `cid'); the controller's table has eight places for them */
static bool
acl_send(uint16_t cid, const uint8_t *payload, unsigned n)
{
	uint8_t table[0x80], pkt[8 + 64], entry[16];
	unsigned slot, prio = 0;

	if (n > 64 || !hakios_w2bt_sram_read(TX_TABLE, table, sizeof(table))) {
		return false;
	}
	for (slot = 0; slot < 8; slot++) {
		uint32_t status = le32(table + 16 * slot + 8);

		if (status == 2) {
			put32(table + 16 * slot, 0xffffffffu);
			put32(table + 16 * slot + 4, 0);
			put32(table + 16 * slot + 8, 0);
		}
	}
	for (unsigned i = 0; i < 8; i++) {
		if (le32(table + 16 * i + 8) == 1 && le32(table + 16 * i + 4) == b.handle && le32(table + 16 * i) + 1 > prio) {
			prio = le32(table + 16 * i) + 1;
		}
	}
	for (slot = 0; slot < 8 && le32(table + 16 * slot + 8) != 0; slot++) {
	}
	if (slot == 8) {
		printf("w2bt: no room to send\n");
		return false;
	}
	put16(pkt, b.handle | 0x2000);                          /* the first part of an L2CAP frame, flushable */
	put16(pkt + 2, (uint16_t)(4 + n));
	put16(pkt + 4, (uint16_t)n);
	put16(pkt + 6, cid);
	memcpy(pkt + 8, payload, n);
	bzero(pkt + 8 + n, ((8 + n + 3) & ~3u) - (8 + n));
	put32(entry, prio);
	put32(entry + 4, b.handle);
	put32(entry + 8, 1);
	put32(entry + 12, 0);
	return hakios_w2bt_sram_write(TX_BUF + slot * TX_BUF_STRIDE, pkt, (8 + n + 3) & ~3u) &&
	       hakios_w2bt_sram_write(TX_TABLE + 16 * slot, entry, sizeof(entry));
}

/* ---- the remote ---- */

static bool
named_remote(const uint8_t *ad, unsigned n)
{
	bool hid = false, name = false;

	for (unsigned i = 0; i + 1 < n && ad[i] != 0 && i + 1 + ad[i] <= n; i += 1 + ad[i]) {
		unsigned len = ad[i] - 1, type = ad[i + 1];
		const uint8_t *v = ad + i + 2;

		if ((type == 0x08 || type == 0x09) && ((len == 5 && memcmp(v, "MI RC", 5) == 0) || (len == 9 &&
		    memcmp(v, "Xiaomi RC", 9) == 0))) {
			name = true;
		}
		if (type == 0x02 || type == 0x03) {
			for (unsigned k = 0; k + 1 < len; k += 2) {
				hid |= le16(v + k) == 0x1812;
			}
		}
	}
	return name || hid;
}

static void
le_meta(const uint8_t *p, unsigned n)
{
	if (n < 1) {
		return;
	}
	if (p[0] == 0x02 && b.state == BT_SCANNING) {             /* advertising reports */
		unsigned count = p[1], o = 2;

		for (unsigned r = 0; r < count && o + 9 <= n; r++) {
			unsigned type = p[o], atype = p[o + 1], len = p[o + 8];
			const uint8_t *addr = p + o + 2, *ad = p + o + 9;

			if (o + 9 + len + 1 > n) {
				break;
			}
			if ((type == 0 || type == 1) && !b.found && (named_remote(ad, len) || (b.bonded && memcmp(addr, b.peer, 6) == 0))) {
				memcpy(b.peer, addr, 6);
				b.peer_type = (uint8_t)atype;
				b.found = true;
				printf("w2bt: the remote advertises (%02x:%02x:%02x:%02x:%02x:%02x, %d dBm)\n", addr[5], addr[4], addr[3],
				    addr[2], addr[1], addr[0], (int8_t)ad[len]);
			} else if (b.step == 1) {
				/* each one once */
				static uint8_t told[16][6];
				static unsigned told_count;
				bool known = false;

				for (unsigned k = 0; k < told_count; k++) {
					known |= memcmp(told[k], addr, 6) == 0;
				}
				if (!known && told_count < 16) {
					memcpy(told[told_count++], addr, 6);
					printf("w2bt:   heard %02x:%02x:%02x:%02x:%02x:%02x (%d dBm, %u bytes)\n", addr[5], addr[4], addr[3],
					    addr[2], addr[1], addr[0], (int8_t)ad[len], len);
				}
			}
			o += 9 + len + 1;
		}
	} else if (p[0] == 0x01 && n >= 19) {                  /* connection complete */
		if (p[1] != 0) {
			printf("w2bt: the connection failed (%#x)\n", p[1]);
			b.state = BT_IDLE;
			return;
		}
		b.handle = le16(p + 2) & 0x0fff;
		b.state = BT_CONNECTED;
		printf("w2bt: connected to the remote (handle %#x, interval %u x 1.25 ms)\n", b.handle, le16(p + 12));
	}
}

static void
smp_received(const uint8_t *p, unsigned n)
{
	switch (p[0]) {
	case 0x02:                                              /* pairing response */
		if (n >= 7) {
			memcpy(b.pres, p, 7);
			b.have_pres = true;
		}
		break;
	case 0x03:
		if (n >= 17) {
			memcpy(b.sconfirm, p + 1, 16);
			b.have_sconfirm = true;
		}
		break;
	case 0x04:
		if (n >= 17) {
			memcpy(b.srand, p + 1, 16);
			b.have_srand = true;
		}
		break;
	case 0x05:
		printf("w2bt: pairing failed (reason %#x)\n", n > 1 ? p[1] : 0);
		b.smp_failed = true;
		break;
	case 0x06:                                              /* encryption information: the long-term key */
		if (n >= 17) {
			memcpy(b.ltk, p + 1, 16);
			b.have_ltk = true;
		}
		break;
	case 0x07:                                              /* central identification: EDIV, Rand */
		if (n >= 11) {
			b.ediv = le16(p + 1);
			memcpy(b.rand, p + 3, 8);
			b.have_ediv = true;
		}
		break;
	case 0x0b:                                              /* security request: we start it anyway */
		break;
	default:
		break;
	}
}

/* a key down or up, for backboardd's twilight remote (consumer page) or keyboard */
static void
key(uint32_t code, bool down)
{
	hakios_touch_post(down ? HAKIOS_BUTTON_DOWN : HAKIOS_BUTTON_UP, code, 0, 0);
	if (b.keys++ < 40) {
		printf("w2bt: key %#x %s\n", code, down ? "down" : "up");
	}
}

/* the remote's consumer usage -> the code of a key record: what tvOS does with each was found on the box */
static uint32_t
consumer_key(uint16_t usage)
{
	switch (usage) {
	case 0x41: case 0x42: case 0x43: case 0x44: case 0x45:
		return 0x0c0000u | usage;                       /* select, up, down, left, right */
	case 0x224:
		return 0x070029;                                /* Back: a keyboard's Escape is tvOS's back */
	case 0x223:
		return 0x0c0040;                                /* Home: the consumer Menu is tvOS's home */
	case 0xcd: case 0xe9: case 0xea: case 0xe2:
		return 0x0c0000u | usage;                       /* play/pause, volume, mute */
	default:
		return 0;
	}
}

/* a report of the consumer page: up to three 16-bit usages, those down now */
static void
consumer_report(const uint8_t *p, unsigned n)
{
	uint16_t now[4] = { 0, 0, 0, 0 };

	for (unsigned i = 0; i < 3 && 2 * i + 1 < n; i++) {
		now[i] = le16(p + 2 * i);
	}
	for (int i = 0; i < 4; i++) {                           /* let go */
		bool still = false;

		for (int k = 0; k < 4; k++) {
			still |= b.pressed[i] != 0 && b.pressed[i] == now[k];
		}
		if (b.pressed[i] != 0 && !still && consumer_key(b.pressed[i]) != 0) {
			key(consumer_key(b.pressed[i]), false);
		}
	}
	for (int k = 0; k < 4; k++) {                           /* newly down */
		bool before = false;

		for (int i = 0; i < 4; i++) {
			before |= now[k] != 0 && b.pressed[i] == now[k];
		}
		if (now[k] != 0 && !before) {
			if (consumer_key(now[k]) != 0) {
				key(consumer_key(now[k]), true);
			} else if (b.keys < 40) {
				printf("w2bt: button %#x (not mapped)\n", now[k]);
			}
		}
	}
	memcpy(b.pressed, now, sizeof(now));
}

static void
att_received(const uint8_t *p, unsigned n)
{
	uint8_t r[8];

	if (n == 0) {
		return;
	}
	if (p[0] == 0x1b || p[0] == 0x1d) {                     /* a notification (or indication) */
		uint16_t h = n >= 3 ? le16(p + 1) : 0;

		if (p[0] == 0x1d) {
			r[0] = 0x1e;                            /* confirmed */
			acl_send(CID_ATT, r, 1);
		}
		for (unsigned i = 0; i < b.report_count; i++) {
			if (b.reports[i].handle == h && b.reports[i].id == 1) {
				consumer_report(p + 3, n - 3);
			}
		}
		return;
	}
	if (p[0] == 0x02) {                                     /* its MTU exchange: ours is the least */
		r[0] = 0x03;
		put16(r + 1, 23);
		acl_send(CID_ATT, r, 3);
		return;
	}
	if ((p[0] & 1) == 0 && p[0] != 0x1e && (p[0] < 0x01 || p[0] > 0x1e || (p[0] != 0x13 && p[0] != 0x11 &&
	    p[0] != 0x05 && p[0] != 0x0b && p[0] != 0x09 && p[0] != 0x0d))) {
		/* a request of its own to our (absent) server: not supported */
		r[0] = 0x01;
		r[1] = p[0];
		put16(r + 2, 0);
		r[4] = 0x06;
		acl_send(CID_ATT, r, 5);
		return;
	}
	if (b.att_waiting != 0 && (p[0] == b.att_waiting + 1 || p[0] == 0x01)) {
		b.att_length = n < sizeof(b.att) ? n : sizeof(b.att);
		memcpy(b.att, p, b.att_length);
		b.att_answered = true;
	}
}

static void
signal_received(const uint8_t *p, unsigned n)
{
	uint8_t r[8];

	if (n >= 12 && p[0] == 0x12) {                          /* connection parameter update request: yes */
		r[0] = 0x13;
		r[1] = p[1];
		put16(r + 2, 2);
		put16(r + 4, 0);
		acl_send(CID_SIGNAL, r, 6);
		memcpy(b.update, p + 4, 8);                     /* interval min, max, latency, timeout: for the main loop */
		b.update_pending = true;
	} else if (n >= 1 && (p[0] & 1) == 0) {
		r[0] = 0x01;                                    /* command reject: not understood */
		r[1] = n > 1 ? p[1] : 0;
		put16(r + 2, 2);
		put16(r + 4, 0);
		acl_send(CID_SIGNAL, r, 6);
	}
}

static void
acl_received(uint16_t handle, const uint8_t *p, unsigned n)
{
	uint16_t cid;

	if ((handle & 0x0fff) != b.handle || n < 4) {
		return;
	}
	cid = le16(p + 2);
	if (le16(p) + 4 > n) {
		return;
	}
	if (cid == CID_SMP) {
		smp_received(p + 4, le16(p));
	} else if (cid == CID_ATT) {
		att_received(p + 4, le16(p));
	} else if (cid == CID_SIGNAL) {
		signal_received(p + 4, le16(p));
	}
}

/* waits (polling) for `flag', up to `ms'; false if the remote is gone or the time is out */
static bool
wait_for(volatile bool *flag, unsigned ms)
{
	uint64_t began = now_ms();

	while (!*flag && b.state == BT_CONNECTED && now_ms() - began < ms) {
		poll_controller();
		if (!*flag) {
			IOSleep(2);
		}
	}
	return *flag;
}

/* e(k, plaintext), AES-128 by the controller (LE Encrypt); everything least significant byte first */
static bool
aes(const uint8_t k[16], const uint8_t in[16], uint8_t out[16])
{
	uint8_t q[32], a[17];

	memcpy(q, k, 16);
	memcpy(q + 16, in, 16);
	if (cmd(0x2017, q, 32, a, sizeof(a)) != 0) {
		return false;
	}
	memcpy(out, a + 1, 16);
	return true;
}

/* the confirm value c1 (Core 5.3, Vol 3 Part H 2.2.3), legacy pairing, we the initiator */
static bool
c1(const uint8_t tk[16], const uint8_t r[16], uint8_t out[16])
{
	uint8_t p1[16], p2[16], t[16];

	p1[0] = 0;                                              /* our address type: public */
	p1[1] = b.peer_type;
	memcpy(p1 + 2, b.preq, 7);
	memcpy(p1 + 9, b.pres, 7);
	memcpy(p2, b.peer, 6);
	memcpy(p2 + 6, b.own, 6);
	bzero(p2 + 12, 4);
	for (int i = 0; i < 16; i++) {
		t[i] = r[i] ^ p1[i];
	}
	if (!aes(tk, t, t)) {
		return false;
	}
	for (int i = 0; i < 16; i++) {
		t[i] ^= p2[i];
	}
	return aes(tk, t, out);
}

/* legacy pairing, Just Works (TK 0), as the initiator; the short-term key then the remote's long-term key */
static bool
pair(void)
{
	uint8_t tk[16], m[17], confirm[16], stk[16], r[16], enc[28];

	bzero(tk, sizeof(tk));
	b.have_pres = b.have_sconfirm = b.have_srand = b.have_ltk = b.have_ediv = b.smp_failed = false;
	b.preq[0] = 0x01;
	b.preq[1] = 0x03;                                       /* no input, no output */
	b.preq[2] = 0x00;
	b.preq[3] = 0x01;                                       /* bonding; no MITM, no secure connections */
	b.preq[4] = 16;
	b.preq[5] = 0x00;                                       /* we give no keys */
	b.preq[6] = 0x01;                                       /* it gives its encryption key */
	acl_send(CID_SMP, b.preq, 7);
	if (!wait_for(&b.have_pres, 5000) || b.smp_failed) {
		printf("w2bt: pairing: no response\n");
		return false;
	}
	printf("w2bt: pairing: the remote answers (io %#x, auth %#x, key %u, gives %#x)\n", b.pres[1], b.pres[3], b.pres[4],
	    b.pres[6]);
	read_random(b.mrand, sizeof(b.mrand));
	if (!c1(tk, b.mrand, confirm)) {
		return false;
	}
	m[0] = 0x03;
	memcpy(m + 1, confirm, 16);
	acl_send(CID_SMP, m, 17);
	if (!wait_for(&b.have_sconfirm, 5000) || b.smp_failed) {
		printf("w2bt: pairing: no confirm\n");
		return false;
	}
	m[0] = 0x04;
	memcpy(m + 1, b.mrand, 16);
	acl_send(CID_SMP, m, 17);
	if (!wait_for(&b.have_srand, 5000) || b.smp_failed) {
		printf("w2bt: pairing: no random\n");
		return false;
	}
	if (!c1(tk, b.srand, confirm) || memcmp(confirm, b.sconfirm, 16) != 0) {
		printf("w2bt: pairing: the remote's confirm value does not check\n");
		return false;
	}
	/* STK = s1(TK, Srand, Mrand): the low halves, the remote's the more significant */
	memcpy(r, b.mrand, 8);
	memcpy(r + 8, b.srand, 8);
	if (!aes(tk, r, stk)) {
		return false;
	}
	put16(enc, b.handle);
	bzero(enc + 2, 10);                                     /* Rand 0, EDIV 0 */
	memcpy(enc + 12, stk, 16);
	b.enc_done = false;
	cmd(0x2019, enc, 28, NULL, 0);
	if (!wait_for(&b.enc_done, 5000) || !b.encrypted) {
		printf("w2bt: pairing: the link did not encrypt\n");
		return false;
	}
	if (wait_for(&b.have_ltk, 3000) && wait_for(&b.have_ediv, 3000)) {
		b.bonded = true;
		printf("w2bt: PAIRED: the remote gave its key (kept for this boot)\n");
	} else {
		printf("w2bt: paired, but the remote gave no key: it will have to pair again next time\n");
	}
	return true;
}

static bool
encrypt_bonded(void)
{
	uint8_t enc[28];

	put16(enc, b.handle);
	memcpy(enc + 2, b.rand, 8);
	put16(enc + 10, b.ediv);
	memcpy(enc + 12, b.ltk, 16);
	b.enc_done = false;
	cmd(0x2019, enc, 28, NULL, 0);
	return wait_for(&b.enc_done, 5000) && b.encrypted;
}

/* one ATT request; its answer in b.att */
static bool
att(const uint8_t *req, unsigned n)
{
	b.att_waiting = req[0];
	b.att_answered = false;
	if (!acl_send(CID_ATT, req, n)) {
		b.att_waiting = 0;
		return false;
	}
	wait_for(&b.att_answered, 3000);
	b.att_waiting = 0;
	return b.att_answered && b.att[0] != 0x01;
}

/* the HID service: its reports, their ids, and notifications on for those that are input */
static void
hid_discover(void)
{
	uint8_t q[8];
	uint16_t start = 1, hid_start = 0, hid_end = 0;
	uint16_t value_handle = 0;

	/* the primary services, for the HID one's range */
	while (start != 0 && hid_start == 0) {
		q[0] = 0x10;
		put16(q + 1, start);
		put16(q + 3, 0xffff);
		put16(q + 5, 0x2800);
		if (!att(q, 7) || b.att_length < 2) {
			break;
		}
		for (unsigned o = 2; o + b.att[1] <= b.att_length; o += b.att[1]) {
			uint16_t s = le16(b.att + o), e = le16(b.att + o + 2);

			if (b.att[1] == 6 && le16(b.att + o + 4) == 0x1812) {
				hid_start = s;
				hid_end = e;
			}
			start = e == 0xffff ? 0 : e + 1;
		}
	}
	if (hid_start == 0) {
		printf("w2bt: the remote has no HID service\n");
		return;
	}
	/* its attributes: report values, their client configuration and report reference descriptors */
	b.report_count = 0;
	for (start = hid_start; start != 0 && start <= hid_end;) {
		q[0] = 0x04;
		put16(q + 1, start);
		put16(q + 3, hid_end);
		if (!att(q, 5) || b.att[1] != 1) {
			break;
		}
		for (unsigned o = 2; o + 4 <= b.att_length; o += 4) {
			uint16_t h = le16(b.att + o), uuid = le16(b.att + o + 2);

			if (uuid == 0x2a4d && b.report_count < 16) {
				value_handle = h;
				b.reports[b.report_count].handle = h;
				b.reports[b.report_count].cccd = 0;
				b.reports[b.report_count].id = 0;
				b.reports[b.report_count].type = 0;
				b.report_count++;
			} else if (uuid == 0x2803 || uuid == 0x2a4a || uuid == 0x2a4b || uuid == 0x2a4c || uuid == 0x2a4e) {
				value_handle = 0;
			} else if (value_handle != 0 && uuid == 0x2902) {
				b.reports[b.report_count - 1].cccd = h;
			} else if (value_handle != 0 && uuid == 0x2908) {
				q[0] = 0x0a;
				put16(q + 1, h);
				{
					unsigned slot = b.report_count - 1;
					uint16_t keep_start = h;

					/* (the read's answer replaces the find's: what is left of it is read again from h + 1) */
					if (att(q, 3) && b.att_length >= 3) {
						b.reports[slot].id = b.att[1];
						b.reports[slot].type = b.att[2];
					}
					start = keep_start + 1;
					goto next_find;
				}
			}
			start = h + 1;
		}
next_find:
		;
	}
	/* notifications on for each input report */
	for (unsigned i = 0; i < b.report_count; i++) {
		printf("w2bt:   report %u (type %u) at %#x, configuration at %#x\n", b.reports[i].id, b.reports[i].type,
		    b.reports[i].handle, b.reports[i].cccd);
		if (b.reports[i].type == 1 && b.reports[i].cccd != 0) {
			q[0] = 0x12;
			put16(q + 1, b.reports[i].cccd);
			put16(q + 3, 0x0001);
			att(q, 5);
		}
	}
	printf("w2bt: THE REMOTE IS READY: %u reports\n", b.report_count);
}

static void
scan(bool on)
{
	uint8_t p[7] = { 0x00, 0x60, 0x00, 0x30, 0x00, 0x00, 0x00 }, e[2] = { on ? 1 : 0, 0 };

	if (on) {
		cmd(0x200b, p, 7, NULL, 0);                     /* passive, 60 ms every 60 ms... (interval 0x60, window 0x30) */
	}
	cmd(0x200c, e, 2, NULL, 0);
}

static void
bt_thread(__unused void *arg, __unused wait_result_t wr_)
{
	uint8_t a[16], mask[8] = { 0xff, 0xff, 0xfb, 0xff, 0x07, 0xf8, 0xbf, 0x3d }, lemask[8] = { 0x1f, 0, 0, 0, 0, 0, 0, 0 };
	uint64_t scan_started = 0;

	if (!controller_start() || b.stop_at == 3) {
		for (unsigned t = 0; b.stop_at != 0; t += 5) {
			uint32_t v = 0;

			IOSleep(5000);
			hakios_w2bt_read(0x00f0303c, &v);
			printf("w2bt: still here after %u s (stopped at stage %u; PMU state %u)\n", t + 5, b.stop_at, v & 0xf);
		}
		return;
	}
	if (cmd(0x0c03, NULL, 0, NULL, 0) != 0) {
		printf("w2bt: HCI reset: no good answer: no Bluetooth\n");
		return;
	}
	if (cmd(0x1009, NULL, 0, a, 7) == 0) {
		memcpy(b.own, a + 1, 6);
		printf("w2bt: HCI is up; the controller's address %02x:%02x:%02x:%02x:%02x:%02x\n", b.own[5], b.own[4], b.own[3],
		    b.own[2], b.own[1], b.own[0]);
	}
	if (cmd(0x1001, NULL, 0, a, 9) == 0) {
		printf("w2bt: HCI version %u (revision %#x), LMP %u, manufacturer %#x\n", a[1], le16(a + 2), a[4], le16(a + 5));
	}
	cmd(0x0c01, mask, 8, NULL, 0);
	cmd(0x2001, lemask, 8, NULL, 0);
	if (b.stop_at >= 4) {
		for (uint64_t began = now_ms(), told = 0;;) {
			if (b.stop_at == 4) {
				poll_controller();
			}
			IOSleep(8);
			if ((now_ms() - began) / 5000 > told) {
				told = (now_ms() - began) / 5000;
				printf("w2bt: still here after %llu s (HCI up, %s)\n", told * 5, b.stop_at == 4 ? "polling" : "not polling");
			}
		}
	}
	for (;;) {
		switch (b.state) {
		case BT_IDLE:
			b.found = false;
			scan(true);
			b.state = BT_SCANNING;
			scan_started = now_ms();
			if (b.step == 1) {
				printf("w2bt: scanning\n");
			}
			break;
		case BT_SCANNING:
			if (b.found && b.step >= 2) {
				uint8_t c[25];

				scan(false);
				put16(c, 0x0060);
				put16(c + 2, 0x0060);
				c[4] = 0;                       /* no filter: this address */
				c[5] = b.peer_type;
				memcpy(c + 6, b.peer, 6);
				c[12] = 0;                      /* we: public */
				put16(c + 13, 8);               /* interval 10 to 30 ms */
				put16(c + 15, 24);
				put16(c + 17, 0);
				put16(c + 19, 400);             /* supervision 4 s */
				put16(c + 21, 0);
				put16(c + 23, 0);
				b.state = BT_CONNECTING;
				cmd(0x200d, c, 25, NULL, 0);
				scan_started = now_ms();
			} else if (b.step == 1 && now_ms() - scan_started > 20000) {
				scan(false);
				printf("w2bt: the scan is over\n");
				b.step = 0;             /* (and quiet from here on) */
			}
			break;
		case BT_CONNECTING:
			if (now_ms() - scan_started > 10000) {
				cmd(0x200e, NULL, 0, NULL, 0);          /* cancel the connection */
				printf("w2bt: the remote did not take the connection\n");
				b.state = BT_IDLE;
			}
			break;
		case BT_CONNECTED:
			if (!b.encrypted) {
				bool ok = b.bonded ? encrypt_bonded() : pair();

				if (!ok || !b.encrypted) {
					uint8_t d[3];

					put16(d, b.handle);
					d[2] = 0x05;            /* authentication failure */
					cmd(0x0406, d, 3, NULL, 0);
					if (b.bonded && !ok) {
						printf("w2bt: the remote's key no longer works: pair it again\n");
						b.bonded = false;
					}
					IOSleep(500);
					break;
				}
				hid_discover();
			}
			if (b.update_pending) {
				uint8_t c[14];

				b.update_pending = false;
				put16(c, b.handle);
				memcpy(c + 2, b.update, 8);
				put16(c + 10, 0);
				put16(c + 12, 0);
				cmd(0x2013, c, 14, NULL, 0);
			}
			break;
		}
		poll_controller();
		IOSleep(b.active ? 1 : 8);
	}
}

void
hakios_w2bt_start(void)
{
	thread_t thread;

	if (!PE_parse_boot_argn("w2bt", &b.step, sizeof(b.step)) || b.step == 0) {
		return;
	}
	PE_parse_boot_argn("w2bt_stop", &b.stop_at, sizeof(b.stop_at));
	if (kernel_thread_start(bt_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_set_thread_name(thread, "w2 bt");
		thread_deallocate(thread);
	}
}

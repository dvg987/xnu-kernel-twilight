/*
 * The Wi-Fi of the Amlogic W2 on the box's internal USB (hakios.xhci.c has the bus and the chip's bring-up). The
 * firmware is a full MAC: it scans, associates, builds and parses 802.11 itself, and hands over Ethernet frames,
 * so what is here is its protocol (Amlogic's Linux driver, amlogic-drivers_w2, is the reference: tvos/docs/w2-*.md):
 *
 *   - messages: one at a time into the chip's memory and a doorbell rung; the answers and the indications read out of
 *     its memory when the "interrupt" (a vendor request on endpoint 0, polled here) says there are some;
 *   - frames received: a ring in the chip's memory, read from where we got to up to where the firmware is, whose
 *     records are a descriptor and a frame; space given back by telling the firmware where we got to;
 *   - frames sent: a page each (a descriptor and the frame without its Ethernet header), counted against what the
 *     firmware has room for and confirmed in a ring read from endpoint 5;
 *   - the station: scan, join (with the WPA2 handshake done here, wifi/lx/src/lx_wpa.c), leave, for the station
 *     boundary (sys/hakios.sta.h), and en0 on top (sys/hakios.net.h).
 *
 * Boot-arg w2=5: the firmware's start-up messages; w2=6: the interface and the station besides (the normal case);
 * w2=7: and a scan right away, for the log.
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/random.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <sys/kpi_mbuf.h>
#include <sys/hakios.net.h>
#include <sys/hakios.sta.h>
#include <sys/hakios.xhci.h>
#include <sys/hakios.input.h>
#include "../../../../wifi/lx/src/lx_wpa.h"

extern void IOSleep(unsigned int milliseconds);
extern void IODelay(unsigned int microseconds);

/* the chip's memory */
#define A2E_MSG         0x60000014u
#define E2A_MSG_1       0x60000210u
#define E2A_MSG_2       0x60012cc8u
#define E2A_MSG_SIZE    1036
#define E2A_PATTERN     0xaddede2au
#define DBG_BUF         0x6000072cu
#define IPC_TRIGGER     0x60800000u         /* doorbell: 2 a message, 0x10 messages read, 1 debug string read */
#define IPC_UNMASK      0x6080000cu
#define EXT_IRQ         0x00a10034u
#define RX_START        0x6001b400u
#define RX_END_EXPAND   0x600684b0u
#define RX_END_NARROW   0x60029130u
#define REO_SLOTS       256                 /* a receive block ack's frames at most */
#define PAGES_EXPAND    21                  /* TX pages with each layout */
#define PAGES_NARROW    77
/* the RX head's layout requests and our answers to them, with the tail we confirm (aml_sdio_usb_rx.c) */
#define FW_NARROW       (1u << 20)
#define FW_EXPAND       (1u << 21)
#define EXPAND_READ     (1u << 25)
#define EXPAND_DONE     (1u << 26)
#define NARROW_READ     (1u << 27)
#define NARROW_DONE     (1u << 28)
#define RX_FIRST_TAIL   0x60b081d0u
#define RX_CONFIRM      0x00a10038u
#define CHAN_SWITCH     0x00a17fc0u

#define ST_MSG_ACK      0x0004u
#define ST_RXDESC       0x0008u
#define ST_EXT          0x0020u
#define ST_TXCFM        0x0f80u
#define ST_DBG          0x1000u
#define ST_MSG          0x2000u
#define ST_CHAN         0x8000u

/* messages: (task << 10) | index */
#define MM_RESET_REQ            0x0000
#define MM_START_REQ            0x0002
#define MM_VERSION_REQ          0x0004
#define MM_ADD_IF_REQ           0x0006
#define MM_KEY_ADD_REQ          0x0024
#define MM_CHANNEL_SWITCH_IND   0x0044
#define MM_OTHER_REQ            0x0069
#define DBG_ERROR_IND           0x0408
#define SCANU_START_REQ         0x1000
#define SCANU_START_CFM         0x1001
#define SCANU_RESULT_IND        0x1004
#define ME_CONFIG_REQ           0x1400
#define ME_CHAN_CONFIG_REQ      0x1402
#define ME_SET_CONTROL_PORT_REQ 0x1404
#define SM_CONNECT_REQ          0x1800
#define SM_CONNECT_IND          0x1802
#define SM_DISCONNECT_REQ       0x1803
#define SM_DISCONNECT_IND       0x1805
#define SM_EXT_AUTH_IND         0x1806
#define PRIV_EFUSE_RESULT       0x400a
#define CFM(id)                 ((id) + 1)

#define SUB_EFUSE               11
#define SUB_EXT_CAPAB           35

#define N_BSS           48
#define N_SLOTS         128                 /* TX frames outstanding at most (the narrow layout has 77 pages) */
#define PAGE_MOST       4624

static LCK_GRP_DECLARE(w2_lck_grp, "hakios-w2");
static LCK_MTX_DECLARE(w2_lock, &w2_lck_grp);

static struct {
	unsigned step;
	/* the one message under way */
	bool outstanding, cfm_done;
	uint16_t cfm_id, cfm_length;
	uint8_t cfm[512];
	unsigned messages_in;
	/* the interface */
	uint8_t mac[6], vif;
	bool up;
	/* frames in */
	uint8_t *ring;                      /* the ring as the chip has it, at the same offsets */
	uint32_t tail, end;                 /* chip addresses, BIT31 the phase */
	bool tail_known;
	unsigned rx_frames, rx_dropped;
	/* receive reordering, by TID */
	struct reo {
		bool on;
		unsigned ssn, size, held;
		struct reo_slot {
			uint8_t *data;
			uint16_t length, flags;
			uint64_t deadline;
		} slot[REO_SLOTS];
	} reo[8];
	unsigned reo_waited, reo_timeouts, reo_stale;
	uint64_t rx_bytes;
	unsigned rx_batches, polls;
	/* frames out */
	int pages;
	uint32_t slot_id[N_SLOTS], prefix;
	bool slot_busy[N_SLOTS];
	unsigned cfm_index;
	unsigned tx_frames, tx_dropped, tx_batches;
	bool tx_stopped, stop_pending;       /* stop asked for; told the firmware once all pages are back */
	int pages_total;
	uint32_t layout_flags, confirmed;   /* our answer to a layout change; the last confirmation written */
	/* the station */
	struct hakios_sta_bss *bss;         /* N_BSS */
	uint64_t bss_heard[N_BSS];
	unsigned bss_count;
	bool scanning, scan_done;
	enum hakios_sta_phase phase;
	bool connect_done, joined;
	uint16_t connect_status;
	uint8_t ap_idx, ch_idx, qos, bssid[6];
	struct hakios_sta_bss target;
	int last_error;
	struct lx_wpa wpa;
	volatile int handshake;
	bool group_pending;                 /* a new group key from the handshake, for the worker to install */
	unsigned eapol;
	uint8_t rsn[22];
} w;

/* the poll thread's pause between looks at the chip: when the last look found nothing (ms), and after one that
 * moved frames (us, 0: straight on); kern.hakios_w2_stats=1 logs the traffic every 2 s */
static unsigned idle_ms = 1, busy_us = 0, stats;
SYSCTL_UINT(_kern, OID_AUTO, hakios_w2_idle_ms, CTLFLAG_RW | CTLFLAG_LOCKED, &idle_ms, 0, "");
SYSCTL_UINT(_kern, OID_AUTO, hakios_w2_busy_us, CTLFLAG_RW | CTLFLAG_LOCKED, &busy_us, 0, "");
SYSCTL_UINT(_kern, OID_AUTO, hakios_w2_stats, CTLFLAG_RW | CTLFLAG_LOCKED, &stats, 0, "");

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

/* with w2_lock held: wait on `event' up to `ms'; false on timeout */
static bool
sleep_ms(void *event, unsigned ms)
{
	uint64_t deadline;

	clock_interval_to_deadline(ms, NSEC_PER_MSEC, &deadline);
	return lck_mtx_sleep_deadline(&w2_lock, LCK_SLEEP_DEFAULT, (event_t)event, THREAD_UNINT, deadline) != THREAD_TIMED_OUT;
}

/* ---- messages ---- */

/*
 * A message to the firmware; with `cfm_id', waits for that answer and copies it to `cfm'. Never from the poll thread
 * (it is the one that hears the answers). 0, or an errno.
 */
static int
send_msg(uint16_t id, const void *params, uint16_t length, uint16_t cfm_id, void *cfm, uint16_t cfm_length)
{
	uint8_t buf[512];
	uint16_t padded = (uint16_t)((length + 3) & ~3u);
	uint64_t began = now_ms();
	bool ok;

	if (8 + padded > 508) {
		return EINVAL;
	}
	bzero(buf, sizeof(buf));
	put16(buf, id);
	put16(buf + 2, id >> 10);
	put16(buf + 4, 100);
	put16(buf + 6, padded);
	if (length != 0) {
		memcpy(buf + 8, params, length);
	}
	lck_mtx_lock(&w2_lock);
	while (w.outstanding && now_ms() - began < 6000) {
		sleep_ms(&w.outstanding, 100);
	}
	if (w.outstanding) {
		printf("w2: message %#x: the one before was never taken; sending anyway\n", id);
	}
	w.outstanding = true;
	w.cfm_id = cfm_id;
	w.cfm_done = false;
	lck_mtx_unlock(&w2_lock);

	ok = hakios_w2_sram_write(A2E_MSG, buf, 8 + padded) && hakios_w2_write(IPC_TRIGGER, 0x2);
	if (!ok) {
		lck_mtx_lock(&w2_lock);
		w.outstanding = false;
		lck_mtx_unlock(&w2_lock);
		printf("w2: message %#x: the bus failed\n", id);
		return EIO;
	}
	if (cfm_id == 0) {
		return 0;
	}
	lck_mtx_lock(&w2_lock);
	while (!w.cfm_done && now_ms() - began < 6000) {
		sleep_ms(&w.cfm_done, 100);
	}
	ok = w.cfm_done;
	if (ok && cfm != NULL) {
		memcpy(cfm, w.cfm, cfm_length < w.cfm_length ? cfm_length : w.cfm_length);
	}
	w.cfm_id = 0;
	lck_mtx_unlock(&w2_lock);
	if (!ok) {
		printf("w2: message %#x: no answer %#x in 6 s\n", id, cfm_id);
		return ETIMEDOUT;
	}
	return 0;
}

/* an Amlogic sub-command (MM_OTHER_REQ): { u32 index; payload } */
static int
send_other(uint32_t sub, const void *payload, uint16_t length, uint16_t cfm_id, void *cfm, uint16_t cfm_length)
{
	uint8_t p[500];

	put32(p, sub);
	memcpy(p + 4, payload, length);
	return send_msg(MM_OTHER_REQ, p, (uint16_t)(4 + length), cfm_id, cfm, cfm_length);
}

/* ---- the station's frames ---- */

static bool transmit_frame(const uint8_t *frame, size_t length, bool special);

/* a message of the WPA2 handshake from the access point (lx_wpa.c answers it) */
static void
handshake_message(const uint8_t *frame, unsigned length)
{
	uint8_t reply[14 + LX_WPA_REPLY_MOST];
	unsigned reply_length = 0;
	const char *why = "";
	int said;

	w.eapol++;
	if (length < 14 || memcmp(frame + 6, w.bssid, 6) != 0) {
		return;
	}
	said = lx_wpa_message(&w.wpa, frame + 14, length - 14, reply + 14, &reply_length, &why);
	if (reply_length != 0) {
		memcpy(reply, w.bssid, 6);
		memcpy(reply + 6, w.mac, 6);
		reply[12] = 0x88;
		reply[13] = 0x8e;
		transmit_frame(reply, 14 + reply_length, true);
	}
	printf("w2: the handshake: a message of %u bytes: %s%s\n", length - 14, said == LX_WPA_ANSWER ? "the first; answered" :
	    said == LX_WPA_KEYS ? "the third, signed with our key too; answered" : said == LX_WPA_GROUP ?
	    "a new group key; answered" : said == LX_WPA_BAD ? "NOT GOOD: " : "not one to answer", why);
	if (said == LX_WPA_GROUP && w.joined) {
		lck_mtx_lock(&w2_lock);
		w.group_pending = true;
		thread_wakeup((event_t)&w.group_pending);
		lck_mtx_unlock(&w2_lock);
	}
	if (said != LX_WPA_NOTHING) {
		lck_mtx_lock(&w2_lock);
		if (w.handshake != LX_WPA_KEYS) {
			w.handshake = said;
		}
		thread_wakeup((event_t)&w.handshake);
		lck_mtx_unlock(&w2_lock);
	}
}

/* an Ethernet frame the radio has received */
static void
ethernet_received(const uint8_t *frame, unsigned length)
{
	if (length < 14) {
		return;
	}
	if (frame[12] == 0x88 && frame[13] == 0x8e) {
		handshake_message(frame, length);
	} else if (w.joined && length <= HAKIOS_NET_FRAME_MAX) {
		w.rx_frames++;
		hakios_net_input(frame, length);
	}
}

/* a beacon or a probe response heard while scanning: into the list (one entry for each access point) */
static void
bss_heard(const uint8_t *f, unsigned length, unsigned freq, int rssi)
{
	const uint8_t *ies = f + 36;
	unsigned n = length - 36, i, slot;
	struct hakios_sta_bss *b;

	if (length < 36) {
		return;
	}
	/* the channel the beacon says it is on (a beacon heard on a neighbouring channel says the right one) */
	for (i = 0; i + 2 <= n && i + 2 + ies[i + 1] <= n; i += 2 + ies[i + 1]) {
		if (ies[i] == 3 && ies[i + 1] == 1) {
			unsigned ch = ies[i + 2];

			freq = ch == 14 ? 2484 : ch < 14 ? 2407 + 5 * ch : 5000 + 5 * ch;
		}
	}
	lck_mtx_lock(&w2_lock);
	for (slot = 0; slot < w.bss_count && memcmp(w.bss[slot].bssid, f + 16, 6) != 0; slot++) {
	}
	if (slot == w.bss_count) {
		if (w.bss_count == N_BSS) {
			lck_mtx_unlock(&w2_lock);
			return;
		}
		w.bss_count++;
	}
	b = &w.bss[slot];
	bzero(b, sizeof(*b));
	memcpy(b->bssid, f + 16, 6);
	b->frequency = (uint16_t)freq;
	b->rssi = (int16_t)rssi;
	b->beacon_interval = le16(f + 32);
	b->capability = le16(f + 34);
	b->elements_length = (uint16_t)(n < HAKIOS_STA_ELEMENTS_MOST ? n : HAKIOS_STA_ELEMENTS_MOST);
	memcpy(b->elements, ies, b->elements_length);
	for (i = 0; i + 2 <= n && i + 2 + ies[i + 1] <= n; i += 2 + ies[i + 1]) {
		if (ies[i] == 0 && ies[i + 1] <= 32) {
			b->ssid_length = ies[i + 1];
			memcpy(b->ssid, ies + i + 2, ies[i + 1]);
			break;
		}
	}
	w.bss_heard[slot] = now_ms();
	lck_mtx_unlock(&w2_lock);
}

static void mpdu_received(const uint8_t *frame, unsigned frmlen, unsigned flags);

/*
 * Receive reordering (aml_reorder.c, simpler): a session for each TID the firmware has agreed a receive block ack for
 * (MM_BA_ADD_REQ forwarded); frames from the window's start on go up in order, the others wait in their sequence
 * number's place until the gap before them fills, the window moves past them (a later frame, a BlockAckReq), or
 * REO_WAIT_MS passes.
 */
#define REO_WAIT_MS     100

static inline bool sn_less(unsigned a, unsigned b) { return ((a - b) & 0xfff) > 2048; }

static void
reo_release(unsigned tid, unsigned index)
{
	struct reo_slot *e = &w.reo[tid].slot[index];

	mpdu_received(e->data, e->length, e->flags);
	kfree_data(e->data, e->length);
	e->data = NULL;
	w.reo[tid].held--;
}

/* the window's start to `ssn' (the frames before it go up, holes are given up on), then on past what is there */
static void
reo_shift(unsigned tid, unsigned ssn)
{
	struct reo *r = &w.reo[tid];
	unsigned sn = r->ssn;

	for (;; sn = (sn + 1) & 0xfff) {
		unsigned index = sn % REO_SLOTS;

		if (r->slot[index].data != NULL) {
			reo_release(tid, index);
		} else if (!sn_less(sn, ssn)) {
			break;
		}
	}
	r->ssn = sn;
}

static void
reo_add(unsigned tid, unsigned sn, const uint8_t *frame, unsigned length, unsigned flags)
{
	struct reo *r = &w.reo[tid];
	unsigned index = sn % REO_SLOTS;

	if (sn_less(sn, r->ssn)) {
		w.reo_stale++;                          /* before the window: a duplicate, or given up on already */
		return;
	}
	if (!sn_less(sn, (r->ssn + r->size) & 0xfff)) {
		reo_shift(tid, (sn - r->size + 1) & 0xfff);
	}
	if (r->slot[index].data != NULL) {
		return;                                 /* a duplicate */
	}
	if (sn == r->ssn) {
		mpdu_received(frame, length, flags);
		reo_shift(tid, (sn + 1) & 0xfff);
		return;
	}
	r->slot[index].data = kalloc_data(length, Z_NOWAIT);
	if (r->slot[index].data == NULL) {
		mpdu_received(frame, length, flags);
		return;
	}
	memcpy(r->slot[index].data, frame, length);
	r->slot[index].length = (uint16_t)length;
	r->slot[index].flags = (uint16_t)flags;
	r->slot[index].deadline = now_ms() + REO_WAIT_MS;
	r->held++;
	w.reo_waited++;
}

/* a BlockAckReq: the sender will not send what is before `ssn' again */
static void
reo_bar(unsigned tid, unsigned ssn)
{
	if (w.reo[tid].on && sn_less(w.reo[tid].ssn, ssn)) {
		reo_shift(tid, ssn);
	}
}

/* frames waited for too long: the holes before the last of them are given up on */
static void
reo_age(void)
{
	uint64_t now = now_ms();

	for (unsigned tid = 0; tid < 8; tid++) {
		struct reo *r = &w.reo[tid];
		int last = -1;

		if (!r->on || r->held == 0) {
			continue;
		}
		for (unsigned i = 0; i < r->size; i++) {
			unsigned sn = (r->ssn + i) & 0xfff;
			struct reo_slot *e = &r->slot[sn % REO_SLOTS];

			if (e->data != NULL && e->deadline <= now) {
				last = (int)sn;
			}
		}
		if (last >= 0) {
			w.reo_timeouts++;
			reo_shift(tid, ((unsigned)last + 1) & 0xfff);
		}
	}
}

static void
reo_start(unsigned tid, unsigned size, unsigned ssn)
{
	struct reo *r = &w.reo[tid & 7];

	if (r->on) {
		reo_shift(tid & 7, (r->ssn + REO_SLOTS) & 0xfff);   /* (all of the old session's frames up) */
	}
	r->on = true;
	r->size = size == 0 || size > REO_SLOTS ? REO_SLOTS : size;
	r->ssn = ssn & 0xfff;
	printf("w2: receive block ack: tid %u, %u frames, from %u\n", tid & 7, r->size, r->ssn);
}

static void
reo_stop(unsigned tid)
{
	struct reo *r = &w.reo[tid & 7];

	if (r->on) {
		reo_shift(tid & 7, (r->ssn + REO_SLOTS) & 0xfff);
		r->on = false;
	}
}

/* one record of the RX ring: a descriptor and a frame */
static void
rx_record(const uint8_t *d, unsigned length)
{
	unsigned frmlen, payl, status, flags, p;
	const uint8_t *frame;

	if (length < 128) {
		return;
	}
	frmlen = le16(d + 48);
	payl = d[52];
	status = d[53];
	flags = le32(d + 96);
	if (status == 0 || frmlen == 0) {
		return;
	}
	if ((status & (0x10 | 0x20 | 0x80 | 0x04)) && status != 0x06 && status != 0x0a) {
		w.rx_dropped++;
		return;
	}
	p = 128 + 36 + payl;
	if (p + frmlen > length) {
		w.rx_dropped++;
		return;
	}
	frame = d + p;
	if (flags & 2) {                                        /* an 802.11 frame: management (or a block ack request) */
		unsigned subtype = frame[0] & 0xfc;

		if (w.scanning && (subtype == 0x80 || subtype == 0x50)) {
			bss_heard(frame, frmlen, le16(d + 90), (int8_t)d[65]);
		}
		if (frame[0] == 0x84 && frmlen >= 20) {        /* BlockAckReq: the sender moved its window on */
			reo_bar((le16(frame + 16) >> 12) & 7, le16(frame + 18) >> 4);
		}
		return;
	}
	/* under a receive block ack session the frames come in the order they were received in, not sent in: the host
	 * puts them back in order (aml_reorder.c); the sequence number in the extension before the frame */
	if (status == 0x06 && payl >= 4) {
		const uint8_t *ext = d + 128 + 36;
		unsigned tid = (flags >> 4) & 7;

		if ((le16(ext + 2) & (1u << 14)) && w.reo[tid].on) {
			reo_add(tid, le16(ext) & 0xfff, frame, frmlen, flags);
			return;
		}
	}
	mpdu_received(frame, frmlen, flags);
}

/* a frame (an Ethernet II frame, or an A-MSDU's subframes) to the stack */
static void
mpdu_received(const uint8_t *frame, unsigned frmlen, unsigned flags)
{
	if (flags & 1) {                                        /* A-MSDU: subframes {DA, SA, length, LLC/SNAP + body} */
		uint8_t eth[HAKIOS_NET_FRAME_MAX];

		for (unsigned o = 0; o + 14 <= frmlen;) {
			unsigned sub = (unsigned)(frame[o + 12] << 8 | frame[o + 13]);

			if (o + 14 + sub > frmlen || sub < 8 || sub - 8 + 14 > sizeof(eth)) {
				break;
			}
			memcpy(eth, frame + o, 12);
			memcpy(eth + 12, frame + o + 14 + 6, 2);        /* the type out of the SNAP header */
			memcpy(eth + 14, frame + o + 14 + 8, sub - 8);
			ethernet_received(eth, 14 + sub - 8);
			o += (14 + sub + 3) & ~3u;
		}
		return;
	}
	ethernet_received(frame, frmlen);
}

static uint8_t *
ring_at(uint32_t addr)
{
	return w.ring + ((addr & 0x7fffffffu) - RX_START);
}

/*
 * Tells the firmware where we got to, with our part of a change of the buffers' layout (aml_sdio_usb_rx_confirm):
 * the firmware stops RX and sets NARROW (more TX pages, a smaller ring) or EXPAND (the other way) in the head; once the
 * ring is drained the new layout applies here and READ answers; when the firmware clears its flag, DONE.
 */
static void
rx_confirm(uint32_t state, uint32_t head)
{
	uint32_t flags = 0, value;
	uint8_t c[8];

	switch (state | w.layout_flags) {
	case FW_NARROW:
		/* drained: the firmware's pointer back at the start if the smaller ring no longer holds it */
		if ((head & 0x7fffffffu) >= RX_END_NARROW) {
			w.tail = RX_START;
		}
		w.end = RX_END_NARROW;
		flags = NARROW_READ;
		break;
	case FW_EXPAND:
		w.end = RX_END_EXPAND;
		flags = EXPAND_READ;
		break;
	case FW_NARROW | NARROW_READ:
	case FW_EXPAND | EXPAND_READ:
		flags = w.layout_flags;                         /* (the firmware has not cleared its flag yet) */
		break;
	case NARROW_READ:
		flags = NARROW_DONE;
		break;
	case EXPAND_READ:
		flags = EXPAND_DONE;
		break;
	case 0:
		break;
	default:
		printf("w2: rx ring: layout state %#x with our flags %#x\n", state, w.layout_flags);
		break;
	}
	if (flags == NARROW_READ || flags == EXPAND_READ) {
		/* the TX pages of the new layout: TX is stopped and every page is back. (The head can still carry the
		 * request after the change is done: answered again, but the pages are not counted again.) */
		int total = flags == NARROW_READ ? PAGES_NARROW : PAGES_EXPAND;

		lck_mtx_lock(&w2_lock);
		if (w.pages_total != total) {
			w.pages += total - w.pages_total;
			w.pages_total = total;
			printf("w2: buffers: %s RX ring, %d TX pages\n", flags == NARROW_READ ? "the small" : "the large", total);
		}
		lck_mtx_unlock(&w2_lock);
	}
	w.layout_flags = (flags == NARROW_DONE || flags == EXPAND_DONE) ? 0 : flags;
	value = flags | w.tail;
	if (value == w.confirmed) {
		return;
	}
	w.confirmed = value;
	put32(c, 1);
	put32(c + 4, value);
	hakios_w2_sram_write(RX_CONFIRM, c, 8);
}

/* the ring from the tail up to the head the firmware gave; the records taken */
static unsigned
rx_process(uint32_t r)
{
	uint32_t head, cur, a, b, state;
	unsigned records = 0;

	if (r == 0) {
		return 0;
	}
	head = 0x60000000u | (r & ~(7u << 19) & 0x0fffffffu);
	if (r & (1u << 19)) {
		head |= 1u << 31;
	}
	state = r & (FW_NARROW | FW_EXPAND);
	if (!w.tail_known) {
		uint8_t t[4];

		if (!hakios_w2_sram_read(RX_FIRST_TAIL, t, 4)) {
			return 0;
		}
		w.tail = le32(t);
		w.tail_known = true;
		printf("w2: rx ring: tail %#x, head %#x\n", w.tail, head);
	}
	if (w.tail == head) {
		if (state != 0 || w.layout_flags != 0) {
			rx_confirm(state, head);
		}
		return 0;
	}
	/* read what is new into the copy of the ring */
	a = w.tail & 0x7fffffffu;
	b = head & 0x7fffffffu;
	if (a < RX_START || a >= w.end || b < RX_START || b >= w.end) {
		printf("w2: rx ring: tail %#x or head %#x outside it; starting again at the head\n", w.tail, head);
		w.tail = head;
		return 0;
	}
	if ((w.tail >> 31) == (head >> 31) && a < b) {
		if (!hakios_w2_sram_read(a, ring_at(a), b - a)) {
			return 0;
		}
	} else {
		if (!hakios_w2_sram_read(a, ring_at(a), w.end - a) || (b > RX_START &&
		    !hakios_w2_sram_read(RX_START, ring_at(RX_START), b - RX_START))) {
			return 0;
		}
	}
	/* and walk its records */
	for (cur = w.tail; (cur & 0x7fffffffu) != b || (cur >> 31) != (head >> 31);) {
		uint32_t at = cur & 0x7fffffffu, next, phase = cur & (1u << 31);

		/* no record begins in the ring's last 128 bytes: the next one is at its start */
		if (at + 128 > w.end) {
			cur = RX_START | (phase ^ (1u << 31));
			continue;
		}
		next = le32(ring_at(at) + 76);

		if (next < RX_START || next >= w.end) {
			printf("w2: rx ring: a record at %#x points at %#x; skipping to the head\n", at, next);
			break;
		}
		if (next > at) {
			rx_record(ring_at(at), next - at);
		} else if (next == RX_START) {
			rx_record(ring_at(at), w.end - at);
			phase ^= 1u << 31;
		} else {
			/* the record wraps: its start here, its rest at the start of the ring */
			static uint8_t joined_up[16384];
			unsigned len = w.end - at, first, skip = 0, rest;
			unsigned flags = le32(ring_at(at) + 96);

			if (len < 164) {
				first = 128;
			} else if ((flags & 2) || len < 220) {
				first = 164;
			} else {
				first = len;
				skip = 36;
			}
			rest = next - (RX_START + skip);
			if (first + rest <= sizeof(joined_up)) {
				memcpy(joined_up, ring_at(at), first);
				memcpy(joined_up + first, ring_at(RX_START + skip), rest);
				rx_record(joined_up, first + rest);
			}
			phase ^= 1u << 31;
		}
		cur = next | phase;
		records++;
	}
	w.tail = head;
	rx_confirm(state, head);
	w.rx_bytes += b > a ? b - a : (w.end - a) + (b - RX_START);
	w.rx_batches++;
	return records;
}

/* ---- frames out ---- */

/*
 * Frames to send go into a batch (a page each: txdesc_host, then the frame without its Ethernet header) that the TX
 * thread sends with one packet command, up to 25 pages and 64 KiB: one exchange on the bus for many frames (a TCP
 * stream's acknowledgements, a stream sent). Two batches: one filled while the other goes out.
 */
#define BATCH_PAGES     25
#define BATCH_BYTES     0x10000

struct batch {
	uint8_t *data;
	uint32_t used;
	unsigned count;
	uint16_t sizes[BATCH_PAGES];
	uint16_t slots[BATCH_PAGES];
};
static struct batch batches[2];
static unsigned filling;                /* the batch frames go into (w2_lock) */

/* `special': a frame of the handshake (or one the firmware should send as soon as it can) */
static bool
transmit_frame(const uint8_t *frame, size_t length, bool special)
{
	unsigned slot, tid, payload = (unsigned)length - 14, size = (72 + payload + 3) & ~3u;
	uint16_t type;
	uint64_t began = now_ms();
	struct batch *b;
	uint8_t *pg;
	static const uint8_t ac_of_tid[8] = { 1, 0, 0, 1, 2, 2, 3, 3 };

	if (length < 14 || size > PAGE_MOST || batches[0].data == NULL) {
		return false;
	}
	type = (uint16_t)(frame[12] << 8 | frame[13]);
	lck_mtx_lock(&w2_lock);
	for (;;) {
		for (slot = 0; slot < N_SLOTS && w.slot_busy[slot]; slot++) {
		}
		b = &batches[filling];
		if (slot < N_SLOTS && w.pages > 1 && !w.tx_stopped && b->count < BATCH_PAGES && b->used + size <= BATCH_BYTES) {
			break;
		}
		if (now_ms() - began > 100) {
			w.tx_dropped++;
			lck_mtx_unlock(&w2_lock);
			return false;
		}
		sleep_ms(&w.pages, 10);
	}
	w.slot_busy[slot] = true;
	w.pages--;
	pg = b->data + b->used;
	bzero(pg, 72);
	put32(pg + 0, w.slot_id[slot]);
	put32(pg + 4, 1);                                       /* packet_addr[0]: 1 on USB */
	put16(pg + 28, (uint16_t)payload);                      /* packet_len[0] */
	pg[40] = 1;                                             /* packet_cnt */
	memcpy(pg + 42, frame, 6);
	memcpy(pg + 48, frame + 6, 6);
	pg[54] = frame[12];                                     /* the type as it is: network order */
	pg[55] = frame[13];
	tid = w.qos ? (type == 0x888e ? 7 : 0) : 0xff;
	put16(pg + 56, special ? 0x400 : 0);
	pg[60] = (uint8_t)tid;
	pg[61] = w.vif;
	pg[62] = w.ap_idx;
	put32(pg + 64, tid == 0xff ? 1 : ac_of_tid[tid]);
	put32(pg + 68, 0xffffffffu);
	memcpy(pg + 72, frame + 14, payload);
	bzero(pg + 72 + payload, size - 72 - payload);
	b->sizes[b->count] = (uint16_t)size;
	b->slots[b->count] = (uint16_t)slot;
	b->count++;
	b->used += size;
	thread_wakeup((event_t)&filling);
	lck_mtx_unlock(&w2_lock);
	return true;
}

/* the TX thread: each batch out as it fills, the other filling meanwhile */
static void
tx_thread(__unused void *arg, __unused wait_result_t wr)
{
	lck_mtx_lock(&w2_lock);
	for (;;) {
		struct batch *b = &batches[filling];
		bool ok;

		if (b->count == 0) {
			lck_mtx_sleep(&w2_lock, LCK_SLEEP_DEFAULT, (event_t)&filling, THREAD_UNINT);
			continue;
		}
		filling ^= 1;
		lck_mtx_unlock(&w2_lock);
		ok = hakios_w2_packets(b->data, b->sizes, b->count);
		lck_mtx_lock(&w2_lock);
		if (ok) {
			w.tx_frames += b->count;
			w.tx_batches++;
		} else {
			for (unsigned i = 0; i < b->count; i++) {
				w.slot_busy[b->slots[i]] = false;
				w.pages++;
			}
			w.tx_dropped += b->count;
		}
		b->count = 0;
		b->used = 0;
		thread_wakeup((event_t)&w.pages);
	}
}

/* en0's frames to send */
static bool
link_transmit(mbuf_t packet, size_t length)
{
	uint8_t frame[HAKIOS_NET_FRAME_MAX];

	if (!w.joined || length > sizeof(frame)) {
		return false;
	}
	return mbuf_copydata(packet, 0, length, frame) == 0 && transmit_frame(frame, length, false);
}

/* the ring of confirmations: each one of ours gives a page back */
static void
tx_confirmations(void)
{
	static uint8_t ring[2048];
	unsigned freed = 0;

	if (!hakios_w2_confirmations(ring, sizeof(ring))) {
		return;
	}
	lck_mtx_lock(&w2_lock);
	for (unsigned n = 0; n < 256; n++) {
		const uint8_t *c = ring + 8 * w.cfm_index;
		uint32_t hostid = le32(c + 4) >> 6;
		unsigned slot = (hostid & 0xffff) - 1;

		if (slot >= N_SLOTS || !w.slot_busy[slot] || w.slot_id[slot] != hostid) {
			break;
		}
		w.slot_busy[slot] = false;
		w.slot_id[slot] = ((++w.prefix << 16) | (hostid & 0xffff)) & 0x3ffffff;
		w.pages++;
		freed++;
		w.cfm_index = (w.cfm_index + 1) % 256;
	}
	if (freed != 0) {
		thread_wakeup((event_t)&w.pages);
	}
	lck_mtx_unlock(&w2_lock);
}

/* ---- what the firmware says ---- */

static void
message_received(const uint8_t *m)
{
	uint16_t id = le16(m), length = le16(m + 6);
	const uint8_t *p = m + 8;

	if (length > 1024) {
		return;
	}
	w.messages_in++;
	lck_mtx_lock(&w2_lock);
	if (w.cfm_id != 0 && id == w.cfm_id && !w.cfm_done) {
		memcpy(w.cfm, p, length < sizeof(w.cfm) ? length : sizeof(w.cfm));
		w.cfm_length = length;
		w.cfm_done = true;
		w.outstanding = false;
		thread_wakeup((event_t)&w.cfm_done);
		thread_wakeup((event_t)&w.outstanding);
		lck_mtx_unlock(&w2_lock);
		return;
	}
	lck_mtx_unlock(&w2_lock);
	switch (id) {
	case SCANU_START_CFM:
		lck_mtx_lock(&w2_lock);
		w.scan_done = true;
		thread_wakeup((event_t)&w.scan_done);
		lck_mtx_unlock(&w2_lock);
		break;
	case SCANU_RESULT_IND:
		if (length >= 16 + 36 && 16 + le16(p) <= length) {
			bss_heard(p + 16, le16(p), le16(p + 4), (int8_t)p[9]);
		}
		break;
	case SM_CONNECT_IND:
		lck_mtx_lock(&w2_lock);
		w.connect_status = le16(p);
		memcpy(w.bssid, p + 2, 6);
		w.ap_idx = p[10];
		w.ch_idx = p[11];
		w.qos = p[12];
		w.connect_done = true;
		thread_wakeup((event_t)&w.connect_done);
		lck_mtx_unlock(&w2_lock);
		printf("w2: connect indication: status %u, station %u, channel context %u, qos %u, aid %u\n", le16(p), p[10], p[11],
		    p[12], le16(p + 18) & 0x3fff);
		break;
	case SM_DISCONNECT_IND:
		for (unsigned tid = 0; tid < 8; tid++) {
			reo_stop(tid);
		}
		printf("w2: disconnected (reason %u%s%s)\n", le16(p) & 0x7fff, (le16(p) & 0x8000) ? ", asked by us" : "",
		    p[3] ? ", reassociating" : "");
		if (!p[3] && w.joined) {
			w.joined = false;
			w.phase = HAKIOS_STA_IDLE;
			hakios_net_link_state(false);
			if (!(le16(p) & 0x8000)) {
				hakios_sta_lost(ENETDOWN);
			}
		}
		lck_mtx_lock(&w2_lock);
		w.connect_done = true;                          /* (a join waiting for its indication ends) */
		w.connect_status = w.connect_status ? w.connect_status : 0xffff;
		thread_wakeup((event_t)&w.connect_done);
		lck_mtx_unlock(&w2_lock);
		break;
	case SM_EXT_AUTH_IND:
		printf("w2: the firmware asks for SAE (WPA3): not done here\n");
		break;
	case DBG_ERROR_IND:
		printf("w2: the firmware reports an error\n");
		break;
	case 0x0028:                                            /* MM_BA_ADD_REQ, forwarded: type, sta, tid, size, ssn */
		if (length >= 6 && p[0] == 1) {
			reo_start(p[2], p[3], le16(p + 4));
		}
		break;
	case 0x002a:                                            /* MM_BA_DEL_REQ: type, sta, tid */
		if (length >= 3 && p[0] == 1) {
			reo_stop(p[2]);
		}
		break;
	case MM_CHANNEL_SWITCH_IND:
	case 0x0029: case 0x002b:
	case 0x004f:                                            /* a channel's survey, during a scan */
	case 0x4019:
		break;
	default:
		if (w.messages_in < 200) {
			printf("w2: message %#x from the firmware (%u bytes)\n", id, length);
		}
		break;
	}
}

static void
messages(void)
{
	static uint8_t m1[E2A_MSG_SIZE], m2[E2A_MSG_SIZE];

	if (!hakios_w2_sram_read(E2A_MSG_1, m1, E2A_MSG_SIZE) || !hakios_w2_sram_read(E2A_MSG_2, m2, E2A_MSG_SIZE)) {
		return;
	}
	hakios_w2_write(IPC_TRIGGER, 0x10);
	if (le32(m1 + 1032) == E2A_PATTERN && le16(m1) != 0) {
		message_received(m1);
	}
	if (le32(m2 + 1032) == E2A_PATTERN && le16(m2) != 0) {
		message_received(m2);
	}
}

static void
debug_string(void)
{
	static uint8_t d[260];

	if (hakios_w2_sram_read(DBG_BUF, d, sizeof(d))) {
		d[255] = 0;
		for (int i = 0; i < 256 && d[i]; i++) {
			if (d[i] < 0x20 && d[i] != '\n') {
				d[i] = ' ';
			}
		}
		printf("w2: firmware: %s%s", (char *)d, strchr((char *)d, '\n') ? "" : "\n");
	}
	hakios_w2_write(IPC_TRIGGER, 0x1);
}

/* TX stopped at the firmware's request (to change the buffers' layout): tell it so once all our frames are sent */
static void
tx_stop_check(void)
{
	bool tell;
	uint8_t c[8];

	lck_mtx_lock(&w2_lock);
	tell = w.stop_pending && w.pages >= w.pages_total;
	if (tell) {
		w.stop_pending = false;
	}
	lck_mtx_unlock(&w2_lock);
	if (tell) {
		put32(c, 3);                            /* DYNAMIC_BUF_NOTIFY_FW_TX_STOP */
		put32(c + 4, 1);
		hakios_w2_sram_write(EXT_IRQ, c, 8);
	}
}

static void
extended(void)
{
	uint32_t v = 0;

	if (!hakios_w2_read(EXT_IRQ, &v)) {
		return;
	}
	lck_mtx_lock(&w2_lock);
	if (v == 1) {
		w.tx_stopped = true;
		w.stop_pending = true;                  /* told once every page is back: tx_stop_check */
	} else if (v == 2) {
		w.tx_stopped = false;
		thread_wakeup((event_t)&w.pages);
	}
	lck_mtx_unlock(&w2_lock);
	tx_stop_check();
	printf("w2: extended interrupt %u%s\n", v, v == 1 ? " (stop sending)" : v == 2 ? " (send again)" : v == 7 ?
	    " (the firmware crashed)" : "");
}

/* ---- the firmware's start, the interface ---- */

static int
efuse(uint32_t addr, uint32_t *value)
{
	uint8_t a[4], r[8];
	int error;

	put32(a, addr);
	error = send_other(SUB_EFUSE, a, 4, PRIV_EFUSE_RESULT, r, sizeof(r));
	*value = le32(r + 4);
	return error;
}

static void
chan_def(uint8_t *c, unsigned freq, unsigned band, unsigned flags)
{
	put16(c, (uint16_t)freq);
	c[2] = (uint8_t)band;
	c[3] = (uint8_t)flags;
	c[4] = 20;
	c[5] = 0;
}

/* the channels: 2.4 GHz 1-14, 5 GHz 36-177; 52-64 and 100-144 with radar; 169-177 not in the world's domain */
static unsigned
channel_list(uint8_t c2[14 * 6], uint8_t c5[28 * 6])
{
	unsigned n5 = 0;

	for (unsigned i = 0; i < 14; i++) {
		chan_def(c2 + 6 * i, i == 13 ? 2484 : 2412 + 5 * i, 0, 0);
	}
	for (unsigned f = 5180; f <= 5885; f += 20) {
		unsigned flags = (f >= 5260 && f <= 5320) || (f >= 5500 && f <= 5720) ? 4 : f >= 5845 ? 2 : 0;

		if (f > 5320 && f < 5500) {
			continue;
		}
		if (f > 5720 && f < 5745) {
			f = 5745;
			flags = 0;
		}
		chan_def(c5 + 6 * n5++, f, 1, flags);
	}
	return n5;
}

static int
firmware_start(void)
{
	uint8_t v[28], cfg[116], ext[10] = { 0x04, 0, 0x40, 0, 0, 0, 0, 0xc0, 0x01, 0 }, chans[254], st[80], add[12], r[4];
	uint32_t e7 = 0, lo = 0, hi = 0;
	int error;
	static const uint8_t mcs[16] = { 0xff, 0xff, 0, 0, 1, 0, 0, 0, 0, 0, 0x2c, 0x01, 0x01, 0, 0, 0 };
	static const uint8_t he_phy[11] = { 0x06, 0xe0, 0x2b, 0x78, 0x0d, 0xc0, 0xcf, 0x04, 0x82, 0x32, 0x00 };
	static const uint8_t ppe[7] = { 0x39, 0x1c, 0xc7, 0x71, 0x1c, 0x07, 0x00 };

	if ((error = send_msg(MM_RESET_REQ, NULL, 0, CFM(MM_RESET_REQ), NULL, 0)) != 0 ||
	    (error = send_msg(MM_VERSION_REQ, NULL, 0, CFM(MM_VERSION_REQ), v, sizeof(v))) != 0) {
		return error;
	}
	printf("w2: firmware %u.%u.%u.%u, MAC hw %#x/%#x, PHY %#x/%#x (%u streams, RF mode %u), features %#x, %u stations, "
	    "%u interfaces\n", v[3], v[2], v[1], v[0], le32(v + 4), le32(v + 8), le32(v + 12), le32(v + 16),
	    (le32(v + 12) >> 8) & 0xf, (le32(v + 12) >> 16) & 0xf, le32(v + 20), le16(v + 24), v[26]);

	/* what the radio can do: HT, VHT, HE as Amlogic's driver says it for this chip (2 streams, 80 MHz) */
	bzero(cfg, sizeof(cfg));
	put16(cfg + 0, 0x09e3);
	cfg[2] = 0x1f;
	memcpy(cfg + 3, mcs, sizeof(mcs));
	put32(cfg + 32, 0x038071b2);
	put16(cfg + 36, 0xfffa);
	put16(cfg + 38, 780);
	put16(cfg + 40, 0xfffa);
	put16(cfg + 42, 780);
	cfg[46] = 0x02;
	memcpy(cfg + 50, he_phy, sizeof(he_phy));
	put16(cfg + 62, 0xfffa);
	put16(cfg + 64, 0xfffa);
	put16(cfg + 66, 0xffff);
	put16(cfg + 68, 0xffff);
	put16(cfg + 70, 0xffff);
	put16(cfg + 72, 0xffff);
	memcpy(cfg + 74, ppe, sizeof(ppe));
	put16(cfg + 100, 1000);
	cfg[102] = 2;
	cfg[103] = cfg[104] = cfg[105] = 1;
	cfg[107] = 1;
	cfg[109] = 1;
	if ((error = send_msg(ME_CONFIG_REQ, cfg, sizeof(cfg), CFM(ME_CONFIG_REQ), NULL, 0)) != 0) {
		return error;
	}
	send_other(SUB_EXT_CAPAB, ext, sizeof(ext), 0, NULL, 0);
	bzero(chans, sizeof(chans));
	chans[253] = (uint8_t)channel_list(chans, chans + 84);
	chans[252] = 14;
	if ((error = send_msg(ME_CHAN_CONFIG_REQ, chans, sizeof(chans), CFM(ME_CHAN_CONFIG_REQ), NULL, 0)) != 0) {
		return error;
	}

	/* the radio's own address, from its fuses */
	if (efuse(0x7, &e7) == 0 && efuse((e7 >> 17) & 1 ? 0x11 : 0x1, &lo) == 0 && efuse((e7 >> 17) & 1 ? 0x12 : 0x2, &hi) == 0 &&
	    (lo | hi) != 0) {
		w.mac[0] = (uint8_t)(hi >> 8) & 0xfc;
		w.mac[1] = (uint8_t)hi;
		w.mac[2] = (uint8_t)(lo >> 24);
		w.mac[3] = (uint8_t)(lo >> 16);
		w.mac[4] = (uint8_t)(lo >> 8);
		w.mac[5] = (uint8_t)lo;
	} else {
		w.mac[0] = 0x1c;
		w.mac[1] = 0xa4;
		w.mac[2] = 0x10;
		read_random(w.mac + 3, 3);
		printf("w2: no address in the fuses: a random one\n");
	}

	bzero(st, sizeof(st));
	put32(st + 64, 300);
	put16(st + 68, 20);
	if ((error = send_msg(MM_START_REQ, st, sizeof(st), CFM(MM_START_REQ), NULL, 0)) != 0) {
		return error;
	}
	bzero(add, sizeof(add));
	memcpy(add + 2, w.mac, 6);
	if ((error = send_msg(MM_ADD_IF_REQ, add, sizeof(add), CFM(MM_ADD_IF_REQ), r, sizeof(r))) != 0) {
		return error;
	}
	if (r[0] != 0) {
		printf("w2: the station interface was refused (status %u)\n", r[0]);
		return EIO;
	}
	w.vif = r[1];
	w.up = true;
	printf("w2: the radio is up: a station interface (%u)\n", w.vif);
	return 0;
}

/* ---- the station (sys/hakios.sta.h) ---- */

static int
sta_power(bool on)
{
	(void)on;
	return 0;
}

static int
sta_scan(const struct hakios_sta_scan *what)
{
	uint8_t req[340], c2[14 * 6], c5[28 * 6];
	unsigned n5 = channel_list(c2, c5), n = 0;
	int error;

	bzero(req, sizeof(req));
	for (unsigned i = 0; i < 14 + n5 && n < 42; i++) {
		const uint8_t *c = i < 14 ? c2 + 6 * i : c5 + 6 * (i - 14);
		unsigned freq = le16(c);
		bool wanted = what == NULL || what->frequency_count == 0;

		for (unsigned k = 0; what != NULL && k < what->frequency_count; k++) {
			wanted |= what->frequencies[k] == freq;
		}
		if (!wanted || (c[3] & 2) || freq == 2484) {
			continue;
		}
		memcpy(req + 6 * n, c, 6);
		if (what != NULL && what->passive) {
			req[6 * n + 3] |= 1;
		}
		n++;
	}
	/* one name: the one asked for, or the empty one (any network) */
	if (what != NULL && what->ssid_length != 0 && what->ssid_length <= 32) {
		req[252] = what->ssid_length;
		memcpy(req + 253, what->ssid, what->ssid_length);
	}
	memset(req + 318, 0xff, 6);
	req[330] = w.vif;
	req[331] = (uint8_t)n;
	req[332] = 1;
	if (what != NULL && what->dwell_ms != 0) {
		put32(req + 336, what->dwell_ms * 1000u);
	}
	lck_mtx_lock(&w2_lock);
	w.scan_done = false;
	w.scanning = true;
	w.phase = HAKIOS_STA_SCANNING;
	lck_mtx_unlock(&w2_lock);
	error = send_msg(SCANU_START_REQ, req, sizeof(req), 0, NULL, 0);
	lck_mtx_lock(&w2_lock);
	for (uint64_t began = now_ms(); error == 0 && !w.scan_done && now_ms() - began < 15000;) {
		sleep_ms(&w.scan_done, 200);
	}
	if (error == 0 && !w.scan_done) {
		error = ETIMEDOUT;
	}
	w.scanning = false;
	w.phase = w.joined ? HAKIOS_STA_JOINED : HAKIOS_STA_IDLE;
	lck_mtx_unlock(&w2_lock);
	printf("w2: scan of %u channels: %s, %u access points known\n", n, error ? "FAILED" : "done", w.bss_count);
	return error;
}

static void
sta_heard(void (*each)(const struct hakios_sta_bss *bss, void *context), void *context)
{
	static struct hakios_sta_bss copy;
	uint64_t t = now_ms();

	for (unsigned i = 0;; i++) {
		lck_mtx_lock(&w2_lock);
		if (i >= w.bss_count) {
			lck_mtx_unlock(&w2_lock);
			break;
		}
		copy = w.bss[i];
		copy.age_ms = (uint32_t)(t - w.bss_heard[i]);
		lck_mtx_unlock(&w2_lock);
		if (copy.age_ms < 120000) {
			each(&copy, context);
		}
	}
}

static int
key_add(unsigned index, unsigned sta, const uint8_t *key, unsigned length, bool pairwise)
{
	uint8_t k[44], r[2];
	int error;

	bzero(k, sizeof(k));
	k[0] = (uint8_t)index;
	k[1] = (uint8_t)sta;
	k[4] = (uint8_t)length;
	memcpy(k + 8, key, length);
	k[40] = 2;                                              /* CCMP */
	k[41] = w.vif;
	k[43] = pairwise;
	error = send_msg(MM_KEY_ADD_REQ, k, sizeof(k), CFM(MM_KEY_ADD_REQ), r, sizeof(r));
	return error ? error : r[0] ? EIO : 0;
}

static int
sta_leave(void)
{
	uint8_t d[4] = { 3, 0, 0, 0 };

	if (!w.joined && w.phase != HAKIOS_STA_JOINING) {
		return 0;
	}
	d[2] = w.vif;
	w.joined = false;
	hakios_net_link_state(false);
	send_msg(SM_DISCONNECT_REQ, d, sizeof(d), CFM(SM_DISCONNECT_REQ), NULL, 0);
	w.phase = HAKIOS_STA_IDLE;
	return 0;
}

/*
 * Which access point of a network to join: the one heard best, with 5 GHz counted 20 dB better than it is. On 2.4 GHz
 * the radio shares its antenna with the Bluetooth of the same chip (the remote), and authentications there time out
 * (status 16) while 5 GHz ones go through; Apple's own devices prefer 5 GHz as well.
 */
static int
band_score(const struct hakios_sta_bss *b)
{
	return b->rssi + (b->frequency > 4000 ? 20 : 0);
}

static int
sta_join_once(const struct hakios_sta_join *what, uint8_t (*avoid)[6], unsigned avoid_count, bool rescan)
{
	uint8_t req[64 + 22], r[1], open[2];
	const struct hakios_sta_bss *t = NULL;
	bool wpa2 = what->security == HAKIOS_STA_WPA2_PSK;
	static const uint8_t rsn[22] = { 0x30, 0x14, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,
		                         0x01, 0x00, 0x00, 0x0f, 0xac, 0x02, 0x00, 0x00 };
	int error = 0;
	uint64_t began;
	bool any_bssid = true;

	for (int i = 0; i < 6; i++) {
		any_bssid &= what->bssid[i] == 0;
	}
	if (w.joined) {
		sta_leave();
	}
	/* the access point of that name heard best (or the one asked for); not heard yet: a scan for it first */
	for (int round = rescan ? 1 : 0; round < 2 && t == NULL; round++) {
		if (round == 1) {
			struct hakios_sta_scan named;

			bzero(&named, sizeof(named));
			memcpy(named.ssid, what->ssid, what->ssid_length);
			named.ssid_length = what->ssid_length;
			sta_scan(&named);
		}
		lck_mtx_lock(&w2_lock);
		for (unsigned i = 0; i < w.bss_count; i++) {
			const struct hakios_sta_bss *b = &w.bss[i];

			bool avoided = false;

			for (unsigned k = 0; k < avoid_count; k++) {
				avoided |= memcmp(b->bssid, avoid[k], 6) == 0;
			}
			if (b->ssid_length == what->ssid_length && memcmp(b->ssid, what->ssid, what->ssid_length) == 0 && !avoided &&
			    (any_bssid ? t == NULL || band_score(b) > band_score(t) : memcmp(b->bssid, what->bssid, 6) == 0)) {
				t = b;
			}
		}
		if (t != NULL) {
			w.target = *t;
		}
		lck_mtx_unlock(&w2_lock);
	}
	if (t == NULL) {
		printf("w2: join: that network has not been heard\n");
		return ENOENT;
	}

	printf("w2: join: access point ..:%02x:%02x:%02x, %u MHz, %d dBm%s\n", w.target.bssid[3], w.target.bssid[4], w.target.bssid[5],
	    w.target.frequency, w.target.rssi, avoid_count ? " (another of that network)" : "");

	/* the handshake's part, ready before the access point begins it */
	bzero(&w.wpa, sizeof(w.wpa));
	if (wpa2) {
		if (what->passphrase_length != 0) {
			lx_wpa_pmk(what->passphrase, what->passphrase_length, what->ssid, what->ssid_length, w.wpa.pmk);
		} else {
			memcpy(w.wpa.pmk, what->pmk, 32);
		}
		memcpy(w.rsn, rsn, sizeof(rsn));
		memcpy(w.wpa.own, w.mac, 6);
		memcpy(w.wpa.peer, w.target.bssid, 6);
		read_random(w.wpa.own_nonce, sizeof(w.wpa.own_nonce));
		w.wpa.rsn = w.rsn;
		w.wpa.rsn_length = sizeof(rsn);
	}
	w.handshake = LX_WPA_NOTHING;
	w.eapol = 0;
	memcpy(w.bssid, w.target.bssid, 6);

	bzero(req, sizeof(req));
	req[0] = what->ssid_length;
	memcpy(req + 1, what->ssid, what->ssid_length);
	memcpy(req + 34, w.target.bssid, 6);
	chan_def(req + 40, w.target.frequency, w.target.frequency < 4000 ? 0 : 1, 0);
	put32(req + 48, wpa2 ? 0x09 : 0x08);                    /* the control port is ours (WPA2); WPA/WPA2 in use */
	req[52] = 0x88;
	req[53] = 0x8e;
	req[59] = w.vif;
	put16(req + 60, wpa2 ? sizeof(rsn) : 0);
	if (wpa2) {
		memcpy(req + 64, rsn, sizeof(rsn));
	}
	lck_mtx_lock(&w2_lock);
	w.connect_done = false;
	w.connect_status = 0;
	w.phase = HAKIOS_STA_JOINING;
	lck_mtx_unlock(&w2_lock);
	error = send_msg(SM_CONNECT_REQ, req, (uint16_t)(64 + (wpa2 ? sizeof(rsn) : 0)), CFM(SM_CONNECT_REQ), r, sizeof(r));
	if (error == 0 && r[0] != 0) {
		printf("w2: join: the firmware refuses (%u)\n", r[0]);
		error = r[0] == 8 || r[0] == 9 ? EBUSY : EIO;
	}
	if (error != 0) {
		goto out;
	}
	lck_mtx_lock(&w2_lock);
	for (began = now_ms(); !w.connect_done && now_ms() - began < 15000;) {
		sleep_ms(&w.connect_done, 200);
	}
	lck_mtx_unlock(&w2_lock);
	if (!w.connect_done || w.connect_status != 0) {
		printf("w2: join: %s (status %u)\n", w.connect_done ? "the access point refused" : "no answer", w.connect_status);
		error = w.connect_done ? ECONNREFUSED : ETIMEDOUT;
		goto out;
	}
	if (wpa2) {
		lck_mtx_lock(&w2_lock);
		for (began = now_ms(); w.handshake != LX_WPA_KEYS && now_ms() - began < 8000;) {
			sleep_ms(&w.handshake, 100);
		}
		lck_mtx_unlock(&w2_lock);
		if (w.handshake != LX_WPA_KEYS) {
			printf("w2: join: the handshake did not come through (%u messages; %s)\n", w.eapol,
			    w.handshake == LX_WPA_ANSWER ? "our second was not answered: a wrong passphrase gives this" :
			    w.handshake == LX_WPA_BAD ? "a message was not good" : "the access point did not begin it");
			error = EAUTH;
			goto out;
		}
		IOSleep(30);                                    /* (message 4 out before there is a key to cipher it with) */
		if ((error = key_add(0, w.ap_idx, w.wpa.pairwise, 16, true)) != 0 ||
		    (error = key_add(w.wpa.group_index, 0xff, w.wpa.group, w.wpa.group_length, false)) != 0) {
			printf("w2: join: the keys were refused (%d)\n", error);
			goto out;
		}
	}
	open[0] = w.ap_idx;
	open[1] = 1;
	send_msg(ME_SET_CONTROL_PORT_REQ, open, sizeof(open), CFM(ME_SET_CONTROL_PORT_REQ), NULL, 0);
	w.joined = true;
	w.phase = HAKIOS_STA_JOINED;
	hakios_net_link_state(true);
	printf("w2: THE NETWORK IS JOINED (%u dBm, %u MHz): frames can pass\n", w.target.rssi, w.target.frequency);
	return 0;
out:
	w.last_error = error;
	if (w.connect_done && w.connect_status == 0) {
		uint8_t d[4] = { 3, 0, w.vif, 0 };

		send_msg(SM_DISCONNECT_REQ, d, sizeof(d), CFM(SM_DISCONNECT_REQ), NULL, 0);
	}
	w.phase = HAKIOS_STA_IDLE;
	return error;
}

/*
 * A join that the access point did not take up (802.11 status 16: no answer to our authentication; or no answer at all)
 * is tried again, twice: after a pause (the firmware may still be leaving the network wifid left a moment before), with
 * a fresh scan for that name, and with another access point of the same network when it has more than one (a band or a
 * mesh node heard well in an old scan may not hear us).
 */
static int
sta_join(const struct hakios_sta_join *what)
{
	uint8_t avoid[2][6];
	unsigned avoided = 0;
	int error = sta_join_once(what, avoid, 0, false);

	for (unsigned attempt = 1; attempt <= 2 && (error == ECONNREFUSED || error == ETIMEDOUT); attempt++) {
		uint8_t tried[6];
		bool any_bssid = true;

		memcpy(tried, w.target.bssid, 6);
		for (int i = 0; i < 6; i++) {
			any_bssid &= what->bssid[i] == 0;
		}
		if (any_bssid && avoided < 2) {
			memcpy(avoid[avoided++], tried, 6);
		}
		printf("w2: join: attempt %u of 3 after status %u\n", attempt + 1, w.connect_status);
		IOSleep(500);
		error = sta_join_once(what, avoid, avoided, true);
		if (error == ENOENT && avoided != 0) {             /* no other access point of that name: the same one again */
			avoided = 0;
			error = sta_join_once(what, avoid, 0, false);
		}
	}
	return error;
}

static void
sta_state(struct hakios_sta_state *s)
{
	bzero(s, sizeof(*s));
	s->phase = w.up ? w.phase : HAKIOS_STA_OFF;
	if (w.joined || w.phase == HAKIOS_STA_JOINING) {
		s->bss = w.target;
	}
	s->last_error = w.last_error;
}

static unsigned
sta_channels(struct hakios_sta_channel *channels, unsigned most)
{
	uint8_t c2[14 * 6], c5[28 * 6];
	unsigned n5 = channel_list(c2, c5), n = 0;

	for (unsigned i = 0; i < 14 + n5 && n < most; i++) {
		const uint8_t *c = i < 14 ? c2 + 6 * i : c5 + 6 * (i - 14);

		if (c[3] & 2) {
			continue;
		}
		channels[n].frequency = le16(c);
		channels[n].flags = (c[3] & 4) ? HAKIOS_STA_CHANNEL_PASSIVE | HAKIOS_STA_CHANNEL_RADAR : 0;
		n++;
	}
	return n;
}

/* what waits for the firmware's answers and so cannot be done on the poll thread: a new group key */
static void
worker_thread(__unused void *arg, __unused wait_result_t wr_)
{
	for (;;) {
		lck_mtx_lock(&w2_lock);
		while (!w.group_pending) {
			sleep_ms(&w.group_pending, 60000);
		}
		w.group_pending = false;
		lck_mtx_unlock(&w2_lock);
		if (w.joined) {
			int error = key_add(w.wpa.group_index, 0xff, w.wpa.group, w.wpa.group_length, false);

			printf("w2: the new group key (number %u): %s\n", w.wpa.group_index, error ? "REFUSED" : "installed");
		}
	}
}

static struct hakios_sta_ops sta_ops = {
	.power = sta_power,
	.scan = sta_scan,
	.heard = sta_heard,
	.join = sta_join,
	.leave = sta_leave,
	.state = sta_state,
	.channels = sta_channels,
};

static struct hakios_net_link w2_link = {
	.name = "the Amlogic W2",
	.wireless = true,
	.transmit = link_transmit,
};

static void
start_thread(__unused void *arg, __unused wait_result_t wr_)
{
	int error = firmware_start();

	if (error != 0) {
		printf("w2: the firmware did not start (%d): no Wi-Fi\n", error);
		return;
	}
	if (w.step < 6) {
		return;
	}
	memcpy(w2_link.address, w.mac, 6);
	memcpy(sta_ops.address, w.mac, 6);
	if (!hakios_net_attach(&w2_link)) {
		printf("w2: en0 has a link already: no Wi-Fi\n");
		return;
	}
	w.phase = HAKIOS_STA_IDLE;
	hakios_sta_attach(&sta_ops);
	hakios_w2bt_start();                                    /* Bluetooth, on the same chip, once Wi-Fi is up */
	printf("w2: en0 is the W2's; the station is there\n");
	/* the network of fw/network of the small image (a name, a passphrase), joined now and whenever the radio comes back */
	{
		static struct hakios_sta_join given;
		unsigned name_length = sizeof(given.ssid), passphrase_length = sizeof(given.passphrase);
		uint64_t size = 0;

		if (hakios_image_file("fw", "network", &size, NULL) != NULL &&
		    hakios_wifi_network(given.ssid, &name_length, given.passphrase, &passphrase_length)) {
			given.ssid_length = (uint8_t)name_length;
			given.passphrase_length = (uint8_t)passphrase_length;
			given.security = HAKIOS_STA_WPA2_PSK;
			hakios_sta_ask_join_standing(&given);
		}
	}
	if (w.step >= 7) {
		struct hakios_sta_scan all;

		bzero(&all, sizeof(all));
		sta_scan(&all);
		for (unsigned i = 0; i < w.bss_count; i++) {
			printf("w2:   %02x:%02x:%02x:..:%02x  %u MHz  %d dBm  \"%.*s\"\n", w.bss[i].bssid[0], w.bss[i].bssid[1],
			    w.bss[i].bssid[2], w.bss[i].bssid[5], w.bss[i].frequency, w.bss[i].rssi, w.bss[i].ssid_length,
			    (const char *)w.bss[i].ssid);
		}
	}
}

/* the bus's thread from here on: the firmware's events, polled */
void
hakios_w2_run(unsigned step)
{
	uint8_t comp[16];
	thread_t thread;
	unsigned failures = 0;

	w.step = step;
	w.end = RX_END_EXPAND;
	w.pages = w.pages_total = PAGES_EXPAND;
	w.prefix = 521;
	for (unsigned i = 0; i < N_SLOTS; i++) {
		w.slot_id[i] = ((i + 1) << 16 | (i + 1)) & 0x3ffffff;
	}
	w.ring = kalloc_data(RX_END_EXPAND - RX_START, Z_WAITOK | Z_ZERO);
	batches[0].data = kalloc_data(BATCH_BYTES, Z_WAITOK);
	batches[1].data = kalloc_data(BATCH_BYTES, Z_WAITOK);
	if (batches[0].data == NULL || batches[1].data == NULL) {
		printf("w2: no memory\n");
		return;
	}
	w.bss = kalloc_data(N_BSS * sizeof(*w.bss), Z_WAITOK | Z_ZERO);
	if (w.ring == NULL || w.bss == NULL) {
		printf("w2: no memory\n");
		return;
	}
	if (hakios_w2_sram_read(0x60000000, comp, sizeof(comp))) {
		printf("w2: shared memory %u bytes, message API %u, IPC version %u, %u RX descriptors\n", le16(comp), le16(comp + 2),
		    comp[4], comp[9]);
	}
	hakios_w2_write(IPC_UNMASK, 0x00004fbf);
	if (kernel_thread_start(start_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_set_thread_name(thread, "w2 start");
		thread_deallocate(thread);
	}
	if (kernel_thread_start(worker_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_set_thread_name(thread, "w2 work");
		thread_deallocate(thread);
	}
	if (kernel_thread_start(tx_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_set_thread_name(thread, "w2 tx");
		thread_deallocate(thread);
	}
	uint64_t stats_at = now_ms();
	for (;;) {
		uint32_t words[4];
		uint32_t st;
		bool busy = false;

		w.polls++;
		if (stats && now_ms() - stats_at >= 2000) {
			uint64_t ms = now_ms() - stats_at;
			static unsigned rx_frames0, tx_frames0;

			printf("w2: %u ms: %u polls, rx %llu bytes (%llu Mbit/s) in %u batches, %u frames (%u dropped; reordered %u, "
			    "timed out %u, stale %u); tx %u frames in %u batches (%u dropped), %d pages free\n", (unsigned)ms, w.polls,
			    w.rx_bytes, w.rx_bytes * 8 / 1000 / ms, w.rx_batches, w.rx_frames - rx_frames0, w.rx_dropped, w.reo_waited,
			    w.reo_timeouts, w.reo_stale, w.tx_frames - tx_frames0, w.tx_batches, w.tx_dropped, w.pages);
			w.reo_waited = w.reo_timeouts = w.reo_stale = 0;
			w.tx_batches = 0;
			rx_frames0 = w.rx_frames;
			tx_frames0 = w.tx_frames;
			w.polls = w.rx_batches = 0;
			w.rx_bytes = 0;
			stats_at = now_ms();
		}

		if (!hakios_w2_irq(words)) {
			if (++failures == 10) {
				printf("w2: the chip's interrupt does not answer\n");
			}
			IOSleep(10);
			continue;
		}
		st = words[1];
		if (st & ST_MSG) {
			messages();
		}
		if (st & ST_MSG_ACK) {
			lck_mtx_lock(&w2_lock);
			w.outstanding = false;
			thread_wakeup((event_t)&w.outstanding);
			lck_mtx_unlock(&w2_lock);
		}
		if (st & ST_DBG) {
			debug_string();
		}
		if (st & ST_EXT) {
			extended();
		}
		busy = rx_process(words[0]) != 0;
		reo_age();
		if (st & ST_TXCFM) {
			tx_confirmations();
			busy = true;
		}
		if (w.stop_pending) {
			tx_stop_check();
		}
		if (!busy) {
			IOSleep(idle_ms);
		} else if (busy_us) {
			IODelay(busy_us);
		}
	}
}

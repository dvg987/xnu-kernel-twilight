/*
 * The Amlogic box's sound out of its HDMI transmitter (S905X5M "S7D", Mi Box S Gen 3). Boot-arg hdmiaudio=N, off by default:
 * without it nothing here touches the hardware (hakios_hdmiaudio_init() is called at the end of hakios_hdmitx_init(), so
 * hdmitx=1 as well).
 *
 *   hdmiaudio=1   a look, read only: the audio block's bus clock, the PLL, the registers of the path, what the sink says it plays
 *   hdmiaudio=2   the path is brought up and a test tone plays (hdmiaudio_secs=5, hdmiaudio_ch=2|4|6|8: each channel its own pitch,
 *                 -21 dBFS), HDMI audio packets on
 *   hdmiaudio=3   as 2, and /dev/twl-audio is there for the audio server's plug-in (sys/hakios.hdmiaudio.h), instead of the tone
 *   hdmiaudio_pll=1     program the HiFi PLL (491.52 MHz, or 451.584 MHz for the 44.1 kHz family) if it is not locked
 *   hdmiaudio_mclk=N    the master clock's source (mux 0..7: 0 hifi1_pll, 4 hifi_pll, 5..7 fclk_div3/4/5), default 4
 *   hdmiaudio_path=spdif  feed the transmitter as the bootloader does: FRDDR_A -> the SoC's own S/PDIF-out A encoder -> TOHDMITX -> the
 *                       transmitter's S/PDIF input (AUD_MODE 2). Stereo only, but proven on this box (the bootloader's beep); no pin or jack is
 *                       involved, HDMI is the only output. Default (hdmiaudio_path unset) is I2S through TDM B.
 *   hdmiaudio_gate=1    open the audio bus clock (sys_audio) if it is closed (otherwise nothing is touched while it is)
 *
 * The path (the DT's i2s2hdmi, Amlogic's sound/soc/amlogic/auge: tdm.c, tdm_hw.c, ddr_mngr.c):
 *
 *   memory --FRDDR_A--> TDMOUT_B (master clock B, 32-bit slots, 2 per lane, up to 4 lanes) --TOHDMITX--> HDMI TX I2S input
 *
 * The register numbers are the TXHD2/TM2 audio block's (the DT says "txhd2-audio-clocks", "t5-audio-ddr-manager" and
 * "tm2-revb-snd-tdmb"); they are the sources' and have not been seen on this box yet: hdmiaudio=1 prints them to check
 * (a block that is clocked reads its reset values; the log says what is expected). The register file is at 0xfe330000, a
 * register is 4 bytes at index * 4.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <miscfs/devfs/devfs.h>
#include <pexpert/pexpert.h>
#include <sys/conf.h>
#include <sys/errno.h>
#include <sys/hakios.hdmiaudio.h>
#include <sys/hakios.hdmitx.h>
#include <sys/proc.h>
#include <sys/systm.h>
#include <sys/uio.h>
#include <vm/vm_kern.h>

extern void clean_dcache(vm_offset_t addr, unsigned count, int phys);

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern uint64_t kvtophys(vm_offset_t va);
extern void IOSleep(unsigned int milliseconds);
extern void IODelay(unsigned int microseconds);

#define AUD_BASE        0xfe330000UL
#define ANA_BASE        0xfe008000UL
#define CLK_BASE        0xfe000000UL

/* ANACTRL: HIFI0 PLL (hifi_pll) at +0x100, HIFI1 (hifi1_pll) at +0x114, five registers each */
#define HIFI0PLL_CTRL0  0x100
#define HIFI1PLL_CTRL0  0x114
#define SYS_CLK_EN0_REG1 0x48

/* the audio register file (index; the byte offset is index * 4) */
#define EE_AUDIO_CLK_GATE_EN0   0x000
#define EE_AUDIO_MCLK_B_CTRL    0x003           /* TXHD2: EE_AUDIO_MCLK_B_CTRL(1): [31] enable [26:24] source [15:0] divider - 1 */
#define EE_AUDIO_SW_RESET0      0x00a
#define EE_AUDIO_MST_B_SCLK_CTRL0 0x012         /* [31:30] enables [29:20] mclk/sclk - 1 [19:10] lrclk high [9:0] bclk per lrclk - 1 */
#define EE_AUDIO_MST_B_SCLK_CTRL1 0x013
#define EE_AUDIO_CLK_TDMOUT_B_CTRL 0x025        /* [31:30] enable [29] bclk invert [27:24] sclk source [23:20] lrclk source */
#define EE_AUDIO_FRDDR_A_CTRL0  0x070
#define EE_AUDIO_FRDDR_A_CTRL1  0x071
#define EE_AUDIO_FRDDR_A_START  0x072
#define EE_AUDIO_FRDDR_A_FINISH 0x073
#define EE_AUDIO_FRDDR_A_INT    0x074
#define EE_AUDIO_FRDDR_A_STATUS1 0x075
#define EE_AUDIO_FRDDR_A_STATUS2 0x076          /* the address being read */
#define EE_AUDIO_CLK_SPDIFOUT_CTRL 0x028        /* [31] enable [26:24] source (4 hifi_pll) [15:0] divider - 1: U-Boot's 0x4f for 6.144 MHz = 128 fs */
#define EE_AUDIO_SPDIFOUT_CTRL0 0x123
#define EE_AUDIO_SPDIFOUT_CTRL1 0x124
#define EE_AUDIO_SPDIFOUT_SWAP  0x126
#define EE_AUDIO_SPDIFOUT_CHSTS0 0x127
#define EE_AUDIO_SPDIFOUT_CHSTS6 0x12d
#define EE_AUDIO_SPDIFOUT_MUTE_VAL 0x133
#define EE_AUDIO_TDMOUT_B_STAT  0x157
#define EE_AUDIO_TDMOUT_B_GAIN0 0x158
#define EE_AUDIO_TDMOUT_B_MUTE_VAL 0x15a
#define EE_AUDIO_TDMOUT_B_MUTE0 0x15b
#define EE_AUDIO_TDMOUT_B_MASK_VAL 0x15f
#define EE_AUDIO_TDMOUT_B_GAIN2 0x2b2
#define EE_AUDIO_TDMOUT_B_GAIN_EN 0x2bc
#define EE_AUDIO_TDMOUT_B_GAIN_CTRL 0x2bd
#define EE_AUDIO_FRDDR_A_INIT   0x079           /* where the FIFO starts reading when it is enabled (ddr_mngr.c writes it after START) */
#define EE_AUDIO_FRDDR_A_CTRL2  0x07a           /* [31:24] channels - 1 [4] source enable [2:0] destination */
#define EE_AUDIO_ARB_CTRL       0x0a0
#define EE_AUDIO_TDMOUT_B_CTRL0 0x150           /* [31] enable [29:28] FIFO reset [19:15] bclk skew [9:5] slots - 1 [4:0] slot width - 1 */
#define EE_AUDIO_TDMOUT_B_CTRL1 0x151           /* [28] lrclk invert [25:24] FIFO id [12:8] bit width - 1 [6:4] FRDDR type */
#define EE_AUDIO_TDMOUT_B_SWAP0 0x152
#define EE_AUDIO_TDMOUT_B_MASK0 0x153           /* 0..3 */
#define EE_AUDIO_TOHDMITX_CTRL0 0x1d1

#define FRDDR_DEST_TDMOUT_B     1
#define TDM_INDEX               1               /* TDM B, master clock B (clk_sel 1) */
#define FIFO_BURST              8
#define FIFO_DEPTH              0x400           /* the t5 DDR manager's FIFO: 1 KiB */

#define RING_BYTES      (2u << 20)
#define TICK_MS         5
#define LEAD_MS         20

static struct {
	vm_offset_t aud, ana, clk;
	bool probed, gate_ok;
	int level;
	uint32_t mclk_src;
	bool spdif;                     /* hdmiaudio_path=spdif */
} a;

static uint32_t aud_rd(uint32_t reg) { return *(volatile uint32_t *)(a.aud + reg * 4); }
static void aud_wr(uint32_t reg, uint32_t v) { *(volatile uint32_t *)(a.aud + reg * 4) = v; }
static void aud_bits(uint32_t reg, uint32_t mask, uint32_t v) { aud_wr(reg, (aud_rd(reg) & ~mask) | (v & mask)); }

static LCK_GRP_DECLARE(ha_lck_grp, "hdmiaudio");
static LCK_MTX_DECLARE(ha_lock, &ha_lck_grp);

static struct {
	bool running;
	struct twl_audio_format fmt;
	uint8_t *ring;                  /* the FIFO's memory: one piece of physical memory */
	uint64_t phys;
	uint32_t ring_bytes, frame_bytes;       /* of the container: 32-bit samples */
	uint32_t wp, clear;             /* where the next frame goes; what is zeroed behind the hardware, up to */
	uint32_t last_hw;
	uint64_t played_bytes, played_time;
	uint32_t underruns, dropped_frames;
	uint32_t gain;                  /* Q16 */
	thread_call_t tick;
	bool armed;
} s = { .gain = 65536 };

/* ---- a look ------------------------------------------------------------------------------------------------ */

static void
dump_state(void)
{
	static const struct { const char *name; uint32_t reg; } regs[] = {
		{ "GATE_EN0", EE_AUDIO_CLK_GATE_EN0 }, { "MCLK_B", EE_AUDIO_MCLK_B_CTRL }, { "SW_RESET0", EE_AUDIO_SW_RESET0 },
		{ "MST_B_SCLK0", EE_AUDIO_MST_B_SCLK_CTRL0 }, { "MST_B_SCLK1", EE_AUDIO_MST_B_SCLK_CTRL1 },
		{ "CLK_TDMOUT_B", EE_AUDIO_CLK_TDMOUT_B_CTRL }, { "FRDDR_A_CTRL0", EE_AUDIO_FRDDR_A_CTRL0 },
		{ "FRDDR_A_CTRL1", EE_AUDIO_FRDDR_A_CTRL1 }, { "FRDDR_A_CTRL2", EE_AUDIO_FRDDR_A_CTRL2 },
		{ "FRDDR_A_START", EE_AUDIO_FRDDR_A_START }, { "FRDDR_A_FINISH", EE_AUDIO_FRDDR_A_FINISH },
		{ "FRDDR_A_STATUS1", EE_AUDIO_FRDDR_A_STATUS1 }, { "FRDDR_A_STATUS2", EE_AUDIO_FRDDR_A_STATUS2 },
		{ "ARB_CTRL", EE_AUDIO_ARB_CTRL }, { "TDMOUT_B_CTRL0", EE_AUDIO_TDMOUT_B_CTRL0 },
		{ "TDMOUT_B_CTRL1", EE_AUDIO_TDMOUT_B_CTRL1 }, { "TDMOUT_B_SWAP0", EE_AUDIO_TDMOUT_B_SWAP0 },
		{ "TDMOUT_B_MASK0", EE_AUDIO_TDMOUT_B_MASK0 }, { "TOHDMITX_CTRL0", EE_AUDIO_TOHDMITX_CTRL0 },
	};

	for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
		printf("hdmiaudio: %-16s [%#05x] = %#010x\n", regs[i].name, regs[i].reg * 4, aud_rd(regs[i].reg));
	}
}

static void
dump_plls(void)
{
	for (unsigned p = 0; p < 2; p++) {
		uint32_t base = p ? HIFI1PLL_CTRL0 : HIFI0PLL_CTRL0;
		uint32_t c0 = *(volatile uint32_t *)(a.ana + base), c1 = *(volatile uint32_t *)(a.ana + base + 4);
		uint32_t m = c0 & 0x1ff, n = c0 >> 12 & 7, od = c0 >> 20 & 7, frac = c1 & 0x1ffff;

		printf("hdmiaudio: %s: ctrl0 %#x ctrl1 %#x (m %u frac %#x n %u od %u, enable %u, lock %u)\n", p ? "hifi1_pll" : "hifi_pll", c0,
		    c1, m, frac, n, od, c1 >> 27 & 1, c0 >> 31 & 1);
	}
}

/* The HiFi PLL, if it is not locked: DCO = 12 MHz * (m + frac / 2^17), output = DCO / 2^od. Amlogic's S7D PLL table has od 2 and an
 * m near 163 for the audio rate (491.52 MHz: m 163, frac 0x1AE14); 451.584 MHz is m 150, frac 0x10E56. Whether a plain store gets
 * through (the PLLs the SCMI firmware owns ignore it) is for the log to say. */
static bool
pll_up(bool family_44k)
{
	uint32_t m = family_44k ? 150 : 163, frac = family_44k ? 0x10e56 : 0x1ae14;
	volatile uint32_t *c = (volatile uint32_t *)(a.ana + HIFI0PLL_CTRL0);

	if (c[0] & 1u << 31) {
		return true;
	}
	c[0] = 0x00010000;
	c[1] = 0x11480000;
	c[2] = 0x1219b010;
	c[3] = 0x20008010;
	c[4] = 0x1f01110f;
	IODelay(10);
	c[0] = (c[0] & ~0x1ffu & ~(7u << 20)) | m | 2u << 20 | 1u << 12;                      /* n 1 */
	c[1] = (c[1] & ~0x1ffffu) | frac | 1u << 27;
	IODelay(20);
	c[0] &= ~(1u << 29);                                                        /* out of reset */
	for (unsigned ms = 0; ms < 20; ms++) {
		if (c[0] & 1u << 31) {
			printf("hdmiaudio: hifi_pll locked: ctrl0 %#x ctrl1 %#x\n", c[0], c[1]);
			return true;
		}
		IOSleep(1);
	}
	printf("hdmiaudio: hifi_pll did not lock (ctrl0 %#x ctrl1 %#x): a plain store is not enough, SCMI?\n", c[0], c[1]);
	return false;
}

/* ---- the path ---------------------------------------------------------------------------------------------- */

static void
path_off(void)
{
	aud_bits(EE_AUDIO_TDMOUT_B_CTRL0, 1u << 31, 0);
	aud_wr(EE_AUDIO_FRDDR_A_CTRL0, 0);
	aud_wr(EE_AUDIO_FRDDR_A_CTRL2, 0);
	aud_bits(EE_AUDIO_TOHDMITX_CTRL0, 3u << 28 | 1u << 30 | 1u << 31, 0);
	aud_bits(EE_AUDIO_SPDIFOUT_CTRL0, 1u << 31, 0);
}

/*
 * Clocks and TDM for `fs' frames a second: 32-bit slots, 2 slots a lane, so the bit clock is 64 fs; the master clock 256 fs. The
 * PLL (491.52 MHz for the 48 kHz family, 451.584 for 44.1) divides down: 192 kHz is /10, 48 kHz is /40.
 */
static bool
path_on(const struct twl_audio_format *f, unsigned lanes)
{
	uint32_t fs = f->kind == TWL_AUDIO_HBR ? 192000 : f->rate;
	bool fam44 = fs % 11025 == 0;
	uint32_t pll = fam44 ? 451584000u : 491520000u, mclk = fs * 256, div = pll / mclk;
	uint32_t fifo_id = 0, mask = lanes >= 1 ? 0x3 : 0;

	if (div == 0 || pll % mclk != 0) {
		printf("hdmiaudio: no divider for %u Hz\n", fs);
		return false;
	}
	path_off();
	/* the bootloader left FRDDR_A running on a buffer of its own, into the SPDIF-out-A path (CTRL2 destination 3, START 0x7f7607b0): reset
	 * the FIFO and TDM out so that what is programmed below is what runs (aml_audio_reset(): a pulse on the block's bit in SW_RESET0) */
	aud_bits(EE_AUDIO_SW_RESET0, 1u << 9 | 1u << 13, 1u << 9 | 1u << 13);
	IODelay(5);
	aud_bits(EE_AUDIO_SW_RESET0, 1u << 9 | 1u << 13, 0);
	aud_bits(EE_AUDIO_CLK_GATE_EN0, 1u << 0 | 1u << 7 | 1u << 9, 1u << 0 | 1u << 7 | 1u << 9);   /* DDR arb, TDMOUT_B, FRDDR_A */
	if (!a.spdif) {
		/* master clock B: source, divider, then enable */
		aud_wr(EE_AUDIO_MCLK_B_CTRL, a.mclk_src << 24 | (div - 1));
		aud_bits(EE_AUDIO_MCLK_B_CTRL, 1u << 31, 1u << 31);
		/* bit and frame clock: bclk = mclk / 4, 64 bit clocks a frame, lrclk high for half */
		aud_wr(EE_AUDIO_MST_B_SCLK_CTRL0, 3u << 30 | (256 / 64 - 1) << 20 | 31u << 10 | 63);
		/* TDM out B: clocks from master B, I2S timing (bclk inverted, lrclk inverted, bclk skew 2: tdm.c for a master with sclk_ws_inv) */
		aud_bits(EE_AUDIO_CLK_TDMOUT_B_CTRL, 3u << 30 | 1u << 29 | 0xf << 24 | 0xf << 20,
		    3u << 30 | 1u << 29 | TDM_INDEX << 24 | TDM_INDEX << 20);
		aud_bits(EE_AUDIO_TDMOUT_B_CTRL1, 1u << 28, 1u << 28);
		aud_bits(EE_AUDIO_TDMOUT_B_CTRL0, 0x1f << 15 | 0x3ff, 2u << 15 | (2 - 1) << 5 | (32 - 1));
		/* the FIFO feeds the TDM: id, 32-bit samples (type 4) */
		aud_bits(EE_AUDIO_TDMOUT_B_CTRL1, 3u << 24 | 0x1f << 8 | 0x7 << 4, fifo_id << 24 | (32 - 1) << 8 | 4u << 4);
		/* channel to lane and slot: the identity (channel 2n, 2n + 1 on lane n); the slots of the lanes in use are on */
		aud_wr(EE_AUDIO_TDMOUT_B_SWAP0, 0x76543210);
		for (unsigned l = 0; l < 4; l++) {
			aud_wr(EE_AUDIO_TDMOUT_B_MASK0 + l, l < lanes ? mask : 0);
		}
		/* FIFO reset (tdm_hw.c: 3<<28 off, then bit 29, then bit 28) */
		aud_bits(EE_AUDIO_TDMOUT_B_CTRL0, 3u << 28, 0);
		aud_bits(EE_AUDIO_TDMOUT_B_CTRL0, 1u << 29, 1u << 29);
		aud_bits(EE_AUDIO_TDMOUT_B_CTRL0, 1u << 28, 1u << 28);
		/* the gain stage and the mutes of TDM B: unity, nothing muted (the sources leave them to their reset values; a gain of 0 would be silence) */
		aud_wr(EE_AUDIO_TDMOUT_B_GAIN_EN, 0);
		aud_wr(EE_AUDIO_TDMOUT_B_GAIN0, 0);
		aud_wr(EE_AUDIO_TDMOUT_B_GAIN0 + 1, 0);
		aud_wr(EE_AUDIO_TDMOUT_B_GAIN2, 0);
		aud_wr(EE_AUDIO_TDMOUT_B_GAIN2 + 1, 0);
		aud_wr(EE_AUDIO_TDMOUT_B_MUTE_VAL, 0);
		for (unsigned l = 0; l < 4; l++) {
			aud_wr(EE_AUDIO_TDMOUT_B_MUTE0 + l, 0);
		}
	} else {
		/* U-Boot's aml_audio_init() (drivers/amlogic/media/vout/hdmitx/sound.c): the S/PDIF-out A encoder from the HiFi PLL */
		aud_bits(EE_AUDIO_CLK_GATE_EN0, 1u << 17, 1u << 17);
		aud_wr(EE_AUDIO_CLK_SPDIFOUT_CTRL, 1u << 31 | a.mclk_src << 24 | (pll / (fs * 128) - 1));
		aud_wr(EE_AUDIO_SPDIFOUT_MUTE_VAL, 0);
		aud_bits(EE_AUDIO_SPDIFOUT_CTRL0, 1u << 22 | 1u << 21 | 1u << 20 | 1u << 19 | 0xffff, 1u << 22 | 1u << 21 | 0x3);
		aud_bits(EE_AUDIO_SPDIFOUT_CTRL1, 3u << 24 | 0x1f << 8 | 0x7 << 4, 0u << 24 | (32 - 1) << 8 | 3u << 4);
		aud_wr(EE_AUDIO_SPDIFOUT_SWAP, 1u << 4);
		/* channel status of both channels, U-Boot's 0x0100 / 0x0200 (consumer, category 1, 48 kHz); byte 0 bit 1 for a stream that is not audio */
		{
			uint32_t fscode = fs >= 192000 ? 0x0e00 : fs >= 96000 ? 0x0a00 : fs >= 48000 ? 0x0200 : fs >= 44100 ? 0x0000 : 0x0300;
			uint32_t b0 = f->kind == TWL_AUDIO_PCM ? 0x0100 : 0x0102;

			aud_wr(EE_AUDIO_SPDIFOUT_CHSTS0, (fscode << 16) | b0);
			aud_wr(EE_AUDIO_SPDIFOUT_CHSTS6, (fscode << 16) | b0);
		}
		/* reset the encoder (bits 28, 29) as aml_spdif_enable() does; it is enabled in path_go() */
		aud_bits(EE_AUDIO_SPDIFOUT_CTRL0, 3u << 28, 0);
		aud_bits(EE_AUDIO_SPDIFOUT_CTRL0, 1u << 29, 1u << 29);
		aud_bits(EE_AUDIO_SPDIFOUT_CTRL0, 1u << 28, 1u << 28);
	}
	/* the DDR manager reads the ring */
	aud_wr(EE_AUDIO_ARB_CTRL, 1u << 31 | 0xff);
	aud_wr(EE_AUDIO_FRDDR_A_START, (uint32_t)s.phys);
	aud_wr(EE_AUDIO_FRDDR_A_INIT, (uint32_t)s.phys);                                /* (t5 does not have int_start_same_addr: INIT is separate) */
	aud_wr(EE_AUDIO_FRDDR_A_FINISH, (uint32_t)s.phys + s.ring_bytes - FIFO_BURST);
	aud_wr(EE_AUDIO_FRDDR_A_INT, (uint32_t)s.phys + s.ring_bytes / 2);
	/* the FIFO as the bootloader had it (0x3f1f0200: depth 0x40 and threshold 0x20 bursts, which runs on this chip) */
	aud_wr(EE_AUDIO_FRDDR_A_CTRL1, (0x40 - 1) << 24 | (0x20 - 1) << 16 | 2u << 8);
	aud_bits(EE_AUDIO_FRDDR_A_CTRL0, 0xffu << 16 | 1, 4u << 16 | 1);               /* aml_frddr_set_intrpt(): 4 << 16; "ugt" */
	/* the channel count, the destination (TDMOUT_B), then the source enable after the FIFO is on (ddr_mngr.c's order) */
	aud_bits(EE_AUDIO_FRDDR_A_CTRL2, 0xffu << 24 | 7, (uint32_t)(f->channels - 1) << 24 | (a.spdif ? 3u : FRDDR_DEST_TDMOUT_B));
	if (a.spdif) {
		/* U-Boot: spdif_clk_cap_inv (bit 3), spdif A, then the clock (30) and data (31) enables */
		aud_wr(EE_AUDIO_TOHDMITX_CTRL0, 1u << 3);
		aud_bits(EE_AUDIO_TOHDMITX_CTRL0, 1u << 30, 1u << 30);
		aud_bits(EE_AUDIO_TOHDMITX_CTRL0, 1u << 31, 1u << 31);
		return true;
	}
	/* master B to the HDMI transmitter: tdm_hw.c i2s_to_hdmitx_ctrl(separated, TDM B, clk_sel 1) */
	aud_wr(EE_AUDIO_TOHDMITX_CTRL0, 1u << 20 | 1u << 19 | TDM_INDEX << 16 | TDM_INDEX << 12 | TDM_INDEX << 8 | 1u << 7 | TDM_INDEX << 4);
	aud_bits(EE_AUDIO_TOHDMITX_CTRL0, 1u << 28, 1u << 28);
	aud_bits(EE_AUDIO_TOHDMITX_CTRL0, 1u << 29, 1u << 29);
	return true;
}

static void
path_go(void)
{
	uint32_t c2 = aud_rd(EE_AUDIO_FRDDR_A_CTRL2);

	aud_wr(EE_AUDIO_FRDDR_A_CTRL2, c2 & ~(1u << 20 | 1u << 12 | 1u << 4));
	aud_bits(EE_AUDIO_FRDDR_A_CTRL0, 1u << 31, 1u << 31);
	aud_wr(EE_AUDIO_FRDDR_A_CTRL2, c2 | 1u << 4);                                   /* source select enable */
	if (a.spdif) {
		aud_bits(EE_AUDIO_SPDIFOUT_CTRL0, 1u << 31, 1u << 31);
	} else {
		aud_bits(EE_AUDIO_TDMOUT_B_CTRL0, 1u << 31, 1u << 31);
	}
}

/* ---- the ring ---------------------------------------------------------------------------------------------- */

static uint32_t
hw_offset(void)
{
	uint64_t at = aud_rd(EE_AUDIO_FRDDR_A_STATUS2);

	return at >= (uint32_t)s.phys && at < (uint32_t)s.phys + s.ring_bytes ? (uint32_t)(at - (uint32_t)s.phys) : s.last_hw;
}

/* With ha_lock held: how far the hardware has read; what it has passed becomes silence */
static void
ring_service(void)
{
	uint32_t hw = hw_offset(), step = (hw + s.ring_bytes - s.last_hw) % s.ring_bytes;

	s.played_bytes += step;
	s.last_hw = hw;
	absolutetime_to_nanoseconds(mach_absolute_time(), &s.played_time);
	while (s.clear != hw) {
		uint32_t n = (hw > s.clear ? hw : s.ring_bytes) - s.clear;

		memset(s.ring + s.clear, 0, n);
		clean_dcache((vm_offset_t)(s.ring + s.clear), n, 0);
		s.clear = (s.clear + n) % s.ring_bytes;
	}
}

static unsigned diag_ticks;
static uint32_t diag_every;              /* hdmiaudio_diag=1: the diagnostic lines every second; else once, as a stream starts */

/* Once a second while a stream plays: where the FIFO reads, where we write, and the registers of the path */
static void
diagnose(void)
{
	uint32_t st2 = aud_rd(EE_AUDIO_FRDDR_A_STATUS2);

	printf("hdmiaudio: diag: FRDDR_A STATUS1 %#x STATUS2 %#x (ring %#x..%#x, hw offset %#x) hw %#x wp %#x lag %#x played %llu B under %u drop %u\n",
	    aud_rd(EE_AUDIO_FRDDR_A_STATUS1), st2, (uint32_t)s.phys, (uint32_t)s.phys + s.ring_bytes, hw_offset(), s.last_hw, s.wp,
	    (s.wp + s.ring_bytes - s.last_hw) % s.ring_bytes, s.played_bytes, s.underruns, s.dropped_frames);
	printf("hdmiaudio: diag: FRDDR_A CTRL0 %#x CTRL1 %#x CTRL2 %#x START %#x INIT %#x FINISH %#x | TDMOUT_B CTRL0 %#x CTRL1 %#x SWAP0 %#x MASK0 %#x\n",
	    aud_rd(EE_AUDIO_FRDDR_A_CTRL0), aud_rd(EE_AUDIO_FRDDR_A_CTRL1), aud_rd(EE_AUDIO_FRDDR_A_CTRL2), aud_rd(EE_AUDIO_FRDDR_A_START),
	    aud_rd(EE_AUDIO_FRDDR_A_INIT), aud_rd(EE_AUDIO_FRDDR_A_FINISH), aud_rd(EE_AUDIO_TDMOUT_B_CTRL0), aud_rd(EE_AUDIO_TDMOUT_B_CTRL1),
	    aud_rd(EE_AUDIO_TDMOUT_B_SWAP0), aud_rd(EE_AUDIO_TDMOUT_B_MASK0));
	printf("hdmiaudio: diag: MCLK_B %#x MST_B_SCLK0 %#x SCLK1 %#x CLK_TDMOUT_B %#x TOHDMITX %#x GATE0 %#x RESET0 %#x ARB %#x\n",
	    aud_rd(EE_AUDIO_MCLK_B_CTRL), aud_rd(EE_AUDIO_MST_B_SCLK_CTRL0), aud_rd(EE_AUDIO_MST_B_SCLK_CTRL1),
	    aud_rd(EE_AUDIO_CLK_TDMOUT_B_CTRL), aud_rd(EE_AUDIO_TOHDMITX_CTRL0), aud_rd(EE_AUDIO_CLK_GATE_EN0), aud_rd(EE_AUDIO_SW_RESET0),
	    aud_rd(EE_AUDIO_ARB_CTRL));
	printf("hdmiaudio: diag: TDMOUT_B STAT %#x GAIN0 %#x GAIN2 %#x GAIN_EN %#x GAIN_CTRL %#x MUTE_VAL %#x MUTE0 %#x MASK_VAL %#x | SPDIFOUT CTRL0 %#x CTRL1 %#x CLK %#x CHSTS0 %#x SWAP %#x\n",
	    aud_rd(EE_AUDIO_TDMOUT_B_STAT), aud_rd(EE_AUDIO_TDMOUT_B_GAIN0), aud_rd(EE_AUDIO_TDMOUT_B_GAIN2), aud_rd(EE_AUDIO_TDMOUT_B_GAIN_EN),
	    aud_rd(EE_AUDIO_TDMOUT_B_GAIN_CTRL), aud_rd(EE_AUDIO_TDMOUT_B_MUTE_VAL), aud_rd(EE_AUDIO_TDMOUT_B_MUTE0), aud_rd(EE_AUDIO_TDMOUT_B_MASK_VAL),
	    aud_rd(EE_AUDIO_SPDIFOUT_CTRL0), aud_rd(EE_AUDIO_SPDIFOUT_CTRL1), aud_rd(EE_AUDIO_CLK_SPDIFOUT_CTRL), aud_rd(EE_AUDIO_SPDIFOUT_CHSTS0),
	    aud_rd(EE_AUDIO_SPDIFOUT_SWAP));
	hakios_hdmitx_audio_show();
}

static void
ring_tick(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	uint64_t deadline;

	lck_mtx_lock(&ha_lock);
	if (s.running) {
		ring_service();
		if (++diag_ticks == 1 || (diag_every && diag_ticks % (1000 / TICK_MS) == 1)) {
			diagnose();
		}
		clock_interval_to_deadline(TICK_MS, NSEC_PER_MSEC, &deadline);
		thread_call_enter_delayed(s.tick, deadline);
	} else {
		s.armed = false;
	}
	lck_mtx_unlock(&ha_lock);
}

/*
 * The ring is cacheable memory that the audio DMA (FRDDR) reads from RAM: what the processor wrote is cleaned out to RAM, or the
 * hardware plays whatever RAM held before (static).
 */

static void
ring_clean(uint32_t from, uint32_t to)
{
	if (to >= from) {
		clean_dcache((vm_offset_t)(s.ring + from), to - from, FALSE);
	} else {
		clean_dcache((vm_offset_t)(s.ring + from), s.ring_bytes - from, FALSE);
		clean_dcache((vm_offset_t)s.ring, to, FALSE);
	}
}

/* With ha_lock held. `data' is `frames' frames of the format's width. */
static void
ring_write(const void *data, uint32_t frames)
{
	const struct twl_audio_format *f = &s.fmt;
	uint32_t hw = hw_offset(), lag = (s.wp + s.ring_bytes - hw) % s.ring_bytes, lead = f->rate * LEAD_MS / 1000 * s.frame_bytes;
	bool pcm = f->kind == TWL_AUDIO_PCM;

	if (f->kind == TWL_AUDIO_HBR) {
		lead = 192000 * LEAD_MS / 1000 * s.frame_bytes;
	}
	if (lag > s.ring_bytes / 4 * 3) {                       /* the writer fell behind the hardware: begin again ahead of it */
		s.underruns++;
		s.wp = (hw + lead) % s.ring_bytes;
		lag = lead;
	}
	if (lag + (uint64_t)frames * s.frame_bytes > s.ring_bytes / 2) {
		s.dropped_frames += frames;
		return;
	}
	uint32_t from = s.wp;

	for (uint32_t i = 0; i < frames * f->channels; i++) {
		int32_t v;

		if (f->bits == 16) {
			int32_t x = ((const int16_t *)data)[i];

			v = pcm ? (int32_t)(((int64_t)x * s.gain) >> 0) : x << 16;      /* Q16 gain on a 16-bit sample is the 32-bit sample */
		} else {
			int32_t x = ((const int32_t *)data)[i];

			v = pcm ? (int32_t)(((int64_t)x * s.gain) >> 16) : x;
		}
		*(int32_t *)(s.ring + s.wp) = v;
		s.wp = (s.wp + 4) % s.ring_bytes;
	}
	ring_clean(from, s.wp);
}

/* ---- the format -------------------------------------------------------------------------------------------- */

static int
stream_stop_locked(void)
{
	if (s.running) {
		hakios_hdmitx_audio_stop();
		path_off();
		s.running = false;
	}
	return 0;
}

static int
stream_start_locked(const struct twl_audio_format *f)
{
	struct hakios_hdmitx_audio h = { 0 };
	unsigned lanes;
	uint32_t hbr_rate = 192000;

	if (a.level < 2 || !a.gate_ok) {
		return ENXIO;
	}
	if (a.spdif && (f->channels != 2 || f->kind == TWL_AUDIO_HBR)) {
		return EINVAL;                                  /* (the S/PDIF encoder carries one stereo pair) */
	}
	if ((f->channels != 2 && f->channels != 4 && f->channels != 6 && f->channels != 8) || (f->bits != 16 && f->bits != 32) ||
	    f->kind > TWL_AUDIO_HBR || f->rate < 32000 || f->rate > 192000) {
		return EINVAL;
	}
	if ((f->kind == TWL_AUDIO_IEC61937 && f->channels != 2) || (f->kind == TWL_AUDIO_HBR && (f->channels != 8 || f->rate != hbr_rate))) {
		return EINVAL;
	}
	stream_stop_locked();
	s.fmt = *f;
	s.frame_bytes = f->channels * 4;
	s.ring_bytes = RING_BYTES / s.frame_bytes * s.frame_bytes;
	memset(s.ring, 0, s.ring_bytes);
	clean_dcache((vm_offset_t)s.ring, s.ring_bytes, 0);
	s.wp = (f->kind == TWL_AUDIO_HBR ? hbr_rate : f->rate) * LEAD_MS / 1000 * s.frame_bytes;
	s.clear = s.last_hw = 0;
	s.played_bytes = 0;
	s.underruns = s.dropped_frames = 0;
	lanes = f->channels / 2;
	if (!path_on(f, lanes)) {
		return EIO;
	}
	h.rate = f->kind == TWL_AUDIO_HBR ? hbr_rate : f->rate;
	h.channels = (uint8_t)f->channels;
	h.hbr = f->kind == TWL_AUDIO_HBR;
	h.nonaudio = f->kind != TWL_AUDIO_PCM;
	h.ca = 0xff;
	h.spdif = a.spdif;
	if (!hakios_hdmitx_audio_start(&h)) {
		path_off();
		return EIO;
	}
	path_go();
	s.running = true;
	diag_ticks = 0;
	if (!s.armed) {
		uint64_t deadline;

		s.armed = true;
		clock_interval_to_deadline(TICK_MS, NSEC_PER_MSEC, &deadline);
		thread_call_enter_delayed(s.tick, deadline);
	}
	printf("hdmiaudio: playing: kind %u, %u Hz, %u ch, %u-bit, ring %u KiB\n", f->kind, f->rate, f->channels, f->bits, s.ring_bytes >> 10);
	return 0;
}

/* ---- /dev/twl-audio ---------------------------------------------------------------------------------------- */

static int
ta_open(__unused dev_t dev, __unused int flags, __unused int devtype, __unused struct proc *p)
{
	return a.level >= 3 ? 0 : ENXIO;
}

static int
ta_close(__unused dev_t dev, __unused int flags, __unused int devtype, __unused struct proc *p)
{
	lck_mtx_lock(&ha_lock);
	stream_stop_locked();
	lck_mtx_unlock(&ha_lock);
	return 0;
}

static int
ta_write(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	int error = 0;
	uint8_t chunk[4096];

	lck_mtx_lock(&ha_lock);
	if (!s.running) {
		lck_mtx_unlock(&ha_lock);
		return 0;                                       /* (no format yet: the HAL's first writes may come before it) */
	}
	while (uio_resid(uio) >= (user_ssize_t)(s.fmt.channels * (s.fmt.bits / 8))) {
		uint32_t fb = s.fmt.channels * (s.fmt.bits / 8), n = (uint32_t)MIN((user_ssize_t)sizeof(chunk), uio_resid(uio));

		n -= n % fb;
		error = uiomove((caddr_t)chunk, (int)n, uio);
		if (error) {
			break;
		}
		ring_write(chunk, n / fb);
	}
	lck_mtx_unlock(&ha_lock);
	return error;
}

static int
ta_ioctl(__unused dev_t dev, u_long command, caddr_t data, __unused int flags, __unused struct proc *p)
{
	int error = 0;

	lck_mtx_lock(&ha_lock);
	switch (command) {
	case TWL_AUDIO_SET_FORMAT:
		error = stream_start_locked((const struct twl_audio_format *)data);
		break;
	case TWL_AUDIO_STOP:
		error = stream_stop_locked();
		break;
	case TWL_AUDIO_SET_VOLUME:
		s.gain = *(uint32_t *)data > 65536 ? 65536 : *(uint32_t *)data;
		break;
	case TWL_AUDIO_GET_POSITION: {
		struct twl_audio_position *pos = (struct twl_audio_position *)data;

		memset(pos, 0, sizeof(*pos));
		if (s.running) {
			ring_service();
			pos->frames = s.played_bytes / s.frame_bytes;
			pos->mach_time = mach_absolute_time();
			pos->fill_frames = ((s.wp + s.ring_bytes - s.last_hw) % s.ring_bytes) / s.frame_bytes;
			pos->underruns = s.underruns;
			pos->dropped_frames = s.dropped_frames;
			pos->running = 1;
		}
		break;
	}
	case TWL_AUDIO_GET_CAPS: {
		struct twl_audio_caps *c = (struct twl_audio_caps *)data;
		unsigned count;
		uint8_t speakers = 0;
		const struct hakios_hdmitx_sad *sads = hakios_hdmitx_sads(&count, &speakers);

		memset(c, 0, sizeof(*c));
		c->count = MIN(count, TWL_AUDIO_SADS);
		c->speakers = speakers;
		for (uint32_t i = 0; i < c->count; i++) {
			c->sad[i].format = sads[i].format;
			c->sad[i].channels = sads[i].channels;
			c->sad[i].rates = sads[i].rates;
			c->sad[i].extra = sads[i].extra;
		}
		break;
	}
	default:
		error = ENOTTY;
	}
	lck_mtx_unlock(&ha_lock);
	return error;
}

static const struct cdevsw ta_cdevsw = {
	.d_open = ta_open, .d_close = ta_close, .d_read = eno_rdwrt, .d_write = ta_write,
	.d_ioctl = ta_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

/* ---- the test tone (hdmiaudio=2) --------------------------------------------------------------------------- */

static uint32_t tone_phase[8], tone_frames_left, tone_channels;
static thread_call_t tone_call;

/* sin(x) for x in 0..2^32 (a turn), -32767..32767: Bhaskara's approximation, good to 0.2%: a test tone does not need more */
static int32_t
sine(uint32_t turn)
{
	int64_t x = (turn >> 16) & 0x7fff;              /* a half turn in 15 bits */
	int64_t num = 16 * x * (32768 - x), den = 5LL * 32768 * 32768 - 4 * x * (32768 - x);
	int32_t v = (int32_t)(num * 32767 / den);

	return turn & 0x80000000u ? -v : v;
}

static void
tone_tick(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	int16_t frames[480 * 8];
	uint64_t deadline;
	uint32_t n = tone_frames_left < 480 ? tone_frames_left : 480;

	lck_mtx_lock(&ha_lock);
	for (uint32_t i = 0; i < n; i++) {
		for (uint32_t c = 0; c < tone_channels; c++) {
			uint32_t hz = 500 + 250 * c;

			tone_phase[c] += (uint32_t)(((uint64_t)hz << 32) / 48000);
			frames[i * tone_channels + c] = (int16_t)(sine(tone_phase[c]) / 11);        /* about -21 dBFS */
		}
	}
	if (n) {
		ring_write(frames, n);
	}
	tone_frames_left -= n;
	if (tone_frames_left == 0) {
		stream_stop_locked();
		printf("hdmiaudio: test tone done: underruns %u, dropped %u frames\n", s.underruns, s.dropped_frames);
		lck_mtx_unlock(&ha_lock);
		return;
	}
	lck_mtx_unlock(&ha_lock);
	clock_interval_to_deadline(10, NSEC_PER_MSEC, &deadline);
	thread_call_enter_delayed(tone_call, deadline);
}

/* ---- start ------------------------------------------------------------------------------------------------- */

/* hdmiaudio_delay=N: the whole of it N seconds later, from a thread of its own (the boot's first lines are gone from the log by then) */
extern void IOSleep(unsigned int milliseconds);
extern kern_return_t kernel_thread_start(thread_continue_t continuation, void *parameter, thread_t *new_thread);
static bool hdmiaudio_delayed;

static void
hdmiaudio_later(void *arg, wait_result_t result __unused)
{
	IOSleep((unsigned int)(uintptr_t)arg * 1000);
	hdmiaudio_delayed = true;
	hakios_hdmiaudio_init();
}

void
hakios_hdmiaudio_init(void)
{
	uint32_t level = 0, secs = 5, ch = 2, pll = 0, gate = 0, src = 4, delay = 0;
	int major;

	(void)PE_parse_boot_argn("hdmiaudio_diag", &diag_every, sizeof(diag_every));
	if (!PE_parse_boot_argn("hdmiaudio", &level, sizeof(level)) || level == 0) {
		return;
	}
	if (PE_parse_boot_argn("hdmiaudio_delay", &delay, sizeof(delay)) && delay != 0 && !hdmiaudio_delayed) {
		thread_t thread;

		/* the ring's physically contiguous memory now, while there is some: later in the boot there is none */
		if (level >= 2 && kmem_alloc_contig(kernel_map, (vm_offset_t *)&s.ring, RING_BYTES, PAGE_MASK, 0, 0,
		    KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
			s.ring = NULL;
		}

		if (kernel_thread_start(hdmiaudio_later, (void *)(uintptr_t)delay, &thread) == KERN_SUCCESS) {
			thread_deallocate(thread);
			return;
		}
	}
	PE_parse_boot_argn("hdmiaudio_secs", &secs, sizeof(secs));
	PE_parse_boot_argn("hdmiaudio_ch", &ch, sizeof(ch));
	PE_parse_boot_argn("hdmiaudio_pll", &pll, sizeof(pll));
	PE_parse_boot_argn("hdmiaudio_gate", &gate, sizeof(gate));
	PE_parse_boot_argn("hdmiaudio_mclk", &src, sizeof(src));
	{
		char path[16] = "";

		if (PE_parse_boot_argn("hdmiaudio_path", path, sizeof(path))) {
			a.spdif = strncmp(path, "spdif", 5) == 0;
		}
	}
	a.level = (int)level;
	a.mclk_src = src & 7;
	a.ana = ml_io_map(ANA_BASE, 0x1000);
	a.clk = ml_io_map(CLK_BASE, 0x1000);
	if (!a.ana || !a.clk) {
		printf("hdmiaudio: cannot map the clock registers\n");
		return;
	}
	printf("hdmiaudio: level %u, path %s\n", level, a.spdif ? "spdif (the bootloader's)" : "i2s (TDM B)");
	dump_plls();
	/* the audio registers hang the bus when the audio block's clock is closed: look first (a plain read of a clock register) */
	a.gate_ok = (*(volatile uint32_t *)(a.clk + SYS_CLK_EN0_REG1) & 1) != 0;
	if (!a.gate_ok && gate) {
		a.gate_ok = hakios_hdmitx_audio_bus_clock(true);
	}
	printf("hdmiaudio: audio bus clock (sys_audio) %s\n", a.gate_ok ? "open" : "CLOSED: the audio registers are not touched (hdmiaudio_gate=1 opens it)");
	hakios_hdmitx_audio_show();
	if (!a.gate_ok) {
		return;
	}
	a.aud = ml_io_map(AUD_BASE, 0x3000);
	if (!a.aud) {
		printf("hdmiaudio: cannot map the audio registers\n");
		return;
	}
	dump_state();
	if (level < 2) {
		return;
	}
	if (!hakios_hdmitx_audio_clock()) {
		printf("hdmiaudio: the transmitter's audio clock stays off\n");
		return;
	}
	if (pll) {
		pll_up(false);
	}
	if (s.ring == NULL && kmem_alloc_contig(kernel_map, (vm_offset_t *)&s.ring, RING_BYTES, PAGE_MASK, 0, 0,
	    KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("hdmiaudio: no memory for the ring\n");
		return;
	}
	s.phys = kvtophys((vm_offset_t)s.ring);
	if (s.phys + RING_BYTES > 0xffffffffull) {
		printf("hdmiaudio: the ring is above 4 GiB (%#llx): the FIFO addresses are 32 bits\n", s.phys);
		return;
	}
	s.tick = thread_call_allocate(ring_tick, NULL);
	if (level >= 3) {
		major = cdevsw_add(-1, &ta_cdevsw);
		if (major < 0 || devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "twl-audio") == NULL) {
			printf("hdmiaudio: cannot create /dev/twl-audio\n");
		}
		return;
	}
	/* the tone: stereo, or more channels, at 48 kHz, PCM, 16-bit */
	{
		struct twl_audio_format f = { TWL_AUDIO_PCM, 48000, ch == 4 || ch == 6 || ch == 8 ? ch : 2, 16 };
		int error;
		uint64_t deadline;

		tone_channels = f.channels;
		tone_frames_left = 48000 * (secs ? secs : 5);
		tone_call = thread_call_allocate(tone_tick, NULL);
		lck_mtx_lock(&ha_lock);
		error = stream_start_locked(&f);
		lck_mtx_unlock(&ha_lock);
		if (error == 0) {
			clock_interval_to_deadline(10, NSEC_PER_MSEC, &deadline);
			thread_call_enter_delayed(tone_call, deadline);
		} else {
			printf("hdmiaudio: the path would not start (%d)\n", error);
		}
	}
}

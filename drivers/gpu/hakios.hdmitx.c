/*
 * The Amlogic box's HDMI transmitter (S905X5M "S7D": Amlogic's hdmitx21, TMDS only), step one: what U-Boot left
 * on the wire, and the TV's EDID. Boot-arg hdmitx=1. Nothing is changed but the DDC master's own registers.
 *
 * The transmitter's two register blocks -- TOP (0xfe300000, 32-bit registers, reg * 4) and the core (COR,
 * 0xfe310000, 8-bit registers at byte offsets) -- belong to the secure world: Amlogic's Linux and U-Boot reach
 * them through the SiP calls 0x82000018 (read: address, width) and 0x82000019 (write: address, value, width). The
 * HDMI PLL and PHY (ANACTRL 0xfe008000), the clocks (0xfe000000), the pads (0xfe004000) and the VPU's encoder
 * (VCBUS 0xff000000, reg * 4) are plain registers. The sources are Amlogic's common_drivers
 * (drivers/media/vout/hdmitx21/hw: reg_ops.c, hdmi_tx_ddc.c, hdmi_tx_hw.c) and U-Boot's s7d/hdmitx_hw.c;
 * tvos/docs/hdmi.md has the register map as used here.
 *
 * The EDID is kept (hakios_hdmitx_edid()) for the framebuffer's mode list, and its CEC physical address
 * (hakios_hdmitx_physical_address()) for the CEC driver.
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

#define TOP_BASE        0xfe300000UL
#define COR_BASE        0xfe310000UL
#define ANA_BASE        0xfe008000UL
#define CLK_BASE        0xfe000000UL
#define PAD_BASE        0xfe004000UL
#define VPU_BASE        0xff000000UL

#define SIP_READ        0x82000018UL
#define SIP_WRITE       0x82000019UL

/* ANACTRL */
#define HDMIPLL_CTRL0   0x1c0
#define HDMIPLL_CTRL3   0x1cc
#define HDMIPHY_CTRL0   0x200
#define HDMIPHY_CTRL3   0x20c
#define HDMIPHY_CTRL5   0x214
/* CLKCTRL */
#define VID_PLL_CLK0_DIV 0xe4
#define ENC_HDMI_CLK_CTRL 0xdc
/* PADCTRL: GPIOH input levels (bit 2: HPD) */
#define GPIOH_I         0x140
/* VPU (VCBUS register numbers) */
#define ENCP_VIDEO_EN   0x1b80
#define ENCP_VIDEO_MAX_PXCNT 0x1b97
#define ENCP_VIDEO_MAX_LNCNT 0x1bae
#define ENCP_DE_H_BEGIN 0x1c3a
#define ENCP_DE_H_END   0x1c3b
#define ENCP_DE_V_BEGIN 0x1c3c
#define ENCP_DE_V_END   0x1c3d
#define VPU_HDMI_SETTING 0x271b
#define VPU_HDMI_FMT_CTRL 0x2743
#define VPP_OSD_SCI_WH_M1 0x1dc9
#define VPP_OSD_SCO_H   0x1dca
#define VPP_OSD_SCO_V   0x1dcb
#define OSD1_HDR2_CTRL  0x38a0
/* TOP (32-bit) */
#define TOP_INTR_STAT   0x010
#define TOP_TMDS_CLK_CNTL 0x030
#define TOP_HV_ACTIVE   0x08c
/* COR (8-bit) */
#define DDC_ADDR        0x003
#define DDC_SEGM        0x004
#define DDC_OFFSET      0x005
#define DDC_DIN_CNT1    0x006
#define DDC_DIN_CNT2    0x007
#define DDC_STATUS      0x008
#define DDC_CMD         0x009
#define DDC_DATA_AON    0x00a
#define DDC_DOUT_CNT    0x00b
#define P2T_CTRL        0x236
#define TPI_AVI_BYTE4   0x610
#define TPI_SC          0x61a
#define TPI_INFO_FSEL   0x6bf
#define TPI_INFO_B0     0x6c0
#define TPI_INFO_EN     0x6df
#define TPI_DDC_MASTER_EN 0x6f8
#define SCRCTL          0x900
/* TOP */
#define TOP_CLK_CNTL    0x004
/* COR: the audio block (Amlogic's *_IVCTX names) */
#define TPI_AVI_BYTE1   0x60d
#define PKT_FILTER_0    0x245
#define ACR_CTRL        0xa01
#define FREQ_SVAL       0xa02
#define N_SVAL1         0xa03
#define ACR_CTS_CLK_DIV 0xa0e
#define AUD_EN          0xa13
#define AUD_MODE        0xa14
#define SPDIF_SSMPL2    0xa19
#define SPDIF_ERTH      0xa1b
#define I2S_IN_MAP      0xa1c
#define I2S_IN_CTRL     0xa1d
#define I2S_CHST0       0xa1e
#define ASRC            0xa25
#define I2S_IN_SIZE     0xa26
#define AIP_RST         0xa2c
#define AIP_HDMI2MHL    0xa2d
#define AUDP_TXCTRL     0xa2f
#define TPI_AUD_CONFIG  0xa62
#define VP_INPUT_SYNC_ADJUST 0xb1c
#define INFOFRAME_AUDIO 2                       /* the generic packet buffer's slot of the audio infoframe */
/* CLKCTRL */
#define SYS_CLK_EN0_REG1 0x48                   /* bit 0: sys_audio, the audio block's bus clock */
#define HTX_CLK_CTRL1   0x120                   /* the HDMI TX audio clock: [10:9] source, [8] enable, [6:0] divider - 1 */

#define DDC_BUSLOW      (1u << 6)
#define DDC_NACK        (1u << 5)
#define DDC_INPROG      (1u << 4)

static struct {
	vm_offset_t ana, clk, pad, vpu;
	uint8_t edid[HAKIOS_HDMITX_EDID_MAX];
	unsigned edid_length;
	uint32_t physical_address;
	uint8_t vics[64];                       /* the video codes the TV lists */
	unsigned vic_count;
	uint32_t max_tmds_khz;                  /* the TV's (HF-VSDB, else the HDMI VSDB, else 165 MHz) */
	bool ycbcr420;                          /* takes 4:2:0 (a 4:2:0 capability map or 4:2:0-only codes) */
	bool pq, hlg, bt2020;                   /* ST 2084, HLG, BT.2020 YCC */
	struct hakios_hdmitx_sad sads[HAKIOS_HDMITX_SADS];      /* the audio formats the sink lists (CTA-861 audio data block) */
	unsigned sad_count;
	uint8_t speakers;                       /* its speaker allocation (CTA-861 speaker allocation data block, byte 1) */
	bool basic_audio;                       /* CTA-861 "basic audio": 2-channel LPCM at 32/44.1/48 kHz */
	struct hakios_hdmitx_timing timings[HAKIOS_HDMITX_TIMINGS_MAX];
	unsigned timing_count;
	char name[14];
} h = { .physical_address = 0xffff, .max_tmds_khz = 165000 };

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

static uint32_t top_rd(uint32_t reg) { return (uint32_t)sip(SIP_READ, TOP_BASE + reg, 32, 0); }
static void top_wr(uint32_t reg, uint32_t v) { (void)sip(SIP_WRITE, TOP_BASE + reg, v, 32); }
static uint8_t cor_rd(uint32_t off) { return (uint8_t)sip(SIP_READ, COR_BASE + off, 8, 0); }
static void cor_wr(uint32_t off, uint8_t v) { (void)sip(SIP_WRITE, COR_BASE + off, v, 8); }
static void cor_bits(uint32_t off, unsigned shift, unsigned width, unsigned v)
{
	uint8_t mask = (uint8_t)(((1u << width) - 1) << shift);

	cor_wr(off, (uint8_t)((cor_rd(off) & ~mask) | ((v << shift) & mask)));
}

static uint32_t rd(vm_offset_t base, uint32_t off) { return *(volatile uint32_t *)(base + off); }
static uint32_t vpu(uint32_t reg) { return rd(h.vpu, reg * 4); }

/* one infoframe slot of the transmitter's packet buffer: [HB0 HB1 HB2 checksum PB1..PB27], enabled or not */
static bool
infoframe(unsigned slot, uint8_t out[31])
{
	cor_wr(TPI_INFO_FSEL, (uint8_t)slot);
	for (unsigned i = 0; i < 31; i++) {
		out[i] = cor_rd(TPI_INFO_B0 + i);
	}
	return (cor_rd(TPI_INFO_EN) & 0x80) != 0;
}

static void
show_infoframe(const char *name, unsigned slot)
{
	uint8_t f[31];
	char hex[3 * 16 + 1];
	bool on = infoframe(slot, f);

	for (unsigned i = 0; i < 16; i++) {
		snprintf(hex + 3 * i, 4, " %02x", f[i]);
	}
	printf("hdmitx: %s infoframe (slot %u) %s:%s\n", name, slot, on ? "on" : "off", hex);
}

/* What is on the wire now: the PLL, the encoder's timing, what the core sends */
static void
show_state(void)
{
	uint32_t pll0 = rd(h.ana, HDMIPLL_CTRL0), pll3 = rd(h.ana, HDMIPLL_CTRL3), phy0 = rd(h.ana, HDMIPHY_CTRL0);
	uint32_t n = pll0 & 0x1ff, od0 = (pll0 >> 20) & 3, frac = pll3 & 0x1ffff, od2 = (pll3 >> 24) & 3;
	uint64_t vco_khz = 24000ull * n / 2 + (12000ull * frac >> 17);     /* N steps of 12 MHz, the fraction in 1/2^17 */
	uint32_t ph = vpu(ENCP_VIDEO_MAX_PXCNT) + 1, pv = vpu(ENCP_VIDEO_MAX_LNCNT) + 1;
	uint8_t p2t = cor_rd(P2T_CTRL), scr = cor_rd(SCRCTL);

	printf("hdmitx: PLL ctrl0 %#x ctrl3 %#x (N %u frac %#x od0 %u od2 %u, VCO %llu kHz, %s), PHY ctrl0 %#x ctrl3 %#x ctrl5 %#x\n",
	    pll0, pll3, n, frac, od0, od2, vco_khz, pll0 & (1u << 31) ? "locked" : "NOT locked", phy0, rd(h.ana, HDMIPHY_CTRL3),
	    rd(h.ana, HDMIPHY_CTRL5));
	printf("hdmitx: clocks: vid_pll_div %#x enc_hdmi %#x\n", rd(h.clk, VID_PLL_CLK0_DIV), rd(h.clk, ENC_HDMI_CLK_CTRL));
	printf("hdmitx: ENCP %s: total %ux%u, active h %u..%u v %u..%u; VPU_HDMI setting %#x fmt %#x\n",
	    vpu(ENCP_VIDEO_EN) & 1 ? "on" : "off", ph, pv, vpu(ENCP_DE_H_BEGIN), vpu(ENCP_DE_H_END), vpu(ENCP_DE_V_BEGIN),
	    vpu(ENCP_DE_V_END), vpu(VPU_HDMI_SETTING), vpu(VPU_HDMI_FMT_CTRL));
	printf("hdmitx: OSD scaler in %#x, out h %#x v %#x; OSD1 HDR core %#x\n", vpu(VPP_OSD_SCI_WH_M1), vpu(VPP_OSD_SCO_H),
	    vpu(VPP_OSD_SCO_V), vpu(OSD1_HDR2_CTRL));
	printf("hdmitx: core: AVI VIC %u, P2T %#x (%s), scrambling %s, %s mode, TOP tmds clock %#x hv active %#x, HPD %s\n",
	    cor_rd(TPI_AVI_BYTE4), p2t, p2t & 0x80 ? (p2t & 3) == 1 ? "10-bit" : (p2t & 3) == 2 ? "12-bit" : "deep ?" : "8-bit TMDS",
	    scr & 1 ? "on" : "off", cor_rd(TPI_SC) & 1 ? "HDMI" : "DVI", top_rd(TOP_TMDS_CLK_CNTL), top_rd(TOP_HV_ACTIVE),
	    rd(h.pad, GPIOH_I) & (1u << 2) ? "high (a TV)" : "low (nothing)");
	show_infoframe("AVI", 0);
	show_infoframe("vendor (DV/HDMI 1.4)", 5);
	show_infoframe("DRM (HDR)", 6);
	show_infoframe("HF vendor", 8);
}

/* One 128-byte EDID block through the core's DDC master (Amlogic's _hdmitx_ddcm_read_), E-DDC segment and all */
static bool
edid_block(unsigned block, uint8_t *out)
{
	unsigned got = 0, ms = 0;
	bool ok = true;

	for (ms = 0; cor_rd(DDC_STATUS) & DDC_INPROG; ms++) {
		if (ms == 20) {                                 /* a transfer left hanging: abort, reset the clock */
			cor_wr(TPI_DDC_MASTER_EN, cor_rd(TPI_DDC_MASTER_EN) | 0x80);
			cor_wr(DDC_CMD, 0x0f);
			cor_wr(DDC_CMD, 0x0a);
			cor_wr(TPI_DDC_MASTER_EN, cor_rd(TPI_DDC_MASTER_EN) & ~0x80);
			break;
		}
		IOSleep(1);
	}
	cor_wr(TPI_DDC_MASTER_EN, cor_rd(TPI_DDC_MASTER_EN) | 0x80);
	cor_wr(DDC_ADDR, 0xa0);
	cor_wr(DDC_SEGM, (uint8_t)(block >> 1));
	cor_wr(DDC_OFFSET, (uint8_t)(block * 128));
	cor_wr(DDC_CMD, 0x09);                                  /* clear the FIFO */
	cor_wr(DDC_DIN_CNT2, 0);
	cor_wr(DDC_DIN_CNT1, 128);
	cor_wr(DDC_CMD, 0x04);                                  /* enhanced DDC read */
	IOSleep(2);
	for (ms = 0; got < 128 && ms < 200;) {
		unsigned n = cor_rd(DDC_DOUT_CNT) & 0x1f;
		uint8_t status;

		if (n != 0) {
			while (n-- > 0 && got < 128) {
				out[got++] = cor_rd(DDC_DATA_AON);
			}
			continue;
		}
		status = cor_rd(DDC_STATUS);
		if (status & (DDC_NACK | DDC_BUSLOW)) {
			printf("hdmitx: EDID block %u: DDC status %#x after %u bytes\n", block, status, got);
			cor_wr(DDC_STATUS, 0);
			ok = false;
			break;
		}
		IOSleep(1);
		ms++;
	}
	if (!ok || got < 128) {
		cor_wr(DDC_CMD, 0x0f);
		cor_wr(DDC_CMD, 0x0a);
		ok = false;
	}
	cor_wr(TPI_DDC_MASTER_EN, cor_rd(TPI_DDC_MASTER_EN) & ~0x80);
	return ok;
}

static bool
checksum_ok(const uint8_t *b)
{
	uint8_t sum = 0;

	for (unsigned i = 0; i < 128; i++) {
		sum += b[i];
	}
	return sum == 0;
}

static void
dump_block(unsigned block, const uint8_t *b)
{
	for (unsigned row = 0; row < 8; row++) {
		char hex[3 * 16 + 1];

		for (unsigned i = 0; i < 16; i++) {
			snprintf(hex + 3 * i, 4, " %02x", b[row * 16 + i]);
		}
		printf("hdmitx: EDID %u+%02x:%s\n", block, row * 16, hex);
	}
}

/* A detailed timing descriptor: pixel clock and geometry */
static void
show_dtd(const uint8_t *d)
{
	unsigned clock = (d[0] | d[1] << 8) * 10;               /* kHz */
	unsigned ha = d[2] | (d[4] & 0xf0) << 4, hb = d[3] | (d[4] & 0x0f) << 8;
	unsigned va = d[5] | (d[7] & 0xf0) << 4, vb = d[6] | (d[7] & 0x0f) << 8;
	unsigned total = (ha + hb) * (va + vb);

	if (clock == 0) {
		return;
	}
	printf("hdmitx: EDID timing %ux%u%s, %u.%03u MHz, %u.%02u Hz\n", ha, va, d[17] & 0x80 ? "i" : "", clock / 1000, clock % 1000,
	    total ? (unsigned)(clock * 100000ull / total / 100) : 0, total ? (unsigned)(clock * 100000ull / total % 100) : 0);
}

/* CTA-861 audio format codes (short audio descriptor, byte 1 bits 6..3) */
static const char *
audio_format_name(unsigned format)
{
	static const char *const names[16] = {
		"?", "LPCM", "AC-3", "MPEG-1", "MP3", "MPEG-2", "AAC LC", "DTS", "ATRAC", "DSD", "E-AC-3 (DD+)", "DTS-HD",
		"MAT (TrueHD/Atmos)", "DST", "WMA Pro", "extension",
	};

	return names[format & 15];
}

/* What the sink says it plays: the short audio descriptors and its speakers. This is what a HAL device must offer. */
static void
show_audio(void)
{
	static const unsigned rate_hz[7] = { 32000, 44100, 48000, 88200, 96000, 176400, 192000 };
	char line[96];

	printf("hdmitx: sink audio: basic audio %u, %u descriptors, speaker allocation %#x\n", h.basic_audio, h.sad_count, h.speakers);
	for (unsigned i = 0; i < h.sad_count; i++) {
		const struct hakios_hdmitx_sad *sad = &h.sads[i];
		unsigned at = 0;

		line[0] = 0;
		for (unsigned r = 0; r < 7; r++) {
			if (sad->rates & (1u << r)) {
				at += snprintf(line + at, sizeof(line) - at, " %u", rate_hz[r] / 1000);
			}
		}
		printf("hdmitx: sink audio %u: %s, up to %u ch, kHz:%s, byte3 %#x%s\n", i, audio_format_name(sad->format),
		    sad->channels, line, sad->extra,
		    sad->format == 10 && (sad->extra & 1) ? " (JOC: Atmos in DD+)" : sad->format == 12 ? " (MAT)" : "");
	}
}

/* What the CTA-861 extension says: video codes, the HDMI VSDB (physical address), HF-VSDB, HDR static metadata */
static void
parse_cta(const uint8_t *b)
{
	unsigned dtd = b[2];
	char vics[200];
	unsigned at = 0;

	vics[0] = 0;
	printf("hdmitx: CTA-861 rev %u: underscan %u basic audio %u YCbCr 4:4:4 %u 4:2:2 %u\n", b[1], !!(b[3] & 0x80),
	    !!(b[3] & 0x40), !!(b[3] & 0x20), !!(b[3] & 0x10));
	for (unsigned i = 4; i < dtd && i < 127;) {
		unsigned tag = b[i] >> 5, len = b[i] & 0x1f;
		const uint8_t *p = b + i + 1;

		if (i + 1 + len > 127) {
			break;
		}
		if (tag == 1) {                                                         /* audio data block: short audio descriptors */
			for (unsigned k = 0; k + 2 < len && h.sad_count < HAKIOS_HDMITX_SADS; k += 3) {
				struct hakios_hdmitx_sad *sad = &h.sads[h.sad_count++];

				sad->format = p[k] >> 3 & 15;
				sad->channels = (uint8_t)((p[k] & 7) + 1);
				sad->rates = p[k + 1] & 0x7f;
				sad->extra = p[k + 2];
			}
		} else if (tag == 4 && len >= 3) {                                      /* speaker allocation */
			h.speakers = p[0];
		} else if (tag == 2) {                                                  /* video data block */
			for (unsigned k = 0; k < len && at < sizeof(vics) - 6; k++) {
				unsigned vic = p[k] >= 129 && p[k] <= 192 ? p[k] & 0x7f : p[k];

				if (h.vic_count < sizeof(h.vics)) {
					h.vics[h.vic_count++] = (uint8_t)vic;
				}

				at += snprintf(vics + at, sizeof(vics) - at, " %u%s", vic, p[k] >= 129 && p[k] <= 192 ? "*" : "");
			}
		} else if (tag == 3 && len >= 5 && p[0] == 0x03 && p[1] == 0x0c && p[2] == 0x00) {   /* HDMI 1.4 VSDB */
			h.physical_address = (uint32_t)p[3] << 8 | p[4];
			if (len >= 7 && p[6] * 5000u > h.max_tmds_khz) {
				h.max_tmds_khz = p[6] * 5000u;
			}
			printf("hdmitx: HDMI VSDB: physical address %x.%x.%x.%x, deep colour %#x, max TMDS %u MHz\n", p[3] >> 4,
			    p[3] & 15, p[4] >> 4, p[4] & 15, len >= 6 ? p[5] : 0, len >= 7 ? p[6] * 5 : 0);
		} else if (tag == 3 && len >= 6 && p[0] == 0xd8 && p[1] == 0x5d && p[2] == 0xc4) {   /* HDMI Forum VSDB */
			if (p[4] * 5000u > h.max_tmds_khz) {
				h.max_tmds_khz = p[4] * 5000u;
			}
			printf("hdmitx: HF-VSDB: max TMDS %u MHz, SCDC %u, 4:2:0 deep colour %#x\n", p[4] * 5, !!(p[5] & 0x80),
			    len >= 7 ? p[6] & 7 : 0);
		} else if (tag == 7 && len >= 3 && p[0] == 0x06) {                      /* HDR static metadata */
			h.pq = (p[1] & 4) != 0;
			h.hlg = (p[1] & 8) != 0;
			printf("hdmitx: HDR static metadata: EOTF SDR %u HDR-gamma %u PQ (HDR10) %u HLG %u, type 1 %u, max %u min %u\n",
			    p[1] & 1, !!(p[1] & 2), !!(p[1] & 4), !!(p[1] & 8), p[2] & 1, len >= 4 ? p[3] : 0, len >= 6 ? p[5] : 0);
		} else if (tag == 7 && len >= 1 && p[0] == 0x0e) {
			h.ycbcr420 = true;
			printf("hdmitx: YCbCr 4:2:0-only video data block (%u codes)\n", len - 1);
		} else if (tag == 7 && len >= 1 && p[0] == 0x0f) {
			h.ycbcr420 = true;
			printf("hdmitx: YCbCr 4:2:0 capability map (%u bytes)\n", len - 1);
		} else if (tag == 7 && len >= 2 && p[0] == 0x05) {
			h.bt2020 = (p[1] & 0x80) != 0;
			printf("hdmitx: colorimetry: BT.2020 YCC %u RGB %u\n", !!(p[1] & 0x80), !!(p[1] & 0x40));
		} else if (tag == 3 && len >= 3 && p[0] == 0x46 && p[1] == 0xd0 && p[2] == 0x00) {
			printf("hdmitx: Dolby Vision VSVDB (%u bytes)\n", len);
		}
		i += 1 + len;
	}
	h.basic_audio = (b[3] & 0x40) != 0;
	printf("hdmitx: video codes (* native):%s\n", vics);
	show_audio();
	for (unsigned i = dtd; dtd >= 4 && i + 18 <= 127; i += 18) {
		show_dtd(b + i);
	}
}

static void
read_edid(void)
{
	uint8_t *b = h.edid;
	unsigned blocks;
	char *name = h.name;

	if (!edid_block(0, b) || b[0] != 0x00 || b[1] != 0xff || !checksum_ok(b)) {
		printf("hdmitx: no EDID (block 0 %s)\n", b[1] == 0xff ? "has a bad checksum" : "unreadable");
		return;
	}
	blocks = 1 + b[126];
	if (blocks > HAKIOS_HDMITX_EDID_MAX / 128) {
		blocks = HAKIOS_HDMITX_EDID_MAX / 128;
	}
	for (unsigned i = 1; i < blocks; i++) {
		if (!edid_block(i, b + 128 * i) || !checksum_ok(b + 128 * i)) {
			printf("hdmitx: EDID block %u bad: %u blocks kept\n", i, i);
			blocks = i;
			break;
		}
	}
	h.edid_length = 128 * blocks;
	for (unsigned d = 54; d < 126; d += 18) {
		if (b[d] == 0 && b[d + 1] == 0 && b[d + 3] == 0xfc) {
			for (unsigned k = 0; k < 13 && b[d + 5 + k] != 0x0a; k++) {
				name[k] = (char)b[d + 5 + k];
			}
		}
	}
	printf("hdmitx: EDID %u blocks: \"%s\", maker %c%c%c product %#x, %u.%u, made %u week %u\n", blocks, name,
	    '@' + (b[8] >> 2 & 31), '@' + ((b[8] & 3) << 3 | b[9] >> 5), '@' + (b[9] & 31), b[10] | b[11] << 8, b[18], b[19],
	    1990 + b[17], b[16]);
	for (unsigned d = 54; d < 126; d += 18) {
		show_dtd(b + d);
	}
	for (unsigned i = 0; i < blocks; i++) {
		dump_block(i, b + 128 * i);
		if (i > 0 && b[128 * i] == 0x02) {
			parse_cta(b + 128 * i);
		}
	}
}

/*
 * The CTA-861 timings the box drives (Amlogic's hdmitx21 timing table; the 1/1.001 rates are the same timings with a
 * pixel clock 1000/1001 of theirs, which the PLL's fraction makes), the colour formats, and from the EDID which of
 * them this TV takes: its own video codes, the 1/1.001 twin of each code at 24, 30 or 60 Hz, and the TMDS rate it
 * allows (8-bit: the pixel clock; 10-bit x 5/4, 12-bit x 3/2; 4:2:2 is a 12-bit container at x 1; 4:2:0 halves).
 */
static const struct {
	uint8_t vic;
	uint16_t width, height, htotal, vtotal;
	uint32_t pixel_khz;
	uint8_t hz;
	bool fractional;                        /* has a 1/1.001 twin */
} cta[] = {
	{ 4, 1280, 720, 1650, 750, 74250, 60, true },
	{ 19, 1280, 720, 1980, 750, 74250, 50, false },
	{ 32, 1920, 1080, 2750, 1125, 74250, 24, true },
	{ 33, 1920, 1080, 2640, 1125, 74250, 25, false },
	{ 34, 1920, 1080, 2200, 1125, 74250, 30, true },
	{ 31, 1920, 1080, 2640, 1125, 148500, 50, false },
	{ 16, 1920, 1080, 2200, 1125, 148500, 60, true },
	{ 93, 3840, 2160, 5500, 2250, 297000, 24, true },
	{ 94, 3840, 2160, 5280, 2250, 297000, 25, false },
	{ 95, 3840, 2160, 4400, 2250, 297000, 30, true },
	{ 96, 3840, 2160, 5280, 2250, 594000, 50, false },
	{ 97, 3840, 2160, 4400, 2250, 594000, 60, true },
};

const struct hakios_hdmitx_color hakios_hdmitx_colors[HAKIOS_HDMITX_COLORS] = {
	/* id, encoding (0 RGB, 1 4:2:0, 2 4:2:2, 3 4:4:4), depth, colorimetry (1 BT.709, 7 BT.2020 YCC, 16 default RGB),
	 * EOTF (0 SDR, 2 PQ, 3 HLG), range (0 full, 1 limited) */
	{ 1, 0, 8, 16, 0, 0 },                  /* SDR RGB, full range */
	{ 2, 3, 8, 1, 0, 1 },                   /* SDR YCbCr 4:4:4 */
	{ 3, 2, 12, 1, 0, 1 },                  /* SDR YCbCr 4:2:2 12-bit */
	{ 4, 1, 8, 1, 0, 1 },                   /* SDR YCbCr 4:2:0 */
	{ 5, 2, 12, 7, 2, 1 },                  /* HDR10 YCbCr 4:2:2 12-bit */
	{ 6, 1, 10, 7, 2, 1 },                  /* HDR10 YCbCr 4:2:0 10-bit */
	{ 7, 3, 10, 7, 2, 1 },                  /* HDR10 YCbCr 4:4:4 10-bit */
	{ 8, 0, 10, 9, 2, 0 },                  /* HDR10 RGB 10-bit (9: BT.2020 RGB) */
};

static uint32_t
tmds_khz(uint32_t pixel_khz, const struct hakios_hdmitx_color *c)
{
	if (c->encoding == 2) {
		return pixel_khz;
	}
	pixel_khz = c->depth == 12 ? pixel_khz * 3 / 2 : c->depth == 10 ? pixel_khz * 5 / 4 : pixel_khz;
	return c->encoding == 1 ? pixel_khz / 2 : pixel_khz;
}

static bool
tv_has(unsigned vic)
{
	for (unsigned i = 0; i < h.vic_count; i++) {
		if (h.vics[i] == vic) {
			return true;
		}
	}
	return false;
}

static void
make_timings(void)
{
	h.timing_count = 0;
	for (unsigned i = 0; i < sizeof(cta) / sizeof(cta[0]); i++) {
		if (!tv_has(cta[i].vic)) {
			continue;
		}
		for (unsigned twin = 0; twin <= (cta[i].fractional ? 1u : 0u); twin++) {
			struct hakios_hdmitx_timing *t = &h.timings[h.timing_count];
			uint32_t colors = 0;

			if (h.timing_count == HAKIOS_HDMITX_TIMINGS_MAX) {
				break;
			}
			t->id = (uint16_t)(1 + h.timing_count);
			t->vic = cta[i].vic;
			t->width = cta[i].width;
			t->height = cta[i].height;
			t->htotal = cta[i].htotal;
			t->vtotal = cta[i].vtotal;
			t->fractional = twin != 0;
			t->pixel_khz = twin ? (uint32_t)((uint64_t)cta[i].pixel_khz * 1000 / 1001) : cta[i].pixel_khz;
			t->rate = twin ? (uint32_t)(((uint64_t)cta[i].hz << 16) * 1000 / 1001) : (uint32_t)cta[i].hz << 16;
			for (unsigned c = 0; c < HAKIOS_HDMITX_COLORS; c++) {
				const struct hakios_hdmitx_color *k = &hakios_hdmitx_colors[c];

				if (tmds_khz(cta[i].pixel_khz, k) > h.max_tmds_khz || tmds_khz(cta[i].pixel_khz, k) > 600000) {
					continue;
				}
				if (k->encoding == 1 && (!h.ycbcr420 || t->height < 2160 || cta[i].hz < 50)) {
					continue;               /* 4:2:0: only where 4:4:4 does not fit, as Apple's modes */
				}
				if (k->eotf == 2 && !(h.pq && h.bt2020)) {
					continue;
				}
				colors |= 1u << (k->id - 1);
			}
			t->colors = colors;
			h.timing_count++;
		}
	}
	for (unsigned i = 0; i < h.timing_count; i++) {
		const struct hakios_hdmitx_timing *t = &h.timings[i];

		printf("hdmitx: timing %u: %ux%u @ %u.%03u Hz (VIC %u%s), %u kHz, colours %#x\n", t->id, t->width, t->height,
		    t->rate >> 16, (unsigned)(((uint64_t)(t->rate & 0xffff) * 1000) >> 16), t->vic, t->fractional ? " /1.001" : "",
		    t->pixel_khz, t->colors);
	}
}

/*
 * Audio. The transmitter takes the audio block's I2S (TOHDMITX, see hakios.hdmiaudio.c) and sends it as audio sample
 * packets (or high-bit-rate packets), the audio clock regeneration packet (N and CTS) and the audio infoframe. The
 * sequence is Amlogic's hdmitx_set_audmode() (hdmitx21/hw/hdmi_tx_hw.c) for an I2S source, in the same order.
 * Nothing here runs unless hakios.hdmiaudio.c asks (hdmiaudio=2 or more).
 */
static const struct {
	uint32_t tmds_khz;
	uint16_t n[3];                          /* for 32, 44.1 and 48 kHz (88.2/96 are twice, 176.4/192 four times that) */
} acr_n[] = {
	{ 74176, { 11648, 17836, 11648 } }, { 74250, { 4096, 6272, 6144 } },
	{ 148352, { 11648, 8918, 5824 } }, { 148500, { 4096, 6272, 6144 } },
	{ 296703, { 5824, 4459, 5824 } }, { 297000, { 3072, 4707, 5120 } },
	{ 593407, { 5824, 8918, 5824 } }, { 594000, { 3072, 9408, 6144 } },
};

/* The TMDS character rate U-Boot (or our framebuffer driver) left, in kHz: the AVI frame's VIC, the colour depth, 4:2:0 */
static uint32_t
current_tmds_khz(bool *deep)
{
	uint8_t vic = cor_rd(TPI_AVI_BYTE4), p2t = cor_rd(P2T_CTRL), y = cor_rd(TPI_AVI_BYTE1) >> 5 & 3;
	uint32_t pixel = 0, frac = rd(h.ana, HDMIPLL_CTRL3) & 0x1ffff;

	for (unsigned i = 0; i < sizeof(cta) / sizeof(cta[0]); i++) {
		if (cta[i].vic == vic) {
			pixel = cta[i].pixel_khz;
			/* the 1/1.001 twin has a fractional PLL; a whole-rate mode leaves the fraction at 0 */
			if (cta[i].fractional && frac != 0) {
				pixel = (uint32_t)((uint64_t)pixel * 1000 / 1001);
			}
		}
	}
	*deep = (p2t & 0x80) != 0 && (p2t & 3) != 0;
	if (*deep) {
		pixel = (p2t & 3) == 1 ? pixel * 5 / 4 : pixel * 3 / 2;
	}
	return y == 3 ? pixel / 2 : pixel;
}

static uint32_t
acr_n_for(uint32_t rate, uint32_t tmds_khz, bool deep)
{
	unsigned base = rate % 11025 == 0 ? 1 : rate % 8000 == 0 ? 2 : 0;
	unsigned mult = rate >= 176400 ? 4 : rate >= 88200 ? 2 : 1;
	uint32_t n = base == 0 ? 4096 : base == 1 ? 6272 : 6144;

	if (!deep) {
		for (unsigned i = 0; i < sizeof(acr_n) / sizeof(acr_n[0]); i++) {
			if (tmds_khz >= acr_n[i].tmds_khz - 1 && tmds_khz <= acr_n[i].tmds_khz + 1) {
				n = acr_n[i].n[base];
			}
		}
	}
	/* (in deep colour the recommended N is another one; any N from 128 fs / 1500 to 128 fs / 300 is allowed: the default is) */
	return n * mult;
}

/* The audio infoframe (CTA-861: type 0x84, version 1, 10 bytes): channel count, channel allocation; the rest "refer to header" */
static void
send_audio_infoframe(unsigned channels, unsigned ca)
{
	uint8_t f[31];
	uint8_t sum = 0;

	memset(f, 0, sizeof(f));
	f[0] = 0x84;
	f[1] = 0x01;
	f[2] = 10;
	f[4] = (uint8_t)(channels - 1);                                 /* PB1: coding type 0, channel count */
	f[7] = (uint8_t)ca;                                             /* PB4 */
	for (unsigned i = 0; i < 31; i++) {
		sum += f[i];
	}
	f[3] = (uint8_t)(0 - sum);
	cor_wr(TPI_INFO_FSEL, INFOFRAME_AUDIO);
	for (unsigned i = 0; i < 31; i++) {
		cor_wr(TPI_INFO_B0 + i, f[i]);
	}
	cor_wr(TPI_INFO_EN, 0xe0);
}

static void
audio_mute(bool mute)
{
	cor_bits(AUD_EN, 0, 1, !mute);
	cor_bits(AUDP_TXCTRL, 7, 1, mute);
	cor_bits(TPI_AUD_CONFIG, 4, 1, mute);
}

void
hakios_hdmitx_audio_stop(void)
{
	if (h.ana == 0) {
		return;
	}
	audio_mute(true);
}

bool
hakios_hdmitx_audio_start(const struct hakios_hdmitx_audio *a)
{
	uint32_t tmds, n;
	bool deep;
	unsigned lines, ca;
	uint8_t chst[5] = { 0 };

	if (h.ana == 0 || a->channels < 2 || a->channels > 8 || (a->channels & 1)) {
		return false;
	}
	tmds = current_tmds_khz(&deep);
	if (tmds == 0) {
		printf("hdmitx: audio: no video mode on the wire (AVI VIC %u)\n", cor_rd(TPI_AVI_BYTE4));
		return false;
	}
	n = acr_n_for(a->rate, tmds, deep);
	lines = a->hbr ? 4 : a->channels / 2;
	ca = a->ca != 0xff ? a->ca : a->channels == 2 ? 0x00 : a->channels == 4 ? 0x03 : a->channels == 6 ? 0x0b : 0x13;
	/* consumer channel status, 4 bytes (Amlogic passes the same): byte 0 bit 1 = not audio, 1 = category general, 3 = sampling frequency */
	chst[0] = a->nonaudio ? 0x02 : 0x00;
	chst[1] = 0x01;
	chst[3] = a->rate >= 192000 ? 0x0e : a->rate >= 176400 ? 0x0c : a->rate >= 96000 ? 0x0a : a->rate >= 88200 ? 0x08 :
	    a->rate >= 48000 ? 0x02 : a->rate >= 44100 ? 0x00 : 0x03;
	if (a->nonaudio) {
		/* as Amlogic's iec_get_channel_status_info(): HBR streams say 768 kHz, original 192; E-AC-3 in 192 kHz frames and AC-3 original 48; 16-bit word */
		if (a->hbr) {
			chst[3] = 0x09;
			chst[4] = 0x02 | 0x10;
		} else {
			chst[4] = 0x02 | 0xd0;
		}
	}
	printf("hdmitx: audio: %u Hz, %u ch%s%s, TMDS %u kHz%s, N %u, %u lane(s)\n", a->rate, a->channels, a->hbr ? ", HBR" : "",
	    a->nonaudio ? ", IEC 61937" : "", tmds, deep ? " (deep colour)" : "", n, lines);

	cor_bits(AIP_RST, 0, 1, 1);
	cor_bits(SPDIF_SSMPL2, 5, 1, 0);
	top_wr(TOP_CLK_CNTL, (top_rd(TOP_CLK_CNTL) & ~(1u << 13)) | ((a->spdif ? 1u : 0u) << 13));    /* ACR measures i2s_mclk (0) or spdif_clk (1) */
	cor_wr(AIP_HDMI2MHL, 0x00);
	cor_wr(PKT_FILTER_0, 0x02);
	cor_wr(ASRC, 0x00);
	cor_wr(VP_INPUT_SYNC_ADJUST, 0x01);
	/* sample packet layout 1 for more than two channels of PCM, layout 0 for HBR and for two channels; start muted */
	cor_wr(AUDP_TXCTRL, (uint8_t)(0x80 | ((!a->hbr && a->channels > 2 && !a->nonaudio) ? 2 : 0)));
	cor_bits(ACR_CTS_CLK_DIV, 4, 1, 0);
	cor_wr(ACR_CTRL, 0x02);
	cor_wr(N_SVAL1, (uint8_t)n);
	cor_wr(N_SVAL1 + 1, (uint8_t)(n >> 8));
	cor_wr(N_SVAL1 + 2, (uint8_t)(n >> 16));
	cor_wr(FREQ_SVAL, 0);
	cor_wr(TPI_AUD_CONFIG, 3 << 6);
	if (!a->spdif) {
		cor_wr(I2S_IN_MAP, 0xe4);
		cor_wr(I2S_IN_CTRL, 0x20);
		cor_wr(I2S_IN_SIZE, 0x0b);
		for (unsigned i = 0; i < 5; i++) {
			cor_wr(I2S_CHST0 + i, chst[i]);
		}
	}
	cor_wr(SPDIF_ERTH, 0);
	cor_wr(AUD_MODE, a->spdif ? 0x02 : (uint8_t)(((1u << lines) - 1) << 4 | (a->hbr ? 1u << 2 : 0)));
	cor_wr(AUD_EN, 0x03);
	send_audio_infoframe(a->channels, ca);
	cor_bits(AIP_RST, 0, 1, 0);
	audio_mute(false);
	return true;
}

/* A look at what the transmitter's audio side holds now (read only) */
void
hakios_hdmitx_audio_show(void)
{
	uint8_t f[31];
	bool on;

	if (h.ana == 0) {
		return;
	}
	printf("hdmitx: audio regs: AUD_EN %#x AUD_MODE %#x AUDP_TXCTRL %#x ACR_CTRL %#x N %u TPI_AUD_CONFIG %#x I2S_IN_CTRL %#x "
	    "AIP_RST %#x CHST %02x %02x %02x %02x %02x\n", cor_rd(AUD_EN), cor_rd(AUD_MODE), cor_rd(AUDP_TXCTRL), cor_rd(ACR_CTRL),
	    cor_rd(N_SVAL1) | cor_rd(N_SVAL1 + 1) << 8 | cor_rd(N_SVAL1 + 2) << 16, cor_rd(TPI_AUD_CONFIG), cor_rd(I2S_IN_CTRL),
	    cor_rd(AIP_RST), cor_rd(I2S_CHST0), cor_rd(I2S_CHST0 + 1), cor_rd(I2S_CHST0 + 2), cor_rd(I2S_CHST0 + 3), cor_rd(I2S_CHST0 + 4));
	printf("hdmitx: audio ACR: CTS (set) %u CTS (measured?) %u, FREQ_SVAL %#x, AUD_MODE %#x, I2S_IN_MAP %#x\n",
	    cor_rd(0xa06) | cor_rd(0xa07) << 8 | cor_rd(0xa08) << 16, cor_rd(0xa09) | cor_rd(0xa0a) << 8 | cor_rd(0xa0b) << 16,
	    cor_rd(FREQ_SVAL), cor_rd(AUD_MODE), cor_rd(I2S_IN_MAP));
	printf("hdmitx: audio clocks: TOP clk cntl %#x, HTX_CLK_CTRL1 %#x (audio clock %s), sys_audio gate %s\n", top_rd(TOP_CLK_CNTL),
	    rd(h.clk, HTX_CLK_CTRL1), rd(h.clk, HTX_CLK_CTRL1) & 0x100 ? "on" : "off",
	    rd(h.clk, SYS_CLK_EN0_REG1) & 1 ? "open" : "closed");
	on = infoframe(INFOFRAME_AUDIO, f);
	printf("hdmitx: audio infoframe (slot %u) %s: %02x %02x %02x %02x %02x %02x %02x %02x\n", INFOFRAME_AUDIO, on ? "on" : "off",
	    f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7]);
}

/* The audio block's bus clock (CLKCTRL sys_audio): the audio registers hang the bus when it is closed. */
bool
hakios_hdmitx_audio_bus_clock(bool open)
{
	if (h.ana == 0) {
		return false;
	}
	if (open && !(rd(h.clk, SYS_CLK_EN0_REG1) & 1)) {
		*(volatile uint32_t *)(h.clk + SYS_CLK_EN0_REG1) |= 1;
	}
	return (rd(h.clk, SYS_CLK_EN0_REG1) & 1) != 0;
}

/* The audio clock of the transmitter itself (fclk_div5-ish, 200 MHz): U-Boot may have left it off */
bool
hakios_hdmitx_audio_clock(void)
{
	uint32_t v;

	if (h.ana == 0) {
		return false;
	}
	v = rd(h.clk, HTX_CLK_CTRL1);
	if (!(v & 0x100)) {
		v = (v & ~0x7fu & ~(3u << 9)) | 3u << 9 | 1;                  /* [10:9] = 3, divider 1 (÷2), as Amlogic's hdmitx21_set_audioclk() */
		*(volatile uint32_t *)(h.clk + HTX_CLK_CTRL1) = v | 0x100;
	}
	return (rd(h.clk, HTX_CLK_CTRL1) & 0x100) != 0;
}

const struct hakios_hdmitx_sad *
hakios_hdmitx_sads(unsigned *count, uint8_t *speakers)
{
	*count = h.sad_count;
	if (speakers) {
		*speakers = h.speakers;
	}
	return h.sad_count ? h.sads : NULL;
}

/* the sink lists a format (CTA-861 code, e.g. 1 LPCM, 2 AC-3, 10 E-AC-3, 12 MAT) with at least this many channels */
bool
hakios_hdmitx_sink_takes(unsigned format, unsigned channels, unsigned rate_bit)
{
	for (unsigned i = 0; i < h.sad_count; i++) {
		if (h.sads[i].format == format && h.sads[i].channels >= channels && (h.sads[i].rates & rate_bit)) {
			return true;
		}
	}
	return false;
}

const struct hakios_hdmitx_timing *
hakios_hdmitx_timings(unsigned *count)
{
	*count = h.timing_count;
	return h.timing_count ? h.timings : NULL;
}

const char *
hakios_hdmitx_tv(bool *pq, bool *hlg, bool *bt2020)
{
	*pq = h.pq;
	*hlg = h.hlg;
	*bt2020 = h.bt2020;
	return h.name;
}

const uint8_t *
hakios_hdmitx_edid(unsigned *length)
{
	*length = h.edid_length;
	return h.edid_length ? h.edid : NULL;
}

uint32_t
hakios_hdmitx_physical_address(void)
{
	return h.physical_address;
}

/* (the audio driver, when it is built, replaces this) */
__attribute__((weak)) void
hakios_hdmiaudio_init(void)
{
}

/*
 * SCDC (HDMI 2.0 status and control, I2C address 0xa8 on the DDC): above 340 MHz of TMDS clock -- 2160p60 -- the
 * sink has to be told that the source scrambles and sends the clock at 1/40 (TMDS_CONFIG, offset 0x20 = 0x03).
 * U-Boot tells it once; a TV that switches inputs, sleeps or is unplugged forgets it, and shows "no signal" for a
 * stream it can no longer descramble. Here a thread looks every second (and right after the hot-plug line rises
 * again) and tells it again when it has forgotten. Amlogic's hdmi_tx_ddc.c: _hdmitx_ddcm_write_, hdmitx_ddcm_read.
 */
#define SCDC_ADDR       0xa8
#define SCDC_TMDS_CFG   0x20
#define SCDC_STATUS0    0x40

static bool
ddc_idle(void)
{
	for (unsigned ms = 0; cor_rd(DDC_STATUS) & DDC_INPROG; ms++) {
		if (ms == 20) {
			cor_wr(TPI_DDC_MASTER_EN, cor_rd(TPI_DDC_MASTER_EN) | 0x80);
			cor_wr(DDC_CMD, 0x0f);
			cor_wr(DDC_CMD, 0x0a);
			cor_wr(TPI_DDC_MASTER_EN, cor_rd(TPI_DDC_MASTER_EN) & ~0x80);
			return false;
		}
		IOSleep(1);
	}
	return true;
}

static bool
scdc_read(uint8_t offset, uint8_t *value)
{
	bool ok = false;

	if (!ddc_idle()) {
		return false;
	}
	cor_wr(TPI_DDC_MASTER_EN, cor_rd(TPI_DDC_MASTER_EN) | 0x80);
	cor_wr(DDC_ADDR, SCDC_ADDR);
	cor_wr(DDC_SEGM, 0);
	cor_wr(DDC_OFFSET, offset);
	cor_wr(DDC_CMD, 0x09);                                  /* clear the FIFO */
	cor_wr(DDC_DIN_CNT2, 0);
	cor_wr(DDC_DIN_CNT1, 1);
	cor_wr(DDC_CMD, 0x02);                                  /* sequential read */
	for (unsigned ms = 0; ms < 20; ms++) {
		IOSleep(1);
		if (cor_rd(DDC_STATUS) & (DDC_NACK | DDC_BUSLOW)) {
			cor_wr(DDC_STATUS, 0);
			break;
		}
		if (cor_rd(DDC_DOUT_CNT) & 0x1f) {
			*value = cor_rd(DDC_DATA_AON);
			ok = true;
			break;
		}
	}
	if (!ok) {
		cor_wr(DDC_CMD, 0x0f);
		cor_wr(DDC_CMD, 0x0a);
	}
	cor_wr(TPI_DDC_MASTER_EN, cor_rd(TPI_DDC_MASTER_EN) & ~0x80);
	return ok;
}

static bool
scdc_write(uint8_t offset, uint8_t value)
{
	bool ok;

	if (!ddc_idle()) {
		return false;
	}
	cor_wr(TPI_DDC_MASTER_EN, cor_rd(TPI_DDC_MASTER_EN) | 0x80);
	cor_wr(DDC_ADDR, SCDC_ADDR);
	cor_wr(DDC_SEGM, 0);
	cor_wr(DDC_OFFSET, offset);
	cor_wr(DDC_CMD, 0x09 | 0x30);                           /* clear the FIFO */
	cor_wr(DDC_DIN_CNT2, 0);
	cor_wr(DDC_DIN_CNT1, 1);
	cor_wr(DDC_DATA_AON, value);
	cor_wr(DDC_CMD, 0x06 | 0x30);                           /* sequential write, acks ignored */
	IOSleep(3);
	ok = (cor_rd(DDC_STATUS) & (DDC_NACK | DDC_BUSLOW)) == 0;
	if (!ok) {
		cor_wr(DDC_STATUS, 0);
		cor_wr(DDC_CMD, 0x0f);
		cor_wr(DDC_CMD, 0x0a);
	}
	cor_wr(TPI_DDC_MASTER_EN, cor_rd(TPI_DDC_MASTER_EN) & ~0x80);
	return ok;
}

static void
scdc_watch(void *arg __unused, wait_result_t wr __unused)
{
	bool hpd_was = true;
	unsigned fixes = 0;

	for (;;) {
		bool hpd = (rd(h.pad, GPIOH_I) & (1u << 2)) != 0;
		uint8_t cfg = 0, st = 0;

		if (hpd != hpd_was) {
			printf("hdmitx: hot-plug %s\n", hpd ? "back: a TV again" : "low: the TV let go");
			hpd_was = hpd;
			if (hpd) {
				IOSleep(300);                   /* the sink's DDC wakes after the line */
			}
		}
		if (hpd && (cor_rd(SCRCTL) & 1) && scdc_read(SCDC_TMDS_CFG, &cfg) && (cfg & 3) != 3) {
			bool ok = scdc_write(SCDC_TMDS_CFG, 0x03);

			(void)scdc_read(SCDC_STATUS0, &st);
			printf("hdmitx: the TV had forgotten scrambling (TMDS_CONFIG %#x): told again, %s; status %#x (%u)\n",
			    cfg, ok ? "acked" : "NOT acked", st, ++fixes);
		}
		IOSleep(1000);
	}
}

static void
scdc_watch_start(void)
{
	uint32_t on = 1;
	thread_t thread;

	(void)PE_parse_boot_argn("hdmitx_scdc", &on, sizeof(on));
	if (on == 0) {
		return;
	}
	if (kernel_thread_start(scdc_watch, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

void
hakios_hdmitx_init(void)
{
	uint32_t on = 0;

	if (!PE_parse_boot_argn("hdmitx", &on, sizeof(on)) || on == 0) {
		return;
	}
	h.ana = ml_io_map(ANA_BASE, 0x1000);
	h.clk = ml_io_map(CLK_BASE, 0x1000);
	h.pad = ml_io_map(PAD_BASE, 0x1000);
	h.vpu = ml_io_map(VPU_BASE, 0x40000);
	if (!h.ana || !h.clk || !h.pad || !h.vpu) {
		printf("hdmitx: cannot map the registers\n");
		return;
	}
	show_state();
	read_edid();
	make_timings();
	scdc_watch_start();
	hakios_hdmiaudio_init();
	hakios_vdec_init();
}

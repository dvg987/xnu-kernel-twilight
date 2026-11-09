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
 *
 * Hot plug. U-Boot sets the output up only when the TV holds the hot-plug line high at power-on: with it low it
 * swaps outputmode for cvbsmode ("dummy_l", which this box has no encoder for) and HDMI is never set up -- no PLL,
 * PHY, encoder or infoframes, the TV says "no signal" for good. So a thread watches the line (GPIOH_2); once a TV
 * holds it high it reads the EDID again, hands the physical address to CEC, and sets the mode itself: U-Boot's
 * hdmitx21_set() for the S7D (TMDS only), in the same order, with the box's own values where Xiaomi's U-Boot differs
 * from Amlogic's (PHY swing). The mode is the one U-Boot left if there was one (its infoframes are kept and sent
 * again), else 2160p60 YCbCr 4:2:2 12-bit, U-Boot's own outputmode/colorattribute. Each time the line rises again
 * (TV off and on, cable replugged) the mode is set again, as Android's driver does. Boot-arg hdmitx_hotplug=0 turns
 * that off (the old behaviour: SCDC kept, nothing else).
 *
 * Debugging: sysctl kern.hakios_hdmitx_dump (the registers, tools/hdmitx-dump.py reads it; _bootdump is the one taken
 * at boot with hdmitx_bootdump=1) and kern.hakios_hdmitx_cmd (1 state to the log, 2 EDID again, 3 set the mode,
 * 4 output off, 5 CEC standby to all, 6 CEC "image view on" + active source, 7 SCDC status to the log, 8/9 the OSD
 * blended in YCbCr / RGB), kern.hakios_hdmitx_reg ("c OFF [VAL]" a COR byte, "t OFF [VAL]" a TOP word; ';' between).
 *
 * Colour: as the vendor's driver, the OSD is turned to YCbCr by the VPP's OSD matrices and blended in YCbCr, SDR, with
 * the Dolby Vision cores out of the path (boot-arg hdmitx_yuvblend=0: Xiaomi U-Boot's Dolby Vision path and HDR10).
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <sys/hakios.hdmitx.h>
#include <sys/hakios.cec.h>

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
#define VPP_OSD2_MATRIX 0x3920                  /* COEF00_01 .. PRE_OFFSET2 (13 words), then EN_CTRL */
#define VPP_WRAP_OSD1_MATRIX 0x3d60
#define VPP_POSTBLEND_H_SIZE 0x1d21
#define VIU_OSD1_BLK0_CFG_W3_ 0x1a13                    /* [28:16] the OSD's last line (as U-Boot leaves it) */
#define VIU_OSD1_BLK2_CFG_W4 0x1a15                     /* [11:0] stride in 16 bytes */
#define VIU_OSD1_BLK0_CFG_W1 0x1a1c                     /* x end << 16 | x start */
#define VIU_OSD1_BLK0_CFG_W2 0x1a1d                     /* y end << 16 | y start */
#define VIU_OSD1_BLK0_CFG_W3 0x1a1e
#define VIU_OSD_BLEND_DIN0_SCOPE_H 0x39b1
#define VIU_OSD_BLEND_DIN0_SCOPE_V 0x39b2
#define VIU_OSD_BLEND_BLEND0_SIZE 0x39bb
#define VIU_OSD_BLEND_BLEND1_SIZE 0x39bc
#define VPP_OSD1_IN_SIZE 0x1df1
#define VPP_OUT_H_V_SIZE 0x1da5
#define VPP_OSD_VSC_PHASE_STEP 0x1dc0
#define VPP_OSD_HSC_PHASE_STEP 0x1dc3
#define VPP_OSD1_BLD_H_SCOPE 0x1df5
#define VPP_OSD1_BLD_V_SCOPE 0x1df6
#define DOLBY_PATH_CTRL 0x1a0c                  /* [2] Dolby Vision core 2a (the OSD's) bypassed */
#define VPP_DOLBY_CTRL  0x1d93                  /* [3] Dolby Vision core 3 in the post-blend path */
#define DOLBY_CORE3_SWAP_CTRL1 0x36f2           /* [31:16] h total, [15:0] v total (sizes + Amlogic's margins) */
#define DOLBY_CORE3_SWAP_CTRL2 0x36f3           /* [31:16] width, [15:0] height */
#define DOLBY_CORE3_CTRL 0x3601                 /* output mode: 2 HDR10, 4 SDR 8-bit (U-Boot's with no TV) */
#define DOLBY_CORE3_MD_START 0x3602
#define DOLBY_CORE3_MD_END 0x3603
#define DOLBY_CORE3_DM 0x3606                   /* 26 display-management registers */
#define DOLBY_CORE3_SWAP_CTRL0 0x36f1
#define COR_SAFE_END    0xc00                   /* COR reads past this give 0xff or never return (0xe00) */
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

/* the mode set (U-Boot's hdmitx21_set(), arch/arm/mach-meson/s7d/hdmitx_clk.c, hdmitx_hw.c) */
#define PWR_BASE        0xfe00c000UL
#define PWR_MEM_PD11    0x06c
#define PAD_PIN_MUX_B   0x02c                   /* [3:0] hdmitx_sda, [7:4] hdmitx_scl: function 1 */
#define HDMIPLL_CTRL1   0x1c4
#define HDMIPLL_CTRL2   0x1c8
#define HDMIPHY_CTRL1   0x204
#define HDMIPHY_CTRL6   0x21c
#define SYS_CLK_EN0_REG2 0x04c                  /* [4] clk81_hdmitx_pclk, [3] hdcp pixel clock */
#define VID_CLK0_CTRL   0x0c0
#define VID_CLK0_CTRL2  0x0c4
#define VID_CLK0_DIV    0x0c8
#define HDMI_CLK_CTRL   0x0e0
#define HTX_CLK_CTRL0   0x11c
#define ENCI_VIDEO_EN   0x1b57
#define ENCP_VIDEO_SYNC_MODE 0x1b81
#define ENCP_VIDEO_MODE 0x1b8d
#define ENCP_VIDEO_MODE_ADV 0x1b8e
#define ENCP_VIDEO_HAVON_END 0x1ba3
#define ENCP_VIDEO_HAVON_BEGIN 0x1ba4
#define ENCP_VIDEO_VAVON_BLINE 0x1ba6
#define ENCP_VIDEO_HSO_BEGIN 0x1ba7
#define ENCP_VIDEO_HSO_END 0x1ba8
#define ENCP_VIDEO_VSO_BEGIN 0x1ba9
#define ENCP_VIDEO_VSO_END 0x1baa
#define ENCP_VIDEO_VSO_BLINE 0x1bab
#define ENCP_VIDEO_VSO_ELINE 0x1bac
#define ENCP_VIDEO_VAVON_ELINE 0x1baf
#define ENCP_DVI_HSO_BEGIN 0x1c30
#define ENCP_DVI_HSO_END 0x1c31
#define ENCP_DVI_VSO_BLINE_EVN 0x1c32
#define ENCP_DVI_VSO_ELINE_EVN 0x1c34
#define ENCP_DVI_VSO_BEGIN_EVN 0x1c36
#define ENCP_DVI_VSO_END_EVN 0x1c38
#define VPU_VENC_CTRL   0x1cef
#define VPU_VIU_VENC_MUX_CTRL 0x271a
#define VPU_HDMI_DITH_CNTL 0x27fc
/* TOP */
#define TOP_SW_RESET    0x000
#define TOP_HPD_FILTER  0x008
#define TOP_INTR_MASKN  0x00c
#define TOP_INTR_STAT_CLR 0x014
#define TOP_BIST_CNTL   0x018
#define TOP_TMDS_CLK_PTTN_01 0x028
#define TOP_TMDS_CLK_PTTN_23 0x02c
#define TOP_INFILTER    0x074
#define TOP_SEC_SCRATCH 0x07c
#define TOP_HS_INTR_CNTL 0x084
#define TOP_I2C_BUSY_CNT_MAX 0x0a0
#define TOP_SEC_VIDEO_OVR 0x0cc
#define TOP_HDCP14_MIN_SIZE 0x0d0
#define TOP_HDCP22_MIN_SIZE 0x0d4
/* COR */
#define INTR3_MASK      0x001
#define AON_CYP_CTL     0x018
#define SOC_FUNC_SEL    0x10b
#define CLKPWD          0x10d
#define PWD_SRST        0x110
#define PCLK2TMDS_MISC0 0x12a
#define PCLK2TMDS_MISC1 0x12b
#define CLK_DIV_CNTRL   0x12f
#define SYS_STAT        0x201
#define HDCP_CTRL       0x20f
#define CLKRATIO        0x235
#define HBLANK_REKEY_CONTROL 0x23c
#define HDPLL_FIX0      0x23d
#define FRL_LINK_RATE_CONFIG 0x29d
#define TEST_TXCTRL     0x2ca
#define SW_RST          0x319
#define PKT_AUTO_0      0x660
#define PKT_LOC_GEN     0x668
#define GCP_CNTL        0x6e1
#define GCP_AUTO_GEN    0x6e3
#define GCP_CUR_STAT    0x6e5
#define VTEM_CTRL       0x6ed
#define GEN5_CTRL       0x6f7
#define HT_DIG_CTL22_PHY 0x75e
#define HDCP2X_TX_SRST  0x8b5
#define VP_INPUT_MASK   0xb18
#define VP_OUTPUT_MAPPING 0xb44
#define VP_OUTPUT_MASK  0xb46
#define VP_CMS_CSC0_MULTI_CSC_CONFIG 0xd20
#define VP_CMS_CSC1_MULTI_CSC_CONFIG 0xda0
#define VP_CMS_CSC1_C444_C422_CONFIG 0xdc0

#define SIP_HDCPTX      0x820000abUL            /* HDCPTX_IOOPR */
#define HDCPTX_GET_RTERM 10                     /* HDMITX_GET_RTERM: the PHY's termination from the efuse, 0xff if none */

/* HDMI colour spaces (the AVI frame's Y): */
enum { CS_RGB = 0, CS_422 = 1, CS_444 = 2, CS_420 = 3 };

/* the mode on the wire (or to be set) */
struct txmode {
	uint8_t vic;
	uint8_t cs;                             /* CS_* */
	uint8_t depth;                          /* 8, 10, 12 (4:2:2: the 12-bit container) */
	bool fractional;                        /* the 1/1.001 rate */
	bool hdr;                               /* HDR10: BT.2020, PQ (Settings' HDR modes) */
};

#define DUMP_MAX        (20 * 1024)

static LCK_GRP_DECLARE(hdmitx_lck_grp, "hakios_hdmitx");
static LCK_MTX_DECLARE(hdmitx_lock, &hdmitx_lck_grp);  /* the core's registers: DDC, mode set, audio, dumps */

static struct {
	vm_offset_t ana, clk, pad, vpu, pwr;
	uint8_t edid[HAKIOS_HDMITX_EDID_MAX];
	unsigned edid_length;
	uint32_t physical_address;
	uint8_t vics[64];                       /* the video codes the TV lists */
	unsigned vic_count;
	uint32_t max_tmds_khz;                  /* the TV's (HF-VSDB, else the HDMI VSDB, else 165 MHz) */
	bool ycbcr420;                          /* takes 4:2:0 (a 4:2:0 capability map or 4:2:0-only codes) */
	bool ycc444, ycc422;                    /* CTA-861 header: takes YCbCr 4:4:4, 4:2:2 */
	bool hdmi;                              /* an HDMI VSDB (else a DVI sink: RGB, no infoframes) */
	unsigned max_lines;                     /* tests: act as if the TV went no higher than this (kern.hakios_hdmitx_mode "cap N") */
	bool pq, hlg, bt2020;                   /* ST 2084, HLG, BT.2020 YCC */
	struct hakios_hdmitx_sad sads[HAKIOS_HDMITX_SADS];      /* the audio formats the sink lists (CTA-861 audio data block) */
	unsigned sad_count;
	uint8_t speakers;                       /* its speaker allocation (CTA-861 speaker allocation data block, byte 1) */
	bool basic_audio;                       /* CTA-861 "basic audio": 2-channel LPCM at 32/44.1/48 kHz */
	struct hakios_hdmitx_timing timings[HAKIOS_HDMITX_TIMINGS_MAX];
	unsigned timing_count;
	char name[14];
	/* hot plug and the mode */
	struct txmode mode;                     /* what is (or is to be) on the wire */
	bool have_frames;                       /* frames[] hold U-Boot's infoframes, sent again with each mode set */
	uint8_t frames[11][31];                 /* packet buffer slots 0..10 */
	bool frame_on[11];
	uint32_t phy0, phy3, phy5;              /* U-Boot's PHY values for this mode (its swing), 0: ours */
	bool analog_path;                       /* U-Boot's ENCP clock came from the PLL's own pixel clock (VID_CLK0_CTRL[18:16] 3) */
	uint32_t max_lncnt;                     /* U-Boot's ENCP_VIDEO_MAX_LNCNT (it runs 2252 lines, not 2250), 0: v total - 1 */
	bool hdr10;                             /* boot-arg hdmitx_hdr (1): core 3 makes HDR10 (PQ, BT.2020), as U-Boot does with a TV */
	bool yuv_blend;                         /* boot-arg hdmitx_yuvblend (1): the OSD goes RGB->YCbCr in the VPU, the TX takes YCbCr */
	uint32_t dv_path, dv_ctrl;              /* U-Boot's DOLBY_PATH_CTRL, VPP_DOLBY_CTRL */
	struct hakios_hdmitx_audio audio;       /* the last audio set up, again after a mode set */
	bool audio_on;
	unsigned modesets;
	uint8_t dump[DUMP_MAX], bootdump[DUMP_MAX];
	unsigned dump_len, bootdump_len;
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

static void top_bits(uint32_t reg, unsigned shift, unsigned width, uint32_t v)
{
	uint32_t mask = (width == 32 ? ~0u : ((1u << width) - 1)) << shift;

	top_wr(reg, (top_rd(reg) & ~mask) | ((v << shift) & mask));
}

static uint32_t rd(vm_offset_t base, uint32_t off) { return *(volatile uint32_t *)(base + off); }
static void wr(vm_offset_t base, uint32_t off, uint32_t v) { *(volatile uint32_t *)(base + off) = v; }
static void bits(vm_offset_t base, uint32_t off, unsigned shift, unsigned width, uint32_t v)
{
	uint32_t mask = (width == 32 ? ~0u : ((1u << width) - 1)) << shift;

	wr(base, off, (rd(base, off) & ~mask) | ((v << shift) & mask));
}
static uint32_t vpu(uint32_t reg) { return rd(h.vpu, reg * 4); }
static void vpu_wr(uint32_t reg, uint32_t v) { wr(h.vpu, reg * 4, v); }
static void vpu_bits(uint32_t reg, unsigned shift, unsigned width, uint32_t v) { bits(h.vpu, reg * 4, shift, width, v); }
static bool hpd_high(void) { return (rd(h.pad, GPIOH_I) & (1u << 2)) != 0; }

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
	h.ycc444 = (b[3] & 0x20) != 0;
	h.ycc422 = (b[3] & 0x10) != 0;
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
			h.hdmi = true;
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

static bool
read_edid(void)
{
	uint8_t *b = h.edid;
	unsigned blocks;
	char *name = h.name;

	/* (again on each hot plug: what the last TV said goes) */
	h.edid_length = 0;
	h.physical_address = 0xffff;
	h.max_tmds_khz = 165000;
	h.vic_count = h.sad_count = h.timing_count = 0;
	h.ycbcr420 = h.pq = h.hlg = h.bt2020 = h.basic_audio = h.ycc444 = h.ycc422 = h.hdmi = false;
	h.speakers = 0;
	memset(h.name, 0, sizeof(h.name));
	if (!edid_block(0, b) || b[0] != 0x00 || b[1] != 0xff || !checksum_ok(b)) {
		printf("hdmitx: no EDID (block 0 %s)\n", b[1] == 0xff ? "has a bad checksum" : "unreadable");
		return false;
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
	return true;
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
	uint16_t hfront, hsync, vfront, vsync;  /* (all of them: both syncs positive) */
} cta[] = {
	{ 4, 1280, 720, 1650, 750, 74250, 60, true, 110, 40, 5, 5 },
	{ 19, 1280, 720, 1980, 750, 74250, 50, false, 440, 40, 5, 5 },
	{ 32, 1920, 1080, 2750, 1125, 74250, 24, true, 638, 44, 4, 5 },
	{ 33, 1920, 1080, 2640, 1125, 74250, 25, false, 528, 44, 4, 5 },
	{ 34, 1920, 1080, 2200, 1125, 74250, 30, true, 88, 44, 4, 5 },
	{ 31, 1920, 1080, 2640, 1125, 148500, 50, false, 528, 44, 4, 5 },
	{ 16, 1920, 1080, 2200, 1125, 148500, 60, true, 88, 44, 4, 5 },
	{ 93, 3840, 2160, 5500, 2250, 297000, 24, true, 1276, 88, 8, 10 },
	{ 94, 3840, 2160, 5280, 2250, 297000, 25, false, 1056, 88, 8, 10 },
	{ 95, 3840, 2160, 4400, 2250, 297000, 30, true, 176, 88, 8, 10 },
	{ 96, 3840, 2160, 5280, 2250, 594000, 50, false, 1056, 88, 8, 10 },
	{ 97, 3840, 2160, 4400, 2250, 594000, 60, true, 176, 88, 8, 10 },
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
		if (!tv_has(cta[i].vic) || (h.max_lines && cta[i].height > h.max_lines)) {
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
				if (k->encoding == 1 && (!h.ycbcr420 || t->height < 2160 || cta[i].hz < 50 || (h.ycc422 && h.hdmi))) {
					continue;               /* 4:2:0: only where 4:4:4 does not fit, as Apple's modes, and 4:2:2 is not taken */
				}
				if (k->eotf == 2 && !(h.pq && h.bt2020)) {
					continue;
				}
				if ((k->encoding == 3 && !h.ycc444) || (k->encoding == 2 && !h.ycc422) || (k->encoding != 0 && !h.hdmi)) {
					continue;               /* (what the CTA-861 header says; a DVI sink takes RGB only) */
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

/* the colour element of a mode (SDR) */
static unsigned
mode_color(const struct txmode *m)
{
	if (m->hdr) {
		return m->cs == CS_422 ? 5 : m->cs == CS_420 ? 6 : m->cs == CS_444 ? 7 : 8;
	}
	return m->cs == CS_RGB ? 1 : m->cs == CS_444 ? 2 : m->cs == CS_422 ? 3 : 4;
}

/* the TV lists this mode (timing, rate, colour) among the ones the box can drive */
static bool
mode_ok(const struct txmode *m)
{
	for (unsigned i = 0; i < h.timing_count; i++) {
		const struct hakios_hdmitx_timing *t = &h.timings[i];

		if (t->vic == m->vic && t->fractional == m->fractional && (t->colors & (1u << (mode_color(m) - 1)))) {
			return true;
		}
	}
	return false;
}

/*
 * The best mode the TV takes: the most lines, then 60 Hz, 50 Hz, then the highest other rate (whole rates before their
 * 1/1.001 twins); YCbCr 4:2:2 12-bit as U-Boot's colorattribute, else 4:4:4 8-bit, 4:2:0 8-bit, RGB. With no EDID:
 * 1080p60 YCbCr 4:4:4 8-bit, what nearly every TV takes. Returns false then.
 */
static bool
select_mode(struct txmode *m)
{
	static const uint8_t pref[] = { 3, 2, 4, 1 };
	const struct hakios_hdmitx_timing *best = NULL;
	unsigned best_score = 0, color = 0;

	for (unsigned i = 0; i < h.timing_count; i++) {
		const struct hakios_hdmitx_timing *t = &h.timings[i];
		unsigned hz = t->rate >> 16, score;

		if (!(t->colors & 0xf)) {
			continue;                               /* (no SDR colour fits) */
		}
		score = (unsigned)t->height << 12 | (hz == 60 ? 0xc00u : hz == 50 ? 0x800u : hz << 2) | (t->fractional ? 0u : 2u);
		if (score > best_score) {
			best_score = score;
			best = t;
		}
	}
	if (best == NULL) {
		m->hdr = false;
		m->vic = 16;
		m->cs = CS_444;
		m->depth = 8;
		m->fractional = false;
		return false;
	}
	for (unsigned k = 0; k < sizeof(pref); k++) {
		if (best->colors & (1u << (pref[k] - 1))) {
			color = pref[k];
			break;
		}
	}
	m->vic = best->vic;
	m->fractional = best->fractional;
	m->hdr = false;
	m->cs = color == 1 ? CS_RGB : color == 2 ? CS_444 : color == 3 ? CS_422 : CS_420;
	m->depth = color == 3 ? 12 : 8;
	return true;
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

	/* 2160p at 24, 25 and 30 Hz say VIC 0 in the AVI frame (make_avi: the vendor frame names them): the mode set's own */
	if (vic == 0 && h.mode.vic != 0) {
		vic = h.mode.vic;
	}

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
	lck_mtx_lock(&hdmitx_lock);
	h.audio_on = false;
	audio_mute(true);
	lck_mtx_unlock(&hdmitx_lock);
}

static bool audio_start_locked(const struct hakios_hdmitx_audio *a);

static bool output_alive(void);

bool
hakios_hdmitx_audio_start(const struct hakios_hdmitx_audio *a)
{
	bool ok;

	if (h.ana == 0) {
		return false;
	}
	lck_mtx_lock(&hdmitx_lock);
	if (!output_alive() && a->channels >= 2 && a->channels <= 8 && !(a->channels & 1)) {
		/* no TV yet (none at power-on): kept, and started by the mode set when one comes */
		printf("hdmitx: audio: %u Hz, %u ch kept until there is a picture on the wire\n", a->rate, a->channels);
		ok = true;
	} else {
		ok = audio_start_locked(a);
	}
	if (ok) {
		h.audio = *a;
		h.audio_on = true;
	}
	lck_mtx_unlock(&hdmitx_lock);
	return ok;
}

static bool
audio_start_locked(const struct hakios_hdmitx_audio *a)
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
	lck_mtx_lock(&hdmitx_lock);
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
	lck_mtx_unlock(&hdmitx_lock);
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

static void hp_request(int cmd);
static void vpp_output(int i);
static int cta_index(uint8_t vic);

/*
 * What the OSD scans: a frame of w x h, stride bytes a row (the framebuffer driver's flip, MiniIOMobileFramebuffer: the
 * window server draws 1280x720 frames in a 720p mode, 1920x1080 otherwise). Its window, the blend scopes, the OSD's input
 * to the VPP and the free scaler's input; the scaler's steps and output follow the mode (vpp_output). Called in the
 * blanking. The registers are the ones twloader's 720p trial scaled (VIU_OSD1_BLK0 0x1a13/0x1a1c-0x1a1e, BLK2 stride).
 */
void
hakios_hdmitx_osd_source(unsigned w, unsigned hgt, unsigned stride)
{
	int i;

	if (h.ana == 0 || w == 0 || hgt == 0 || (stride & 15)) {
		return;
	}
	vpu_bits(VIU_OSD1_BLK2_CFG_W4, 0, 12, stride / 16);
	vpu_wr(VIU_OSD1_BLK0_CFG_W3_, (hgt - 1) << 16);
	vpu_wr(VIU_OSD1_BLK0_CFG_W1, (w - 1) << 16);
	vpu_wr(VIU_OSD1_BLK0_CFG_W2, (hgt - 1) << 16);
	vpu_wr(VIU_OSD1_BLK0_CFG_W3, (w - 1) << 16);
	vpu_wr(VIU_OSD_BLEND_DIN0_SCOPE_H, (w - 1) << 16);
	vpu_wr(VIU_OSD_BLEND_DIN0_SCOPE_V, (hgt - 1) << 16);
	vpu_wr(VIU_OSD_BLEND_BLEND0_SIZE, hgt << 16 | w);
	vpu_wr(VIU_OSD_BLEND_BLEND1_SIZE, hgt << 16 | w);
	vpu_wr(VPP_OSD1_IN_SIZE, hgt << 16 | w);
	vpu_wr(VPP_OSD_SCI_WH_M1, (w - 1) << 16 | (hgt - 1));
	i = cta_index(h.mode.vic);
	if (i >= 0) {
		vpp_output(i);
	}
}

/* the picture's height on the wire (0: none) */
unsigned
hakios_hdmitx_output_height(void)
{
	int i = h.ana != 0 && h.mode.vic != 0 ? cta_index(h.mode.vic) : -1;

	return i < 0 ? 0 : cta[i].height;
}

/* one frame on the wire, in ns (0: no output): the mode's own, 41.7 ms at 23.976 Hz */
uint64_t
hakios_hdmitx_frame_ns(void)
{
	int i = h.ana != 0 && h.mode.vic != 0 ? cta_index(h.mode.vic) : -1;

	if (i < 0) {
		return 0;
	}
	uint64_t ns = (uint64_t)cta[i].htotal * cta[i].vtotal * 1000000ull / cta[i].pixel_khz;
	return h.mode.fractional ? ns * 1001 / 1000 : ns;
}

unsigned
hakios_hdmitx_output_width(void)
{
	int i = h.ana != 0 && h.mode.vic != 0 ? cta_index(h.mode.vic) : -1;

	return i < 0 ? 0 : cta[i].width;
}

/*
 * How many colour elements Settings gets: the four SDR ones, HDR10 too with boot-arg hdmitx_hdrmodes=1. tvOS treats this
 * display as a built-in panel, whose modes are all "fixed" SDR 8-bit: it cannot list HDR apart and took an HDR10 element
 * for "4K SDR" (picture wrong). HDR needs the display to be a TV out (IOMobileFramebuffer.framework: "AppleTV"/"external";
 * backboardd then needs the IOAV side too: with either property alone the screen stayed black).
 */
static uint32_t hdr_modes = 1;

unsigned
hakios_hdmitx_color_count(void)
{
	return hdr_modes ? HAKIOS_HDMITX_COLORS : 4;
}

/* the output's mode as Settings sees it: its timing and colour element (0, 0 with no output or a mode not in the list) */
void
hakios_hdmitx_current(unsigned *timing_id, unsigned *color_id)
{
	*timing_id = *color_id = 0;
	for (unsigned i = 0; i < h.timing_count; i++) {
		if (h.timings[i].vic == h.mode.vic && h.timings[i].fractional == h.mode.fractional) {
			*timing_id = h.timings[i].id;
			*color_id = mode_color(&h.mode);
			return;
		}
	}
}

static void (*listener)(void);

/* called (from the hot-plug thread) after each EDID read and each mode set: the list or the current mode changed */
void
hakios_hdmitx_set_listener(void (*fn)(void))
{
	listener = fn;
}

static void
tell_listener(void)
{
	if (listener != NULL) {
		listener();
	}
}

/* Settings' choice (IOMobileFramebufferSetDigitalOutMode), SDR or HDR10. The mode set runs on the hot-plug thread. */
bool
hakios_hdmitx_set_output(unsigned timing_id, unsigned color_id)
{
	const struct hakios_hdmitx_timing *t = NULL;
	struct txmode m;

	if (h.ana == 0 || color_id < 1 || color_id > hakios_hdmitx_color_count()) {
		return false;
	}
	lck_mtx_lock(&hdmitx_lock);
	for (unsigned i = 0; i < h.timing_count; i++) {
		if (h.timings[i].id == timing_id) {
			t = &h.timings[i];
		}
	}
	if (t == NULL || !(t->colors & (1u << (color_id - 1)))) {
		lck_mtx_unlock(&hdmitx_lock);
		printf("hdmitx: Settings asked for timing %u colour %u: not one this TV takes\n", timing_id, color_id);
		return false;
	}
	m.vic = t->vic;
	m.fractional = t->fractional;
	{
		const struct hakios_hdmitx_color *k = &hakios_hdmitx_colors[color_id - 1];

		m.cs = k->encoding == 0 ? CS_RGB : k->encoding == 3 ? CS_444 : k->encoding == 2 ? CS_422 : CS_420;
		m.depth = k->depth;
		m.hdr = k->eotf == 2;
	}
	h.mode = m;
	h.have_frames = false;
	h.phy0 = 0;
	h.max_lncnt = 0;
	lck_mtx_unlock(&hdmitx_lock);
	printf("hdmitx: Settings: %ux%u%s, VIC %u, colour %u%s\n", t->width, t->height, t->fractional ? " /1.001" : "", t->vic, color_id,
	    m.hdr ? " (HDR10)" : "");
	hp_request(3);
	return true;
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


/* ---------------------------------------------------------------------------------------------------------------- */
/* The mode set: U-Boot's hdmitx21_set() for the S7D, TMDS (no FRL, no DSC), in its order                            */
/* ---------------------------------------------------------------------------------------------------------------- */

static int
cta_index(uint8_t vic)
{
	for (unsigned i = 0; i < sizeof(cta) / sizeof(cta[0]); i++) {
		if (cta[i].vic == vic) {
			return (int)i;
		}
	}
	return -1;
}

/* The output runs: the PLL locked, the encoder on, the PHY driving */
static bool
output_alive(void)
{
	return (rd(h.ana, HDMIPLL_CTRL0) & (1u << 31)) && (vpu(ENCP_VIDEO_EN) & 1) && rd(h.ana, HDMIPHY_CTRL0) != 0;
}

static uint32_t
mode_tmds_khz(const struct txmode *m)
{
	int i = cta_index(m->vic);
	uint32_t pixel;

	if (i < 0) {
		return 0;
	}
	pixel = cta[i].pixel_khz;
	if (m->fractional) {
		pixel = (uint32_t)((uint64_t)pixel * 1000 / 1001);
	}
	if (m->cs != CS_422) {
		pixel = m->depth == 12 ? pixel * 3 / 2 : m->depth == 10 ? pixel * 5 / 4 : pixel;
	}
	return m->cs == CS_420 ? pixel / 2 : pixel;
}

/* The VID PLL divider after the HDMI PLL (set_tmds_vid_clk_div): 5 for 8-bit / 4:2:2, 6.25 for 10-bit, 7.5 for 12-bit */
static void
vid_pll_div(unsigned depth, bool ycc422)
{
	uint32_t shift_val, shift_sel;

	if (ycc422 || depth == 8) {
		shift_val = 0x0c63;
		shift_sel = 2;
	} else if (depth == 10) {
		shift_val = 0x0000;
		shift_sel = 3;
	} else {
		shift_val = 0x78f0;
		shift_sel = 2;
	}
	bits(h.clk, VID_PLL_CLK0_DIV, 18, 2, 0);
	bits(h.clk, VID_PLL_CLK0_DIV, 15, 1, 0);
	bits(h.clk, VID_PLL_CLK0_DIV, 16, 2, 0);
	bits(h.clk, VID_PLL_CLK0_DIV, 0, 15, 0);
	bits(h.clk, VID_PLL_CLK0_DIV, 16, 2, shift_sel);
	bits(h.clk, VID_PLL_CLK0_DIV, 15, 1, 1);
	bits(h.clk, VID_PLL_CLK0_DIV, 0, 15, shift_val);
	bits(h.clk, VID_PLL_CLK0_DIV, 15, 1, 0);
	bits(h.clk, VID_PLL_CLK0_DIV, 19, 1, 1);
}

/* hdmitx21_set_clk(): the HDMI PLL (set_s7d_htxpll_clk_other + set21_s7d_htxpll_clk_out), then the VID PLL divider */
static bool
pll_set(const struct txmode *m, bool analog_path)
{
	int i = cta_index(m->vic);
	uint32_t base, vco, div = 1, quotient, remainder, od00 = 0, od01 = 0, od20 = 0, od21, od1 = 0;
	unsigned tries;

	wr(h.ana, HDMIPLL_CTRL0, 0);
	wr(h.ana, HDMIPLL_CTRL3, 0);
	base = cta[i].pixel_khz;
	if (m->cs != CS_422) {
		base = m->depth == 12 ? base * 3 / 2 : m->depth == 10 ? base * 5 / 4 : base;
	}
	if (m->fractional) {
		base -= base / 1001;
	}
	base *= 10;
	if (m->cs == CS_420) {
		base /= 2;
	}
	for (vco = base; !(vco >= 3000000 && vco < 6000000) && div <= 32;) {
		div *= 2;
		vco *= 2;
	}
	if (vco < 3000000 || vco >= 6000000) {
		printf("hdmitx: no PLL setting for %u kHz\n", base);
		return false;
	}
	quotient = vco / 12000;
	remainder = (vco - quotient * 12000) * (1u << 17) / 12000;
	wr(h.ana, HDMIPLL_CTRL0, 0x00017000 | quotient);
	wr(h.ana, HDMIPLL_CTRL1, 0x9040137d);
	wr(h.ana, HDMIPLL_CTRL2, 0x04000000);
	wr(h.ana, HDMIPLL_CTRL3, 0x01160000 | remainder);
	bits(h.ana, HDMIPLL_CTRL0, 28, 1, 1);
	IODelay(10);
	bits(h.ana, HDMIPLL_CTRL0, 18, 1, 1);
	IODelay(10);
	bits(h.ana, HDMIPLL_CTRL0, 30, 1, 1);
	bits(h.ana, HDMIPLL_CTRL0, 18, 1, 0);
	IODelay(80);
	bits(h.ana, HDMIPLL_CTRL0, 29, 1, 1);
	IODelay(80);
	for (tries = 0; tries < 10; tries++) {
		IODelay(50);
		if (rd(h.ana, HDMIPLL_CTRL0) & (1u << 31)) {
			break;
		}
		bits(h.ana, HDMIPLL_CTRL0, 30, 1, 1);
		bits(h.ana, HDMIPLL_CTRL0, 30, 1, 0);
	}
	/* the output dividers: od0 (two stages of 2/4/8), od2 (to the PHY, and clk_out2), od1 (the deep colour ratio) */
	if (div % 8 == 0) { od00 = 3; div /= 8; } else if (div % 4 == 0) { od00 = 2; div /= 4; } else if (div % 2 == 0) { od00 = 1; div /= 2; }
	if (div % 8 == 0) { od01 = 3; div /= 8; } else if (div % 4 == 0) { od01 = 2; div /= 4; } else if (div % 2 == 0) { od01 = 1; div /= 2; }
	if (div % 4 == 0) { od20 = 2; div /= 4; } else if (div % 2 == 0) { od20 = 1; div /= 2; }
	od21 = m->cs == CS_420 ? od20 : od20 + 1;
	if (m->cs != CS_422) {
		od1 = m->depth == 12 ? 2 : m->depth == 10 ? 1 : 0;
	}
	bits(h.ana, HDMIPLL_CTRL3, 19, 1, 1);
	if (analog_path) {
		bits(h.ana, HDMIPLL_CTRL3, 22, 2, od1);
	}
	bits(h.ana, HDMIPLL_CTRL3, 24, 6, od20 << 3 | od21);
	bits(h.ana, HDMIPLL_CTRL0, 20, 6, od01 << 3 | od00);
	if (m->depth == 8 && (m->vic == 16 || m->vic == 31 || m->vic == 4 || m->vic == 19)) {
		bits(h.ana, HDMIPLL_CTRL3, 20, 2, 3);                   /* spread spectrum, 1000 ppm */
		bits(h.ana, HDMIPLL_CTRL2, 16, 4, 2);
		bits(h.ana, HDMIPLL_CTRL2, 4, 1, 1);
	}
	if (!analog_path) {
		vid_pll_div(m->depth, m->cs == CS_422);
	}
	printf("hdmitx: PLL VCO %u kHz (N %u frac %#x) / %u: ctrl0 %#x ctrl3 %#x, %s after %u resets\n", vco, quotient, remainder,
	    vco / (base ? base : 1), rd(h.ana, HDMIPLL_CTRL0), rd(h.ana, HDMIPLL_CTRL3),
	    rd(h.ana, HDMIPLL_CTRL0) & (1u << 31) ? "locked" : "NOT locked", tries);
	return (rd(h.ana, HDMIPLL_CTRL0) & (1u << 31)) != 0;
}

/* hdmi_hwp_init(): the DDC pins, the transmitter's fixed clocks, out of reset, interrupts, E-DDC, GCP, TPI */
static void
hw_init(void)
{
	bits(h.pad, PAD_PIN_MUX_B, 0, 4, 1);
	bits(h.pad, PAD_PIN_MUX_B, 4, 4, 1);
	/* hdmitx21_set_default_clk() */
	bits(h.clk, SYS_CLK_EN0_REG2, 4, 1, 1);
	wr(h.clk, HDMI_CLK_CTRL, 3u << 9 | 1);                         /* cts_hdmitx_sys_clk: fclk_div5 / 2 */
	wr(h.clk, HDMI_CLK_CTRL, 3u << 9 | 1u << 8 | 1);
	wr(h.clk, HTX_CLK_CTRL0, 3u << 25 | 1u << 16 | 3u << 9 | 1);   /* 200m (/2 = 25 MHz? as theirs) and prif clocks */
	wr(h.clk, HTX_CLK_CTRL0, 3u << 25 | 1u << 24 | 1u << 16 | 3u << 9 | 1u << 8 | 1);
	bits(h.pwr, PWR_MEM_PD11, 8, 8, 0);
	top_wr(TOP_SW_RESET, 0);
	cor_wr(INTR3_MASK, 0xff);
	top_wr(TOP_SEC_SCRATCH, 1);
	top_wr(TOP_INTR_STAT_CLR, 0x1ff);
	top_wr(TOP_CLK_CNTL, 1u << 13 | 1u << 3 | 1u << 2 | 1u << 1 | 1u << 0);
	top_wr(TOP_INTR_MASKN, top_rd(TOP_INTR_MASKN) | 1u << 2 | 1u << 1);
	top_wr(TOP_INFILTER, 1u << 24);
	cor_wr(AON_CYP_CTL, 2);
	cor_bits(PCLK2TMDS_MISC0, 0, 2, 0);
	cor_bits(HBLANK_REKEY_CONTROL, 6, 1, 1);
	cor_bits(HDPLL_FIX0, 0, 1, 0);
	cor_bits(GCP_CNTL, 0, 1, 1);
	cor_bits(GCP_AUTO_GEN, 2, 2, 2);                                /* clear AV mute */
	cor_bits(TPI_SC, 3, 1, 0);
	cor_bits(TPI_SC, 7, 1, 1);
}

/* the video clock tree: vid_pll (or the PLL's own pixel clock) -> enc0 -> ENCP, the transmitter's pixel/fe/pnx clocks */
static void
clock_tree(bool analog_path)
{
	/* _hdmitx21_set_clk(): set_vid_clk_div(1), set_hdmi_tx_pixel_div(1), set_encp_div(0), the ENCP clocks, fe clock */
	bits(h.clk, VID_CLK0_CTRL, 16, 3, 0);
	bits(h.clk, VID_CLK0_DIV, 0, 8, 0);
	bits(h.clk, VID_CLK0_CTRL, 0, 3, 7);
	if (analog_path) {
		bits(h.clk, VID_CLK0_CTRL, 16, 3, 3);
	}
	bits(h.clk, VID_CLK0_CTRL2, 5, 1, 1);
	bits(h.clk, HDMI_CLK_CTRL, 16, 4, 0);
	bits(h.clk, VID_CLK0_DIV, 24, 4, 0);
	bits(h.clk, VID_CLK0_CTRL, 19, 1, 1);
	bits(h.clk, VID_CLK0_CTRL2, 2, 1, 1);
	bits(h.clk, VID_CLK0_CTRL2, 3, 1, 1);
	bits(h.clk, VID_CLK0_CTRL2, 5, 1, 1);
	bits(h.clk, VID_CLK0_CTRL2, 9, 1, 1);
	bits(h.clk, VID_CLK0_CTRL2, 10, 1, 1);
	bits(h.clk, HTX_CLK_CTRL1, 24, 1, 1);
	bits(h.clk, VID_CLK0_CTRL2, 9, 1, 1);
	bits(h.clk, HDMI_CLK_CTRL, 20, 4, (rd(h.clk, VID_CLK0_DIV) >> 24) & 0xf);
}

/* config_tv_enc_calc(): ENCP's timing from the CTA timing (the DE window, syncs; the VPP fetch 2 pixels ahead) */
static void
encp_timing(int i, bool y420)
{
	const uint32_t hsync_st = 5, vsync_st = 1, latency = 2;
	uint32_t de_h_end = cta[i].htotal - (cta[i].hfront - hsync_st), de_h_begin = de_h_end - cta[i].width;
	uint32_t de_v_end = cta[i].vtotal - (cta[i].vfront - vsync_st), de_v_begin = de_v_end - cta[i].height;

	vpu_wr(ENCP_DVI_HSO_BEGIN, hsync_st);
	vpu_wr(ENCP_DVI_HSO_END, hsync_st + cta[i].hsync);
	vpu_wr(ENCP_DVI_VSO_BLINE_EVN, vsync_st + y420);
	vpu_wr(ENCP_DVI_VSO_ELINE_EVN, vsync_st + cta[i].vsync + y420);
	vpu_wr(ENCP_DVI_VSO_BEGIN_EVN, hsync_st);
	vpu_wr(ENCP_DVI_VSO_END_EVN, hsync_st);
	vpu_wr(ENCP_DE_H_BEGIN, de_h_begin);
	vpu_wr(ENCP_DE_H_END, de_h_end);
	vpu_wr(ENCP_DE_V_BEGIN, de_v_begin);
	vpu_wr(ENCP_DE_V_END, de_v_end);
	vpu_wr(ENCP_VIDEO_MODE, 0x0040 | 1u << 14);
	vpu_wr(ENCP_VIDEO_MODE_ADV, 0x18);
	vpu_wr(ENCP_VIDEO_HAVON_BEGIN, de_h_begin - latency);
	vpu_wr(ENCP_VIDEO_HAVON_END, de_h_end - latency - 1);
	vpu_wr(ENCP_VIDEO_VAVON_BLINE, de_v_begin);
	vpu_wr(ENCP_VIDEO_VAVON_ELINE, de_v_end - 1);
	vpu_wr(ENCP_VIDEO_HSO_BEGIN, hsync_st - latency);
	vpu_wr(ENCP_VIDEO_HSO_END, hsync_st + cta[i].hsync - latency);
	vpu_wr(ENCP_VIDEO_VSO_BEGIN, 0);
	vpu_wr(ENCP_VIDEO_VSO_END, 0);
	vpu_wr(ENCP_VIDEO_VSO_BLINE, vsync_st);
	vpu_wr(ENCP_VIDEO_VSO_ELINE, vsync_st + cta[i].vsync);
	vpu_wr(ENCP_VIDEO_MAX_PXCNT, cta[i].htotal - 1u);
	vpu_wr(ENCP_VIDEO_MAX_LNCNT, h.max_lncnt != 0 && cta[i].vic == h.mode.vic ? h.max_lncnt : cta[i].vtotal - 1u);
	vpu_bits(ENCP_VIDEO_MODE_ADV, 0, 3, 0);
}

static void
cor_pulse(uint32_t off, unsigned bit)
{
	cor_bits(off, bit, 1, 1);
	cor_bits(off, bit, 1, 0);
}

/* hdmitx_set_div40(): the sink told (SCDC TMDS_CONFIG), the TMDS clock pattern 1/40 or 1/10, scrambling */
static bool scdc_write(uint8_t offset, uint8_t value);
static void tx_csc(const struct txmode *m);
static void core3_hdr10(void);
static bool core3_hdr(void);
static const uint8_t uboot_drm[31];

static void
set_div40(bool div40)
{
	bool ok = scdc_write(SCDC_TMDS_CFG, div40 ? 0x03 : 0x00);

	if (div40) {
		top_wr(TOP_TMDS_CLK_PTTN_01, 0);
		top_wr(TOP_TMDS_CLK_PTTN_23, 0x3ffu << 16 | 0x3ff);
	} else {
		top_wr(TOP_TMDS_CLK_PTTN_01, 0x001f001f);
		top_wr(TOP_TMDS_CLK_PTTN_23, 0x001f001f);
	}
	top_wr(TOP_TMDS_CLK_CNTL, 0x1);
	top_wr(TOP_BIST_CNTL, 1u << 12);                                /* tmds_sel 1: normal data */
	if (div40) {
		top_wr(TOP_TMDS_CLK_CNTL, 0x2);
	}
	cor_wr(SCRCTL, (uint8_t)(1u << 5 | (div40 ? 1 : 0)));
	printf("hdmitx: TMDS clock 1/%u, scrambling %s; SCDC TMDS_CONFIG %s\n", div40 ? 40 : 10, div40 ? "on" : "off",
	    ok ? "acked" : "NOT acked");
}

/* config_hdmi21_tx(): the core for this mode (the audio part is hakios_hdmitx_audio_start's) */
static void
core_config(const struct txmode *m, int i)
{
	uint32_t tmds = mode_tmds_khz(m);
	bool scramble = tmds > 340000;
	unsigned cd = m->depth == 12 ? 6 : m->depth == 10 ? 5 : 4;     /* Amlogic's COLORDEPTH_24B/30B/36B */
	unsigned dp = m->cs == CS_422 ? 4 : cd;
	uint32_t map;

	top_wr(TOP_SW_RESET, 0);
	set_div40(scramble);
	top_wr(TOP_HPD_FILTER, 8u << 28 | 7u << 12);
	/* DRM and vendor infoframes off until they are sent again */
	cor_wr(TPI_INFO_FSEL, 6);
	cor_wr(TPI_INFO_EN, 0);
	cor_wr(TPI_INFO_FSEL, 5);
	cor_wr(TPI_INFO_EN, 0);
	cor_pulse(PWD_SRST, 0);
	cor_wr(P2T_CTRL, (uint8_t)((dp & 3) | (dp != 4 ? 0x80 : 0)));
	cor_bits(AON_CYP_CTL, 0, 2, 2);
	cor_wr(SCRCTL, (uint8_t)(1u << 5 | (scramble ? 1 : 0)));
	cor_bits(FRL_LINK_RATE_CONFIG, 0, 4, 0);
	cor_wr(SW_RST, 0);
	cor_wr(HT_DIG_CTL22_PHY, 0);
	cor_wr(CLK_DIV_CNTRL, 1);
	cor_wr(CLKPWD, 0xf4);
	cor_wr(SOC_FUNC_SEL, 0x01);
	cor_wr(TEST_TXCTRL, 0x02);
	cor_wr(CLKRATIO, 0x8a);
	cor_wr(VP_CMS_CSC1_MULTI_CSC_CONFIG, 0x00);
	cor_wr(VP_CMS_CSC1_MULTI_CSC_CONFIG + 1, 0x08);
	tx_csc(m);
	cor_wr(VP_INPUT_MASK, (uint8_t)((6 - cd) << 4 | (6 - cd) << 2 | (6 - cd)));
	cor_wr(VP_CMS_CSC1_C444_C422_CONFIG, 0x00);
	if (m->cs == CS_422) {
		map = 2u << 9 | 4u << 6;
		cor_wr(VP_OUTPUT_MAPPING, (uint8_t)map);
		cor_wr(VP_OUTPUT_MAPPING + 1, (uint8_t)(map >> 8));
		cor_wr(VP_OUTPUT_MASK, 2u << 4 | 2u << 2 | 2u);
	} else {
		map = 2u << 9 | 1u << 6;
		cor_wr(VP_OUTPUT_MAPPING, (uint8_t)map);
		cor_wr(VP_OUTPUT_MAPPING + 1, (uint8_t)(map >> 8));
		cor_wr(VP_OUTPUT_MASK, 0);
	}
	cor_wr(VTEM_CTRL, 0x04);
	cor_wr(GEN5_CTRL, 0x04);
	top_wr(TOP_HS_INTR_CNTL, 0x010);
	top_wr(TOP_SEC_VIDEO_OVR, 0xefu << 22 | 0xcdu << 12 | 0xabu << 2);
	top_wr(TOP_HDCP14_MIN_SIZE, 0);
	top_wr(TOP_HDCP22_MIN_SIZE, 0);
	top_wr(TOP_HV_ACTIVE, (uint32_t)cta[i].height << 16 | cta[i].width);
	cor_bits(PWD_SRST, 1, 2, 3);
	cor_bits(PWD_SRST, 1, 2, 0);
	cor_bits(TPI_SC, 0, 1, 1);                                      /* HDMI, not DVI */
	cor_wr(PKT_AUTO_0, cor_rd(PKT_AUTO_0) | 0x40);                  /* the HDR packet where it is by default */
	cor_wr(PKT_LOC_GEN, 0);
}

/* One packet buffer slot: [HB0 HB1 HB2 checksum PB1..PB27]; the checksum is made here unless raw (packets as read back) */
static void
send_packet(unsigned slot, const uint8_t in[31], bool raw)
{
	uint8_t f[31], sum = 0;

	memcpy(f, in, sizeof(f));
	if (!raw) {
		f[3] = 0;
		for (unsigned k = 0; k < 31; k++) {
			sum += f[k];
		}
		f[3] = (uint8_t)(0 - sum);
	}
	cor_wr(TPI_INFO_FSEL, (uint8_t)slot);
	for (unsigned k = 0; k < 31; k++) {
		cor_wr(TPI_INFO_B0 + k, f[k]);
	}
	cor_wr(TPI_INFO_EN, 0xe0);
}

/* construct_avi_packet(): underscan, 16:9, BT.709 (BT.2020 with a PQ/HLG DRM frame), limited range, graphics */
static void
make_avi(const struct txmode *m, bool bt2020, uint8_t f[31])
{
	int i = cta_index(m->vic);
	uint8_t vic = m->vic;

	memset(f, 0, 31);
	if (vic == 93 || vic == 94 || vic == 95) {
		vic = 0;                                                /* (HDMI 1.4b: these go in the vendor frame) */
	}
	f[0] = 0x82;
	f[1] = 0x02;
	f[2] = 13;
	f[4] = (uint8_t)(m->cs << 5 | 1u << 4 | 2);                     /* Y, A0 = 1, S = underscan */
	f[5] = (uint8_t)((bt2020 ? 3u : cta[i].height <= 576 ? 1u : 2u) << 6 | 2u << 4 | 8);    /* C, M 16:9, R as picture */
	f[6] = (uint8_t)((bt2020 ? 6u : 0u) << 4 | 1u << 2);            /* EC (6: BT.2020 YCC), Q limited */
	f[7] = vic;
}

/* the HDMI 1.4 vendor frame of 4K at 24/25/30 Hz (hdmi_set_vend_spec_infofram) */
static bool
make_vendor(uint8_t vic, uint8_t f[31])
{
	memset(f, 0, 31);
	if (vic != 93 && vic != 94 && vic != 95) {
		return false;
	}
	f[0] = 0x81;
	f[1] = 0x01;
	f[2] = 5;
	f[4] = 0x03;
	f[5] = 0x0c;
	f[6] = 0x00;
	f[7] = 0x20;
	f[8] = vic == 95 ? 1 : vic == 94 ? 2 : 3;
	return true;
}

/* hdmitx_set_phypara(): reset the PHY, its swing for the TMDS rate, the termination from the efuse */
static void
phy_set(uint32_t tmds_khz)
{
	uint64_t rterm = sip(SIP_HDCPTX, HDCPTX_GET_RTERM, 0, 0) & 0xff;
	uint32_t val = rterm != 0xff ? (uint32_t)rterm << 28 : 0x80000000u;
	uint32_t c0, c3 = 0x4ef001, c5 = 0x2555;

	if (tmds_khz > 300000) {
		c0 = 0x3caeb;                                           /* (Xiaomi's swing: Amlogic's sources have 0x3cafb) */
	} else if (tmds_khz > 150000) {
		c0 = 0x380dd;
	} else {
		c0 = 0x2038088;
	}
	c0 |= val;
	if (h.phy0 != 0 && mode_tmds_khz(&h.mode) == tmds_khz) {
		c0 = h.phy0;                                            /* what U-Boot left for this very mode */
		c5 = h.phy5 ? h.phy5 : c5;
	}
	wr(h.ana, HDMIPHY_CTRL0, 0);
	bits(h.ana, HDMIPHY_CTRL1, 16, 16, 0x0390);
	bits(h.ana, HDMIPHY_CTRL1, 0, 4, 0);
	for (int k = 0; k < 2; k++) {
		bits(h.ana, HDMIPHY_CTRL1, 0, 4, 0xf);
		IOSleep(2);
		bits(h.ana, HDMIPHY_CTRL1, 0, 4, 0xe);
		IOSleep(2);
	}
	wr(h.ana, HDMIPHY_CTRL0, c0);
	wr(h.ana, HDMIPHY_CTRL5, c5);
	wr(h.ana, HDMIPHY_CTRL3, c3);
	IODelay(100);
	bits(h.ana, HDMIPHY_CTRL3, 10, 2, 3);
	bits(h.ana, HDMIPHY_CTRL3, 3, 2, 3);
	bits(h.ana, HDMIPHY_CTRL3, 1, 1, 1);
	IODelay(1000);
	bits(h.ana, HDMIPHY_CTRL3, 28, 2, 3);
	printf("hdmitx: PHY ctrl0 %#x ctrl3 %#x ctrl5 %#x (rterm %#llx)\n", rd(h.ana, HDMIPHY_CTRL0), rd(h.ana, HDMIPHY_CTRL3),
	    rd(h.ana, HDMIPHY_CTRL5), rterm);
}

/* hdmi_phy_suspend() and the PLL off: no TMDS on the wire (cmd 4, for tests) */
static void
output_off(void)
{
	wr(h.ana, HDMIPHY_CTRL0, 0);
	wr(h.ana, HDMIPHY_CTRL3, 0x3);
	wr(h.ana, HDMIPHY_CTRL6, 0x800);
	wr(h.ana, HDMIPLL_CTRL0, 0);
	wr(h.ana, HDMIPLL_CTRL3, 0);
	printf("hdmitx: output off\n");
}

/*
 * The vendor's colour pipeline (CoreELEC's): the OSD turned to YCbCr by the OSD matrices of the VPP wrapper (BT.709
 * limited) and blended in YCbCr, both Dolby Vision cores out of the path, SDR. Xiaomi's U-Boot instead runs the OSD
 * through Dolby Vision core 2a (RGB to YCbCr with tone-mapping LUTs it loads through a write-only DMA port) and core 3
 * (HDR10 out), sized and set for its mode: with no TV at power-on both are left for 1080p SDR 8-bit, which cut a 4K
 * picture and could not be set right from here. Off: U-Boot's path back (boot-arg hdmitx_yuvblend=0). With the lock held.
 */
/*
 * OSD1's HDR core (OSD1_HDR2, VCBUS 0x38a0..0x38df) as the vendor's amvecm sets it on the S7D for SDR graphics on an HDR10
 * output (hdr_func(OSD1_HDR, SDR_HDR), set_hdr2_v0.c): the sRGB-ish EOTF to linear, BT.709 to BT.2020 gamut, a gain that
 * puts SDR white near 300 nits, the PQ OETF, out as RGB (10-bit). The OSD matrices then make BT.2020 YCbCr of it. Off:
 * HDR_BYPASS. Without it tvOS's SDR pictures went out marked PQ / BT.2020, oversaturated.
 */
#define OSD1_HDR2_CTRL          0x38a0
#define OSD1_HDR2_MATRIXI       0x38a2          /* COEF00_01 .. COEF22 (5), ... OFFSET0_1 0x38aa, OFFSET2, PRE_OFFSET0_1, PRE_OFFSET2 */
#define OSD1_HDR2_MATRIXO       0x38ae          /* COEF00_01 .. COEF22 (5), ... OFFSET0_1 0x38b6, OFFSET2, PRE_OFFSET0_1, PRE_OFFSET2 */
#define OSD1_HDR2_CGAIN_OFFT    0x38bc
#define OSD1_EOTF_LUT_ADDR      0x38be
#define OSD1_EOTF_LUT_DATA      0x38bf
#define OSD1_OETF_LUT_ADDR      0x38c0
#define OSD1_OETF_LUT_DATA      0x38c1
#define OSD1_CGAIN_LUT_ADDR     0x38c2
#define OSD1_CGAIN_LUT_DATA     0x38c3
#define OSD1_HDR2_CGAIN_COEF0   0x38c4
#define OSD1_HDR2_CGAIN_COEF1   0x38c5
#define OSD1_OGAIN_LUT_ADDR     0x38c6
#define OSD1_OGAIN_LUT_DATA     0x38c7
#define OSD1_HDR2_ADPS_CTRL     0x38c8          /* then ALPHA0, ALPHA1, BETA0..2, COEF0, COEF1 */
#define OSD1_HDR2_GMUT_CTRL     0x38d0          /* then COEF0..4 */
#define OSD1_HDR2_MATRIXI_EN    0x38db
#define OSD1_HDR2_MATRIXO_EN    0x38dc
#define OSD1_HDR2_HIST_CTRL     0x38dd          /* then H_START_END, V_START_END */

static const uint32_t osd_eotf_sdr[143] = {             /* eo_y_lut_sdr */
	0, 163808, 199044, 219568, 234610, 247036, 255751, 264272, 270593,
	277939, 282430, 287163, 292441, 296593, 299792, 303278, 307054, 328697,
	344063, 354448, 364052, 372123, 379268, 384835, 391196, 395793, 399793,
	404214, 409063, 411973, 414834, 417919, 421229, 424767, 427260, 429261,
	431380, 433618, 435975, 438453, 441054, 443072, 444496, 445981, 447530,
	449142, 450817, 452556, 454360, 456229, 458163, 459457, 460489, 461555,
	462654, 463787, 464953, 466153, 467387, 468655, 469957, 471294, 472665,
	474071, 475324, 476062, 476817, 477590, 478381, 479190, 480016, 480861,
	481723, 482603, 483502, 484418, 485353, 486306, 487278, 488267, 489276,
	490302, 491348, 491966, 492507, 493057, 493618, 494187, 494766, 495354,
	495952, 496559, 497176, 497803, 498439, 499084, 499740, 500405, 501079,
	501764, 502458, 503161, 503875, 504598, 505332, 506075, 506828, 507590,
	508133, 508525, 508921, 509322, 509728, 510140, 510556, 510977, 511404,
	511835, 512272, 512713, 513160, 513611, 514068, 514530, 514997, 515469,
	515946, 516429, 516916, 517409, 517907, 518410, 518918, 519432, 519950,
	520474, 521003, 521537, 522077, 522622, 523172, 523727, 524287,
};

static const uint16_t osd_oetf_pq[149] = {              /* oe_y_lut_hdr (12-bit; written 10-bit) */
	0, 3, 5, 8, 12, 19, 28, 41, 60, 67, 74, 80, 85,
	96, 105, 113, 120, 134, 146, 157, 167, 184, 200, 214, 227, 250, 270,
	288, 304, 332, 357, 380, 400, 435, 465, 492, 517, 559, 595, 628, 656,
	706, 749, 787, 820, 850, 878, 903, 927, 949, 970, 989, 1008, 1042, 1073,
	1102, 1129, 1154, 1177, 1199, 1219, 1258, 1292, 1324, 1354, 1381, 1407,
	1431, 1453, 1495, 1533, 1568, 1600, 1630, 1657, 1683, 1708, 1753, 1794,
	1831, 1865, 1897, 1926, 1954, 1980, 2028, 2071, 2110, 2146, 2179, 2210,
	2239, 2267, 2317, 2361, 2402, 2440, 2474, 2506, 2536, 2564, 2616, 2662,
	2704, 2742, 2778, 2810, 2841, 2870, 2922, 2969, 3011, 3050, 3086, 3119,
	3150, 3179, 3231, 3278, 3321, 3360, 3396, 3429, 3459, 3488, 3540, 3587,
	3629, 3668, 3703, 3736, 3766, 3795, 3821, 3846, 3870, 3892, 3913, 3934,
	3953, 3971, 3989, 4006, 4022, 4038, 4053, 4068, 4082, 4095,
};

/*
 * Boot-args hdmitx_osdhdr (1): the core on in HDR10 modes, 0: off (SDR graphics as they are); hdmitx_osdgain: SDR white
 * (20, ~375 nits; the vendor's 16 looked washed out); hdmitx_osdsat: how far (%) the BT.709 to BT.2020 gamut matrix is
 * taken back toward none (25: a little more saturated than exact; 100 is the SDR colours as BT.2020's, far too much);
 * hdmitx_osdchroma: chroma gain (0x400 = 1.0; above it dark text on light grey got cyan edges). Live: sysctl
 * kern.hakios_hdmitx_osdhdr "GAIN CHROMA SAT".
 */
static uint32_t osd_hdr_on = 1, osd_hdr_gain = 20, osd_hdr_chroma = 0x400, osd_hdr_sat = 25;  /* (512 = 1.0; 16 the vendor's) (0x400 = 1.0) (%) */
static bool osd_hdr_set;
/*
 * The video core's own (kern.hakios_hdmitx_vdhdr "GAIN INSHIFT OUTSHIFT OFFSCALE [BITS]"): SDR white, HDR_CTRL bits 14
 * and 15 (set for every core by the vendor), the YCbCr offsets' scale (1: 10-bit values, as the vendor's), and the core's
 * width: on the S7D VD1's HDR core is a 12-bit one, OSD1's a 10-bit one (amvecm set_hdr2_v0.c hdr_func: bit_depth),
 * which changes the PQ OETF table (12-bit values as they are; the 10-bit core takes them >> 2) and the chroma gain's
 * unity (CGAIN_COEF1 [28:16] 0x1000; 0x400). Loaded as the OSD's 10-bit core, SDR video came out a quarter of the
 * level (very dim) with bright green edges. Applied at once while the core is on.
 */
static uint32_t vd_hdr_gain = 20, vd_in_shift = 1, vd_out_shift = 1, vd_off_scale = 1, vd_bits = 12;

/*
 * An HDR core (S7D: OSD1's at 0x38a0, VD1's at 0x3800, the same layout) set as amvecm's SDR_HDR, or off (HDR_BYPASS).
 * video: the input is BT.709 limited YCbCr (VD1: the video plane's pictures) and the output BT.2020 limited YCbCr,
 * the gamut matrix exact; else RGB in and out, the OSD's (its matrices after it make the YCbCr).
 */
#define HDR_CTRL        0x00
#define HDR_MATRIXI     0x02
#define HDR_MATRIXO     0x0e
#define HDR_CGAIN_OFFT  0x1c
#define HDR_EOTF_ADDR   0x1e
#define HDR_EOTF_DATA   0x1f
#define HDR_OETF_ADDR   0x20
#define HDR_OETF_DATA   0x21
#define HDR_CGAIN_ADDR  0x22
#define HDR_CGAIN_DATA  0x23
#define HDR_CGAIN_COEF0 0x24
#define HDR_CGAIN_COEF1 0x25
#define HDR_OGAIN_ADDR  0x26
#define HDR_OGAIN_DATA  0x27
#define HDR_ADPS_CTRL   0x28
#define HDR_GMUT_CTRL   0x30
#define HDR_MATRIXI_EN  0x3b
#define HDR_MATRIXO_EN  0x3c
#define HDR_HIST_CTRL   0x3d
#define VD1_HDR2_CTRL   0x3800

/* a 3x3 matrix (1024 = 1.0) and its offsets into an HDR core's matrix registers (amvecm set_hdr_matrix) */
static void
hdr_matrix(uint32_t at, const int32_t m[9], const int32_t pre[3], const int32_t post[3])
{
	vpu_wr(at + 0, (uint32_t)(m[0] & 0x1fff) << 16 | (uint32_t)(m[1] & 0x1fff));
	vpu_wr(at + 1, (uint32_t)(m[2] & 0x1fff) << 16 | (uint32_t)(m[3] & 0x1fff));
	vpu_wr(at + 2, (uint32_t)(m[4] & 0x1fff) << 16 | (uint32_t)(m[5] & 0x1fff));
	vpu_wr(at + 3, (uint32_t)(m[6] & 0x1fff) << 16 | (uint32_t)(m[7] & 0x1fff));
	vpu_wr(at + 4, (uint32_t)(m[8] & 0x1fff));
	vpu_wr(at + 8, (uint32_t)(post[0] & 0xfff) << 16 | (uint32_t)(post[1] & 0xfff));
	vpu_wr(at + 9, (uint32_t)(post[2] & 0xfff));
	vpu_wr(at + 10, (uint32_t)(pre[0] & 0xfff) << 16 | (uint32_t)(pre[1] & 0xfff));
	vpu_wr(at + 11, (uint32_t)(pre[2] & 0xfff));
}

static void
hdr_core(uint32_t base, bool on, bool video, bool *set)
{
	static const uint32_t identity[5] = { 0x04000000, 0, 0x04000000, 0, 0x400 };

	if (!on && !*set) {
		return;                 /* (never turned on: whatever U-Boot left stays) */
	}
	/* the input matrix: identity, no offsets (S7D: "osd blend after hdr core") */
	vpu_bits(base + HDR_CTRL, 13, 1, on);
	vpu_wr(base + HDR_MATRIXI_EN, 1);
	vpu_bits(base + HDR_CTRL, 4, 1, on);
	vpu_bits(base + HDR_CTRL, 16, 1, !on);
	vpu_bits(base + HDR_CTRL, 14, 1, video ? vd_in_shift : 1);
	static const int32_t ycbcr709_rgb[9] = { 1192, 0, 1836, 1192, -217, -546, 1192, 2166, 0 };
	static const int32_t rgb_ycbcr2020[9] = { 230, 594, 52, -125, -323, 448, 448, -412, -36 };
	const int32_t o = (int32_t)vd_off_scale, yuv_pre[3] = { -64 * o, -512 * o, -512 * o }, yuv_post[3] = { 64 * o, 512 * o, 512 * o };
	static const int32_t zero[3] = { 0, 0, 0 };
	if (video && on) {
		hdr_matrix(base + HDR_MATRIXI, ycbcr709_rgb, yuv_pre, zero);
	} else {
		for (unsigned k = 0; k < 5; k++) {
			vpu_wr(base + HDR_MATRIXI + k, identity[k]);
		}
		for (unsigned k = 8; k < 12; k++) {
			vpu_wr(base + HDR_MATRIXI + k, 0);       /* OFFSET0_1, OFFSET2, PRE_OFFSET0_1, PRE_OFFSET2 */
		}
	}
	/* EOTF */
	vpu_bits(base + HDR_CTRL, 3, 1, on);
	if (on) {
		vpu_wr(base + HDR_EOTF_ADDR, 0);
		for (unsigned k = 0; k < 143; k++) {
			vpu_wr(base + HDR_EOTF_DATA, osd_eotf_sdr[k]);
		}
	}
	/* gamut (BT.709 to BT.2020, 2048 = 1), the chroma gain's luma weights, the adaptive scaler */
	vpu_bits(base + HDR_CTRL, 13, 1, on);
	vpu_bits(base + HDR_CTRL, 6, 2, 1);
	vpu_wr(base + HDR_GMUT_CTRL, 0xb);
	if (on) {
		/* ncl_709_2020 (2048 = 1.0), taken osd_hdr_sat % of the way to the identity */
		static const int32_t g709[9] = { 1285, 674, 89, 142, 1883, 23, 34, 180, 1834 };
		uint32_t m[9], t = video ? 0 : osd_hdr_sat > 100 ? 100 : osd_hdr_sat;

		for (unsigned k = 0; k < 9; k++) {
			m[k] = (uint32_t)((g709[k] * (int32_t)(100 - t) + (k % 4 == 0 ? 2048 : 0) * (int32_t)t + 50) / 100) & 0xffff;
		}
		vpu_wr(base + HDR_GMUT_CTRL + 1, m[1] << 16 | m[0]);
		vpu_wr(base + HDR_GMUT_CTRL + 2, m[3] << 16 | m[2]);
		vpu_wr(base + HDR_GMUT_CTRL + 3, m[5] << 16 | m[4]);
		vpu_wr(base + HDR_GMUT_CTRL + 4, m[7] << 16 | m[6]);
		vpu_wr(base + HDR_GMUT_CTRL + 5, m[8]);
		vpu_wr(base + HDR_CGAIN_COEF0, 0x09480398);
		vpu_bits(base + HDR_CGAIN_COEF1, 0, 12, 0x0d0);
		vpu_wr(base + HDR_ADPS_CTRL, 0x00010001);
	} else {
		vpu_wr(base + HDR_GMUT_CTRL + 1, 0x800);
		vpu_wr(base + HDR_GMUT_CTRL + 2, 0);
		vpu_wr(base + HDR_GMUT_CTRL + 3, 0x800);
		vpu_wr(base + HDR_GMUT_CTRL + 4, 0);
		vpu_wr(base + HDR_GMUT_CTRL + 5, 0x800);
		vpu_wr(base + HDR_CGAIN_COEF0, 0x00001000);
		vpu_bits(base + HDR_CGAIN_COEF1, 0, 12, 0);
		vpu_wr(base + HDR_ADPS_CTRL, 0x00010071);
	}
	vpu_wr(base + HDR_ADPS_CTRL + 2, 0xa09a0400);           /* ALPHA1 */
	vpu_wr(base + HDR_ADPS_CTRL + 1, 0x04000400);           /* ALPHA0 */
	vpu_wr(base + HDR_ADPS_CTRL + 3, 0);
	vpu_wr(base + HDR_ADPS_CTRL + 4, 0);
	vpu_wr(base + HDR_ADPS_CTRL + 5, 0);
	vpu_wr(base + HDR_ADPS_CTRL + 6, 0x04000400);
	vpu_wr(base + HDR_ADPS_CTRL + 7, 0x400);
	/* the OOTF gain (SDR white's level) */
	vpu_bits(base + HDR_CTRL, 1, 1, on);
	if (on) {
		uint32_t gain = video ? vd_hdr_gain : osd_hdr_gain;

		vpu_wr(base + HDR_OGAIN_ADDR, 0);
		for (unsigned k = 0; k < 74; k++) {
			vpu_wr(base + HDR_OGAIN_DATA, gain << 16 | gain);
		}
		vpu_wr(base + HDR_OGAIN_DATA, gain);
	}
	/* the PQ OETF: 12-bit values in a 12-bit core, >> 2 in a 10-bit one */
	vpu_bits(base + HDR_CTRL, 2, 1, on);
	if (on) {
		unsigned sh = video && vd_bits == 12 ? 0 : 2;

		vpu_wr(base + HDR_OETF_ADDR, 0);
		for (unsigned k = 0; k < 74; k++) {
			vpu_wr(base + HDR_OETF_DATA, (uint32_t)(osd_oetf_pq[2 * k + 1] >> sh) << 16 | (osd_oetf_pq[2 * k] >> sh));
		}
		vpu_wr(base + HDR_OETF_DATA, osd_oetf_pq[148] >> sh);
	}
	/* out: RGB (identity) */
	vpu_bits(base + HDR_CTRL, 13, 1, on);
	vpu_wr(base + HDR_CGAIN_OFFT, 0x02000200);
	vpu_wr(base + HDR_MATRIXO_EN, on ? 1 : 0);
	vpu_bits(base + HDR_CTRL, 17, 1, 0);
	vpu_bits(base + HDR_CTRL, 15, 1, video ? vd_out_shift : 1);
	if (video && on) {
		hdr_matrix(base + HDR_MATRIXO, rgb_ycbcr2020, zero, yuv_post);
	} else {
		for (unsigned k = 0; k < 5; k++) {
			vpu_wr(base + HDR_MATRIXO + k, identity[k]);
		}
		for (unsigned k = 8; k < 12; k++) {
			vpu_wr(base + HDR_MATRIXO + k, 0);
		}
	}
	/* the chroma gain (flat at 1.0 in the vendor's) */
	vpu_bits(base + HDR_CTRL, 12, 1, 0);
	vpu_bits(base + HDR_CTRL, 0, 1, on);
	vpu_bits(base + HDR_CGAIN_COEF1, 16, 13, video && vd_bits == 12 ? 0x1000 : 0x400);
	if (on) {
		vpu_wr(base + HDR_CGAIN_ADDR, 0);
		uint32_t c = (osd_hdr_chroma ? osd_hdr_chroma : 0x400) - 1;    /* (the vendor writes value - 1) */

		for (unsigned k = 0; k < 32; k++) {
			vpu_wr(base + HDR_CGAIN_DATA, c << 16 | c);
		}
		vpu_wr(base + HDR_CGAIN_DATA, c);
	}
	/* no histogram */
	vpu_wr(base + HDR_HIST_CTRL, 0x5510);
	vpu_wr(base + HDR_HIST_CTRL + 1, 0x00010000);
	vpu_wr(base + HDR_HIST_CTRL + 2, 0);
	*set = on;
	printf("hdmitx: %s HDR core %s (CTRL %#x)\n", video ? "VD1" : "OSD", on ? "on: SDR to PQ BT.2020" : "off",
	    vpu(base + HDR_CTRL));
}


static void
osd_hdr(bool on)
{
	hdr_core(OSD1_HDR2_CTRL, on, false, &osd_hdr_set);
}

/*
 * The video plane's pictures (VD1) on an HDR10 output: an SDR picture made PQ BT.2020 by VD1's HDR core, as the OSD's
 * graphics are; an HDR10 one (PQ already) through as it is. Without it SDR video went out as if BT.2020/PQ: far too
 * red and saturated. hakios.vplane.c says what each picture is (its token's TWL_VPLANE_PQ).
 */
static bool vd_hdr_set, vd_pq;

static void
video_hdr_locked(void)
{
	bool want = h.yuv_blend && h.mode.hdr && osd_hdr_on && !vd_pq;
	if (want != vd_hdr_set) hdr_core(VD1_HDR2_CTRL, want, true, &vd_hdr_set);
}

void
hakios_hdmitx_video_pq(bool pq)
{
	lck_mtx_lock(&hdmitx_lock);
	vd_pq = pq;
	video_hdr_locked();
	lck_mtx_unlock(&hdmitx_lock);
}

static void
osd_yuv(bool on)
{
	static const uint32_t bt709[13] = {
		0x00bb0275, 0x003f1f99, 0x1ea601c2, 0x01c21e67, 0x00001fd7, 0, 0, 0, 0, 0x00400200, 0x00000200, 0, 0,
	};
	/* (RGB to BT.2020 YCbCr, 10-bit limited: after the HDR core, MATRIX_RGB_BT2020YUV) */
	static const uint32_t bt2020[13] = {
		0x00e60252, 0x00341f83, 0x1ebd01c0, 0x01c01e63, 0x00001fdc, 0, 0, 0, 0, 0x00400200, 0x00000200, 0, 0,
	};
	bool hdr = on && h.mode.hdr && osd_hdr_on;
	const uint32_t *m = hdr ? bt2020 : bt709;

	osd_hdr(hdr);
	video_hdr_locked();
	for (unsigned k = 0; k < 13; k++) {
		vpu_wr(VPP_WRAP_OSD1_MATRIX + k, m[k]);
		vpu_wr(VPP_OSD2_MATRIX + k, m[k]);
	}
	vpu_wr(VPP_WRAP_OSD1_MATRIX + 13, on ? 1 : 0);
	vpu_wr(VPP_OSD2_MATRIX + 13, on ? 1 : 0);
	if (on) {
		vpu_bits(DOLBY_PATH_CTRL, 2, 1, 1);                     /* core 2a bypassed */
		vpu_bits(VPP_DOLBY_CTRL, 3, 1, 0);                      /* core 3 out of the post-blend path */
	} else {
		vpu_wr(DOLBY_PATH_CTRL, h.dv_path);
		vpu_wr(VPP_DOLBY_CTRL, h.dv_ctrl);
	}
}

/*
 * The infoframes for what the pipeline makes: U-Boot's own (HDR10 from its Dolby Vision path) when it left this mode
 * and that path is kept, else made here (SDR BT.709, or HDR10 when core 3 makes it). With the lock held.
 */
static void
send_frames(const struct txmode *m)
{
	uint8_t f[31];
	bool hdr = core3_hdr() || m->hdr;

	if (h.have_frames && !h.yuv_blend) {
		for (unsigned s = 0; s < 11; s++) {
			if (s == INFOFRAME_AUDIO) {
				continue;                               /* (the audio side sends its own) */
			}
			if (h.frame_on[s]) {
				send_packet(s, h.frames[s], true);
			} else {
				cor_wr(TPI_INFO_FSEL, (uint8_t)s);
				cor_wr(TPI_INFO_EN, 0);
			}
		}
		return;
	}
	make_avi(m, hdr, f);
	send_packet(0, f, false);
	if (make_vendor(m->vic, f)) {
		send_packet(5, f, false);
	} else {
		cor_wr(TPI_INFO_FSEL, 5);
		cor_wr(TPI_INFO_EN, 0);
	}
	if (hdr) {
		send_packet(6, uboot_drm, false);
	} else {
		cor_wr(TPI_INFO_FSEL, 6);
		cor_wr(TPI_INFO_EN, 0);
	}
}

/* the TX's input CSC for this output colour space, as the vendor writes it (this block reads 0xff through the SIP call) */
static void
tx_csc(const struct txmode *m)
{
	if (m->cs == CS_RGB) {
		cor_wr(VP_CMS_CSC0_MULTI_CSC_CONFIG, 0x65);
		cor_wr(VP_CMS_CSC0_MULTI_CSC_CONFIG + 1, 0x08);
	} else {
		cor_wr(VP_CMS_CSC0_MULTI_CSC_CONFIG, 0x00);
		cor_wr(VP_CMS_CSC0_MULTI_CSC_CONFIG + 1, 0x08);
	}
}

/*
 * The VPP's output for this mode: the OSD scaled from its input (VPP_OSD_SCI_WH_M1, the framebuffer) to the whole
 * picture, the post-blend and output sizes, the VIU to ENCP. U-Boot sets these for its mode; with no TV at power-on
 * (outputmode dummy_l) it leaves 1080 sizes, 1:1 steps and no mux: the picture came out cut, or not at all (green).
 */
static void
vpp_output(int i)
{
	uint32_t w = cta[i].width, hgt = cta[i].height, in = vpu(VPP_OSD_SCI_WH_M1);
	uint32_t in_w = (in >> 16) + 1, in_h = (in & 0x1fff) + 1;

	vpu_wr(VPP_POSTBLEND_H_SIZE, w);
	vpu_wr(VPP_OUT_H_V_SIZE, w << 16 | hgt);
	vpu_wr(VPP_OSD_VSC_PHASE_STEP, (uint32_t)(((uint64_t)in_h << 24) / hgt));
	vpu_wr(VPP_OSD_HSC_PHASE_STEP, (uint32_t)(((uint64_t)in_w << 24) / w));
	vpu_wr(VPP_OSD_SCO_H, w - 1);
	vpu_wr(VPP_OSD_SCO_V, hgt - 1);
	vpu_wr(VPP_OSD1_BLD_H_SCOPE, w - 1);
	vpu_wr(VPP_OSD1_BLD_V_SCOPE, hgt - 1);
	vpu_bits(VPU_VIU_VENC_MUX_CTRL, 0, 4, 0xe);
	/*
	 * Xiaomi's U-Boot puts Dolby Vision core 3 in the path (in bypass) after the post-blend, sized for its mode: with no
	 * TV, 1920x1080. A 4K picture then stopped after 1920 x 1080 pixels (a quarter down) and its last pixel ran on.
	 * dolby_core3_set(): h + 0x140, v + 0x40 + 2 * 0x10 (vsize_hold).
	 */
	if (vpu(VPP_DOLBY_CTRL) & (1u << 3)) {
		uint32_t c1 = vpu(DOLBY_CORE3_SWAP_CTRL1), c2 = vpu(DOLBY_CORE3_SWAP_CTRL2);

		vpu_wr(DOLBY_CORE3_SWAP_CTRL1, (w + 0x140) << 16 | (hgt + 0x40 + 2 * 0x10));
		vpu_wr(DOLBY_CORE3_SWAP_CTRL2, w << 16 | hgt);
		if (c2 != (w << 16 | hgt)) {
			printf("hdmitx: Dolby Vision core 3 (in the path) was sized %ux%u (%#x %#x): now %ux%u\n", c2 >> 16, c2 & 0xffff,
			    c1, c2, w, hgt);
		}
		if (h.hdr10 && vpu(DOLBY_CORE3_CTRL) != 2) {
			core3_hdr10();
		}
	}
}

/*
 * Dolby Vision core 3 as Xiaomi's U-Boot sets it up with a TV there (read back from the box): HDR10 out, 1000 nits.
 * With no TV U-Boot leaves it in SDR 8-bit (CTRL 4) with other DM values. Its HDR10 infoframes go with it.
 */
static const uint32_t core3_hdr10_dm[26] = {
	0x011a4000, 0x4000071b, 0xf8e5fee6, 0x47af4000, 0x000eeb7b, 0xafc76dfb, 0xe6b0023d, 0xf9d63f7a, 0xfcd3ff2c, 0x000d23ff,
	0x0000051e, 0x0fa00000, 0x00000db0, 0x00000100, 0x00000800, 0x00000800, 0x00010000, 0x00000000, 0x4a411cc5, 0xf05b067e,
	0x3803d7a0, 0xcc7d3803, 0x000ffb7e, 0x00000100, 0x00000800, 0x00000800,
};
static const uint8_t uboot_drm[31] = {          /* DRM (HDR) infoframe: PQ, static metadata type 1, 1000 nits */
	0x87, 0x01, 0x1a, 0x0e, 0x02, 0x00, 0x48, 0x8a, 0x08, 0x39, 0x34, 0x21, 0xaa, 0x9b, 0x96, 0x19, 0xfc, 0x08, 0x13, 0x3d,
	0x42, 0x40, 0xe8, 0x03, 0x31,
};

/* dolby_core3_set()'s order: metadata window, mode, DM registers, swap enable. With the lock held. */
static void
core3_hdr10(void)
{
	vpu_wr(DOLBY_CORE3_MD_START, 1);
	vpu_wr(DOLBY_CORE3_CTRL, 2);
	for (unsigned k = 0; k < 26; k++) {
		vpu_wr(DOLBY_CORE3_DM + k, core3_hdr10_dm[k]);
	}
	vpu_wr(DOLBY_CORE3_MD_END, 1);
	vpu_wr(DOLBY_CORE3_SWAP_CTRL0, 1);
	printf("hdmitx: Dolby Vision core 3 set to HDR10, as U-Boot does with a TV\n");
}

/* core 3 in the path and making HDR10: the picture is PQ / BT.2020 and the infoframes must say so */
static bool
core3_hdr(void)
{
	return (vpu(VPP_DOLBY_CTRL) & (1u << 3)) && vpu(DOLBY_CORE3_CTRL) == 2;
}

/* boot-arg hdmitx_trace=1: each step of a mode set to the log (a step that stalls the bus is the last line) */
static uint32_t trace;
#define STEP(what) do { if (trace) printf("hdmitx: mode set: %s\n", what); } while (0)

/* hdmitx21_set(). With the lock held. */
static bool
modeset(const struct txmode *m)
{
	int i = cta_index(m->vic);
	bool analog_path = h.analog_path, y420 = m->cs == CS_420, locked;
	uint32_t tmds = mode_tmds_khz(m), v;
	uint8_t f[31];
	uint64_t t0 = mach_absolute_time(), ns;

	if (i < 0 || tmds == 0) {
		printf("hdmitx: mode set: VIC %u not known\n", m->vic);
		return false;
	}
	printf("hdmitx: mode set %u: VIC %u %ux%u%s, %s %u-bit, TMDS %u kHz\n", ++h.modesets, m->vic, cta[i].width, cta[i].height,
	    m->fractional ? " /1.001" : "", m->cs == CS_RGB ? "RGB" : m->cs == CS_422 ? "YCbCr 4:2:2" : m->cs == CS_444 ? "YCbCr 4:4:4" :
	    "YCbCr 4:2:0", m->depth, tmds);
	/* the ENCP clock: through the VID PLL divider (VID_CLK0_CTRL source 0), or the PLL's "analog path" (3) as U-Boot had it */
	STEP("PLL");
	locked = pll_set(m, analog_path);
	STEP("hw init");
	hw_init();
	STEP("clock tree");
	clock_tree(analog_path);
	STEP("encoders off");
	vpu_wr(ENCP_VIDEO_EN, 0);
	vpu_wr(ENCI_VIDEO_EN, 0);
	/* enable_crt_video_hdmi(1, 0, 0): pixel, fe, pnx clocks */
	bits(h.clk, VID_CLK0_CTRL2, 10, 1, 1);
	bits(h.clk, VID_CLK0_CTRL2, 9, 1, 1);
	bits(h.clk, VID_CLK0_CTRL2, 5, 1, 1);
	wr(h.clk, ENC_HDMI_CLK_CTRL, 0);
	bits(h.clk, ENC_HDMI_CLK_CTRL, 20, 1, 1);
	bits(h.clk, ENC_HDMI_CLK_CTRL, 12, 1, 1);
	bits(h.clk, ENC_HDMI_CLK_CTRL, 4, 1, 1);
	/* GCP: non-merge for 8-bit and 4:2:2, merge for deep colour */
	STEP("GCP");
	cor_bits(GCP_CNTL, 0, 1, m->cs == CS_422 || m->depth == 8 ? 0 : 1);
	STEP("ENCP timing");
	vpu_wr(VPU_VENC_CTRL, 1);
	encp_timing(i, y420);
	bits(h.clk, VID_CLK0_CTRL2, 2, 1, 1);
	bits(h.clk, VID_CLK0_CTRL2, 3, 1, 1);
	bits(h.clk, VID_CLK0_CTRL2, 5, 1, 1);
	bits(h.clk, VID_CLK0_CTRL2, 9, 1, 1);
	bits(h.clk, VID_CLK0_CTRL2, 10, 1, 1);
	bits(h.clk, HTX_CLK_CTRL1, 24, 1, 1);
	STEP("formatter");
	/* the VPU's HDMI formatter: 4:4:4 in, to 4:2:2 / 4:2:0, chroma averaged, dither for 8 bits */
	v = (m->cs == CS_RGB ? 3u : m->cs == CS_420 ? 2u : m->cs == CS_422 ? 1u : 0u) | 2u << 2 | (m->depth == 8 ? 1u : 0u) << 10 | 2u << 22;
	if (m->cs == CS_RGB) {
		v |= 1u << 31;
	}
	vpu_wr(VPU_HDMI_FMT_CTRL, v);
	vpu_wr(VPU_HDMI_DITH_CNTL, (m->depth == 8 ? 1u : 0u) << 10);
	vpu_bits(VPU_HDMI_DITH_CNTL, 4, 1, m->depth == 8 ? 1 : 0);
	top_wr(TOP_I2C_BUSY_CNT_MAX, 30);
	top_wr(TOP_HDCP14_MIN_SIZE, 1u << 31 | 240u << 16 | 1u << 15 | 640);
	top_wr(TOP_HDCP22_MIN_SIZE, 1u << 31 | 1080u << 16 | 1u << 15 | 1920);
	STEP("soft resets");
	/* hdmitx_soft_reset(1..4): all logic, the cipher, the TPI state machine, HDCP 2 */
	cor_pulse(PWD_SRST, 0);
	cor_pulse(HDCP_CTRL, 2);
	cor_pulse(AON_CYP_CTL, 3);
	cor_pulse(HDCP2X_TX_SRST, 5);
	cor_bits(PCLK2TMDS_MISC1, 4, 1, 0);
	STEP("OSD matrices");
	osd_yuv(h.yuv_blend);
	STEP("VPP output");
	vpp_output(i);
	STEP("core");
	core_config(m, i);
	STEP("infoframes");
	vpu_bits(ENCP_VIDEO_SYNC_MODE, 4, 1, 0);
	send_frames(m);
	STEP("ENCP on");
	vpu_wr(VPU_HDMI_SETTING, 2u | 1u << 2 | 1u << 3 | (m->cs == CS_420 ? 4u : 0u) << 5 | (m->cs == CS_420 ? 1u : 0u) << 20 |
	    (m->cs == CS_RGB ? 1u : 0u) << 16);
	vpu_wr(ENCP_VIDEO_EN, 1);
	vpu_wr(ENCI_VIDEO_EN, 0);
	STEP("PHY to digital");
	/* hdmitx_set_phy_todig() */
	bits(h.ana, HDMIPHY_CTRL3, 0, 2, 3);
	bits(h.ana, HDMIPHY_CTRL3, 3, 1, 1);
	/* the GCP phase must settle (deep colour); 4:2:2 and 8-bit want phase 0, 12-bit 2: reset the pixel FIFO until it does */
	for (int loop = 0; loop < 20; loop++) {
		unsigned want = m->cs == CS_422 ? 0 : m->depth == 12 ? 2 : 0, st;

		for (int k = 0; k < 10 && !(cor_rd(SYS_STAT) & 1); k++) {
			IODelay(100);
		}
		st = (cor_rd(GCP_CUR_STAT) >> 5) & 3;
		if (st == want || (m->cs != CS_422 && m->depth != 12 && st == 0)) {
			break;
		}
		cor_pulse(PWD_SRST, 1);
	}
	STEP("PHY");
	phy_set(tmds);
	STEP("audio");
	if (h.audio_on) {
		(void)audio_start_locked(&h.audio);
	}
	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	printf("hdmitx: mode set %u done in %llu ms: PLL %s, ENCP %s, PHY %#x, VIU mux %#x\n", h.modesets, ns / 1000000,
	    locked ? "locked" : "NOT locked", vpu(ENCP_VIDEO_EN) & 1 ? "on" : "off", rd(h.ana, HDMIPHY_CTRL0), vpu(VPU_VIU_VENC_MUX_CTRL));
	return locked;
}

/* What U-Boot left: the mode (from the AVI frame, the PLL, P2T), its infoframes, its PHY values. With the lock held. */
static bool
snapshot_mode(void)
{
	uint8_t p2t = cor_rd(P2T_CTRL);
	uint32_t ph = vpu(ENCP_VIDEO_MAX_PXCNT) + 1, pv = vpu(ENCP_VIDEO_MAX_LNCNT) + 1;
	int i = -1;

	if (!output_alive()) {
		return false;
	}
	for (unsigned s = 0; s < 11; s++) {
		h.frame_on[s] = infoframe(s, h.frames[s]);
	}
	if (h.frame_on[0] && h.frames[0][0] == 0x82) {
		i = cta_index(h.frames[0][7]);
		h.mode.cs = (h.frames[0][4] >> 5) & 3;
	}
	/* (VIC 0 in the AVI frame: 4K at 24/25/30, by the encoder's totals) */
	for (unsigned k = 0; i < 0 && k < sizeof(cta) / sizeof(cta[0]); k++) {
		if (cta[k].htotal == ph && (cta[k].vtotal == pv || cta[k].vtotal + 2 == pv) && cta[k].height == 2160) {
			i = (int)k;
		}
	}
	if (i < 0) {
		printf("hdmitx: U-Boot's mode not known (AVI VIC %u, totals %ux%u)\n", h.frames[0][7], ph, pv);
		return false;
	}
	h.mode.vic = cta[i].vic;
	h.mode.hdr = false;
	h.mode.depth = h.mode.cs == CS_422 ? 12 : (p2t & 0x80) == 0 ? 8 : (p2t & 3) == 1 ? 10 : 12;
	h.mode.fractional = cta[i].fractional && (rd(h.ana, HDMIPLL_CTRL3) & 0x1ffff) != 0;
	h.have_frames = true;
	h.phy0 = rd(h.ana, HDMIPHY_CTRL0);
	h.phy3 = rd(h.ana, HDMIPHY_CTRL3);
	h.phy5 = rd(h.ana, HDMIPHY_CTRL5);
	h.max_lncnt = vpu(ENCP_VIDEO_MAX_LNCNT);
	h.analog_path = ((rd(h.clk, VID_CLK0_CTRL) >> 16) & 7) == 3;
	printf("hdmitx: U-Boot's mode: VIC %u, colour space %u, %u-bit%s, %u lines; its infoframes kept\n",
	    h.mode.vic, h.mode.cs, h.mode.depth, h.mode.fractional ? ", 1/1.001" : "", pv);
	return true;
}

/* ---------------------------------------------------------------------------------------------------------------- */
/* Register dumps (sysctl kern.hakios_hdmitx_dump, _bootdump; tools/hdmitx-dump.py)                                   */
/* ---------------------------------------------------------------------------------------------------------------- */

struct dump_head {
	char name[8];
	uint32_t base, count, width;            /* width 4: 32-bit registers (VCBUS: base and count in register numbers), 1: bytes */
};

static unsigned
dump_section(uint8_t *out, unsigned at, const char *name, uint32_t base, uint32_t count, uint32_t width)
{
	struct dump_head d;

	memset(&d, 0, sizeof(d));
	strlcpy(d.name, name, sizeof(d.name));
	d.base = base;
	d.count = count;
	d.width = width;
	if (at + sizeof(d) + count * width > DUMP_MAX) {
		return at;
	}
	memcpy(out + at, &d, sizeof(d));
	return at + sizeof(d);
}

static void
put32(uint8_t *p, uint32_t v)
{
	memcpy(p, &v, 4);
}

/* With the lock held */
static unsigned
take_dump(uint8_t *out)
{
	static const struct { const char *name; uint32_t phys, count; } mmio[] = {
		{ "clk", CLK_BASE, 256 }, { "ana", ANA_BASE, 256 }, { "pwr", PWR_BASE, 64 }, { "pad", PAD_BASE, 128 },
	};
	/* (only blocks known to answer: a read of a VPU block that is gated off can hang the bus) */
	static const struct { uint32_t first, count; } vcbus[] = {
		{ 0x1a10, 0x20 },               /* OSD1 */
		{ 0x1b57, 0x01 },               /* ENCI_VIDEO_EN */
		{ 0x1b80, 0x40 },               /* ENCP video */
		{ 0x1c30, 0x10 },               /* ENCP DVI/DE */
		{ 0x1cef, 0x01 },               /* VPU_VENC_CTRL */
		{ 0x1dc0, 0x10 },               /* VPP OSD scaler */
		{ 0x2710, 0x10 },               /* VIU mux, VPU_HDMI_SETTING */
		{ 0x2743, 0x01 },               /* VPU_HDMI_FMT_CTRL */
		{ 0x27fc, 0x01 },               /* VPU_HDMI_DITH_CNTL */
		{ 0x3d60, 0x0e },               /* the OSD1 matrix */
	};
	unsigned at = 8, n;
	uint8_t fsel = cor_rd(TPI_INFO_FSEL);

	memcpy(out, "HDTX", 4);
	put32(out + 4, 1);
	for (unsigned s = 0; s < sizeof(mmio) / sizeof(mmio[0]); s++) {
		vm_offset_t base = mmio[s].phys == CLK_BASE ? h.clk : mmio[s].phys == ANA_BASE ? h.ana : mmio[s].phys == PWR_BASE ? h.pwr : h.pad;

		at = dump_section(out, at, mmio[s].name, mmio[s].phys, mmio[s].count, 4);
		for (unsigned k = 0; k < mmio[s].count; k++, at += 4) {
			put32(out + at, rd(base, k * 4));
		}
	}
	for (unsigned s = 0; s < sizeof(vcbus) / sizeof(vcbus[0]); s++) {
		at = dump_section(out, at, "vcbus", vcbus[s].first, vcbus[s].count, 4);
		for (unsigned k = 0; k < vcbus[s].count; k++, at += 4) {
			put32(out + at, vpu(vcbus[s].first + k));
		}
	}
	at = dump_section(out, at, "top", TOP_BASE, 64, 4);
	for (unsigned k = 0; k < 64; k++, at += 4) {
		put32(out + at, top_rd(k * 4));
	}
	/* (COR 0xc00.. reads back 0xff through the SIP call, and a read at 0xe00 never returns: the CPU stays in EL3) */
	at = dump_section(out, at, "cor", COR_BASE, COR_SAFE_END, 1);
	for (unsigned k = 0; k < COR_SAFE_END; k++) {
		out[at++] = k == DDC_DATA_AON ? 0 : cor_rd(k);          /* (reading the DDC FIFO would take a byte out of it) */
	}
	at = dump_section(out, at, "frames", 0, 17 * 32, 1);
	for (unsigned s = 0; s < 17; s++) {
		cor_wr(TPI_INFO_FSEL, (uint8_t)s);
		for (unsigned k = 0; k < 31; k++) {
			out[at++] = cor_rd(TPI_INFO_B0 + k);
		}
		out[at++] = cor_rd(TPI_INFO_EN);
	}
	cor_wr(TPI_INFO_FSEL, fsel);
	n = hpd_high() ? 0x60 : 0;
	at = dump_section(out, at, "scdc", SCDC_ADDR, n, 1);
	for (unsigned k = 0; k < n; k++) {
		uint8_t v = 0xee;

		(void)scdc_read((uint8_t)k, &v);
		out[at++] = v;
	}
	return at;
}

static int
dump_sysctl(__unused struct sysctl_oid *oidp, void *arg1, __unused int arg2, struct sysctl_req *req)
{
	if (h.ana == 0) {
		return ENODEV;
	}
	if (arg1 != NULL) {                                     /* the one taken at boot */
		return h.bootdump_len ? SYSCTL_OUT(req, h.bootdump, h.bootdump_len) : ENOENT;
	}
	if (req->oldptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, NULL, DUMP_MAX);
	}
	lck_mtx_lock(&hdmitx_lock);
	h.dump_len = take_dump(h.dump);
	lck_mtx_unlock(&hdmitx_lock);
	return SYSCTL_OUT(req, h.dump, h.dump_len);
}

SYSCTL_PROC(_kern, OID_AUTO, hakios_hdmitx_dump, CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, dump_sysctl, "S", "the HDMI transmitter's registers now (tools/hdmitx-dump.py)");
SYSCTL_PROC(_kern, OID_AUTO, hakios_hdmitx_bootdump, CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_LOCKED,
    (void *)1, 0, dump_sysctl, "S", "the HDMI transmitter's registers as U-Boot left them (boot-arg hdmitx_bootdump=1)");

/* ---------------------------------------------------------------------------------------------------------------- */
/* Hot plug                                                                                                         */
/* ---------------------------------------------------------------------------------------------------------------- */

/* (the CEC driver, when it is built, replaces these) */
__attribute__((weak)) void
hakios_cec_hotplug(uint32_t physical_address __unused)
{
}

__attribute__((weak)) void
hakios_cec_request(int what __unused)
{
}

static struct {
	bool enabled;                           /* boot-arg hdmitx_hotplug (1) */
	bool need_set;                          /* set the mode once the line has been high long enough */
	unsigned high_polls;
	unsigned plugs;
	volatile int cmd;                       /* from sysctl, for the thread */
} hp;

/* a command for the hot-plug thread (1-9, as kern.hakios_hdmitx_cmd) */
static void
hp_request(int cmd)
{
	hp.cmd = cmd;
}

static void
scdc_status(void)
{
	uint8_t ver = 0, cfg = 0, st0 = 0, st1 = 0, err[7] = { 0 };

	(void)scdc_read(0x01, &ver);
	(void)scdc_read(SCDC_TMDS_CFG, &cfg);
	(void)scdc_read(SCDC_STATUS0, &st0);
	(void)scdc_read(SCDC_STATUS0 + 1, &st1);
	for (unsigned k = 0; k < 7; k++) {
		(void)scdc_read((uint8_t)(0x50 + k), &err[k]);
	}
	printf("hdmitx: SCDC: sink version %u, TMDS_CONFIG %#x, status %#x %#x (clock %s, channels locked %u%u%u), errors %u %u %u\n",
	    ver, cfg, st0, st1, st0 & 1 ? "detected" : "NOT detected", !!(st0 & 2), !!(st0 & 4), !!(st0 & 8),
	    (err[1] & 0x7f) << 8 | err[0], (err[3] & 0x7f) << 8 | err[2], (err[5] & 0x7f) << 8 | err[4]);
}

/* A TV is on the line: its EDID, its physical address to CEC, the mode. With the lock held. */
static void
plugged(void)
{
	bool edid = false;

	for (int k = 0; k < 5 && !(edid = read_edid()); k++) {
		IOSleep(200);
	}
	if (edid) {
		make_timings();
		hakios_cec_hotplug(h.physical_address);
	}
	if (h.mode.vic == 0 || (edid && !mode_ok(&h.mode))) {
		struct txmode was = h.mode;
		bool from_edid = select_mode(&h.mode);

		printf("hdmitx: mode %s: VIC %u, colour space %u, %u-bit%s (was VIC %u)\n", from_edid ? "for this TV" :
		    "without an EDID", h.mode.vic, h.mode.cs, h.mode.depth, h.mode.fractional ? ", 1/1.001" : "", was.vic);
		h.have_frames = false;                          /* (U-Boot's frames and PHY values were for its mode) */
		h.phy0 = 0;
		h.max_lncnt = 0;
	}
	(void)modeset(&h.mode);
	IOSleep(200);
	scdc_status();
	tell_listener();
}

static void
run_cmd(int cmd)
{
	lck_mtx_lock(&hdmitx_lock);
	switch (cmd) {
	case 1:
		show_state();
		break;
	case 2:
		if (read_edid()) {
			make_timings();
			hakios_cec_hotplug(h.physical_address);
		}
		break;
	case 3:
		plugged();
		break;
	case 4:
		output_off();
		break;
	case 7:
		scdc_status();
		break;
	case 8:
	case 9:
		h.yuv_blend = cmd == 8;
		if (output_alive()) {
			osd_yuv(h.yuv_blend);
			send_frames(&h.mode);
		}
		printf("hdmitx: %s\n", h.yuv_blend ? "OSD blended in YCbCr, Dolby Vision out of the path, SDR" : "U-Boot's Dolby Vision path");
		break;
	}
	lck_mtx_unlock(&hdmitx_lock);
	if (cmd == 5 || cmd == 6) {
		hakios_cec_request(cmd == 5 ? HAKIOS_CEC_STANDBY : HAKIOS_CEC_ONE_TOUCH_PLAY);
	}
}

static int
cmd_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int v = hp.cmd, error = sysctl_handle_int(oidp, &v, 0, req);

	if (error || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (h.ana == 0) {
		return ENODEV;
	}
	if (v < 1 || v > 9) {
		return EINVAL;
	}
	hp.cmd = v;                                             /* the hot-plug thread runs it */
	return 0;
}

SYSCTL_PROC(_kern, OID_AUTO, hakios_hdmitx_cmd, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, cmd_sysctl, "I", "HDMI: 1 state, 2 EDID, 3 set the mode, 4 output off, 5 CEC standby, 6 CEC one touch play, 7 SCDC, 8/9 OSD blend YCbCr/RGB");


/* "GAIN CHROMA": the OSD HDR core's SDR white and chroma gain, applied now if the core is on */
static int
osdhdr_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char in[32], out[32];
	unsigned long g, c, t;
	char *e, *f;
	int error;

	if (req->newptr == USER_ADDR_NULL) {
		snprintf(out, sizeof(out), "%u %#x %u", osd_hdr_gain, osd_hdr_chroma, osd_hdr_sat);
		return SYSCTL_OUT(req, out, strlen(out) + 1);
	}
	if (req->newlen >= sizeof(in)) {
		return E2BIG;
	}
	error = SYSCTL_IN(req, in, req->newlen);
	if (error) {
		return error;
	}
	in[req->newlen] = 0;
	g = strtoul(in, &e, 0);
	c = strtoul(e, &f, 0);
	t = f != e ? strtoul(f, &e, 0) : osd_hdr_sat;
	if (e == f) {
		t = osd_hdr_sat;        /* (no third number) */
	}
	if (g == 0 || g > 512 || c > 0xfff || t > 100) {
		return EINVAL;
	}
	lck_mtx_lock(&hdmitx_lock);
	osd_hdr_gain = (uint32_t)g;
	osd_hdr_chroma = c != 0 ? (uint32_t)c : osd_hdr_chroma;
	osd_hdr_sat = (uint32_t)t;
	if (osd_hdr_set) {
		osd_hdr(true);
	}
	lck_mtx_unlock(&hdmitx_lock);
	return 0;
}
/* "GAIN INSHIFT OUTSHIFT OFFSCALE": the video core's (above vd_hdr_gain), applied now if it is on */
static int
vdhdr_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char in[48], out[64];
	unsigned long v[5];
	char *p, *e;
	int error;

	if (req->newptr == USER_ADDR_NULL) {
		snprintf(out, sizeof(out), "%u %u %u %u %u (on %d, pq %d)", vd_hdr_gain, vd_in_shift, vd_out_shift, vd_off_scale,
		    vd_bits, vd_hdr_set, vd_pq);
		return SYSCTL_OUT(req, out, strlen(out) + 1);
	}
	if (req->newlen >= sizeof(in)) {
		return E2BIG;
	}
	error = SYSCTL_IN(req, in, req->newlen);
	if (error) {
		return error;
	}
	in[req->newlen] = 0;
	p = in;
	v[4] = vd_bits;
	for (unsigned k = 0; k < 5; k++, p = e) {
		v[k] = strtoul(p, &e, 0);
		if (e == p) {
			if (k == 4) {
				v[4] = vd_bits;         /* (no fifth number) */
				break;
			}
			return EINVAL;
		}
	}
	if (v[0] == 0 || v[0] > 512 || v[1] > 1 || v[2] > 1 || (v[3] != 1 && v[3] != 2 && v[3] != 4) ||
	    (v[4] != 10 && v[4] != 12)) {
		return EINVAL;
	}
	lck_mtx_lock(&hdmitx_lock);
	vd_hdr_gain = (uint32_t)v[0];
	vd_in_shift = (uint32_t)v[1];
	vd_out_shift = (uint32_t)v[2];
	vd_off_scale = (uint32_t)v[3];
	vd_bits = (uint32_t)v[4];
	if (vd_hdr_set) {
		hdr_core(VD1_HDR2_CTRL, true, true, &vd_hdr_set);
	}
	lck_mtx_unlock(&hdmitx_lock);
	return 0;
}
SYSCTL_PROC(_kern, OID_AUTO, hakios_hdmitx_vdhdr, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, vdhdr_sysctl, "A", "HDMI: the video HDR core's SDR white, input/output shifts, offset scale and width (10/12)");

SYSCTL_PROC(_kern, OID_AUTO, hakios_hdmitx_osdhdr, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, osdhdr_sysctl, "A", "HDMI: the OSD HDR core's SDR white (512 = 1.0) and chroma gain (0x400 = 1.0)");

/* "VIC CS DEPTH [FRAC]": the mode for the next mode set (cmd 3); CS 0 RGB, 1 4:2:2, 2 4:4:4, 3 4:2:0. For tests. */
static int
mode_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char in[64], out[64];
	int error;
	unsigned long v[4] = { 0 };
	unsigned n = 0;
	char *p = in, *e;

	if (req->newptr == USER_ADDR_NULL) {
		snprintf(out, sizeof(out), "%u %u %u %u", h.mode.vic, h.mode.cs, h.mode.depth, h.mode.fractional);
		return SYSCTL_OUT(req, out, strlen(out) + 1);
	}
	if (req->newlen >= sizeof(in)) {
		return E2BIG;
	}
	error = SYSCTL_IN(req, in, req->newlen);
	if (error) {
		return error;
	}
	in[req->newlen] = 0;
	if (strncmp(in, "auto", 4) == 0 || strncmp(in, "cap", 3) == 0) {
		lck_mtx_lock(&hdmitx_lock);
		if (in[0] == 'c') {
			h.max_lines = (unsigned)strtoul(in + 3, NULL, 10);
			make_timings();
		}
		h.mode.vic = 0;                 /* the next mode set (cmd 3) picks the best this TV takes */
		lck_mtx_unlock(&hdmitx_lock);
		return 0;
	}
	while (n < 4) {
		while (*p == ' ') {
			p++;
		}
		v[n] = strtoul(p, &e, 10);
		if (e == p) {
			break;
		}
		p = e;
		n++;
	}
	if (n < 3 || cta_index((uint8_t)v[0]) < 0 || v[1] > 3 || (v[2] != 8 && v[2] != 10 && v[2] != 12)) {
		return EINVAL;
	}
	lck_mtx_lock(&hdmitx_lock);
	h.mode.vic = (uint8_t)v[0];
	h.mode.cs = (uint8_t)v[1];
	h.mode.depth = (uint8_t)v[2];
	h.mode.fractional = n > 3 && v[3] != 0;
	h.mode.hdr = false;
	h.have_frames = false;          /* (U-Boot's infoframes are for its mode) */
	h.phy0 = 0;
	h.max_lncnt = 0;
	lck_mtx_unlock(&hdmitx_lock);
	return 0;
}

SYSCTL_PROC(_kern, OID_AUTO, hakios_hdmitx_mode, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, mode_sysctl, "A", "HDMI mode for the next mode set: \"VIC CS DEPTH [FRAC]\", \"auto\", \"cap LINES\" (act as a TV of at most LINES; 0 off)");

/* "c OFF [VAL]" a COR byte, "t OFF [VAL]" a TOP word, ';' between; a read gives the last command's values */
static char reg_out[512];

static int
reg_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char in[256];
	int error;

	if (req->newptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, reg_out, strlen(reg_out) + 1);
	}
	if (h.ana == 0) {
		return ENODEV;
	}
	if (req->newlen >= sizeof(in)) {
		return E2BIG;
	}
	error = SYSCTL_IN(req, in, req->newlen);
	if (error) {
		return error;
	}
	in[req->newlen] = 0;
	reg_out[0] = 0;
	lck_mtx_lock(&hdmitx_lock);
	for (char *p = in; *p;) {
		char what, *e;
		unsigned long off, val = 0;
		bool write;
		size_t at = strlen(reg_out);

		while (*p == ' ' || *p == ';' || *p == '\n') {
			p++;
		}
		if (*p == 0) {
			break;
		}
		what = *p++;
		off = strtoul(p, &e, 16);
		if (e == p || (what != 'c' && what != 't' && what != 'k' && what != 'a' && what != 'p' && what != 'v') ||
		    (what == 'v' ? (off < 0x1a00 || off > 0x3fff) :
		    off >= (what == 'c' ? (unsigned long)COR_SAFE_END : what == 't' ? 0x400ul : 0x1000ul))) {
			error = EINVAL;
			break;
		}
		p = e;
		while (*p == ' ') {
			p++;
		}
		write = *p && *p != ';';
		if (write) {
			val = strtoul(p, &e, 16);
			p = e;
		}
		if (what == 'c') {
			if (write) {
				cor_wr((uint32_t)off, (uint8_t)val);
			}
			snprintf(reg_out + at, sizeof(reg_out) - at, "%sc%03lx=%02x", at ? " " : "", off, cor_rd((uint32_t)off));
		} else if (what == 'v') {
			if (write) {
				vpu_wr((uint32_t)off, (uint32_t)val);
			}
			snprintf(reg_out + at, sizeof(reg_out) - at, "%sv%04lx=%08x", at ? " " : "", off, vpu((uint32_t)off));
		} else if (what == 'k' || what == 'a' || what == 'p') {
			vm_offset_t base = what == 'k' ? h.clk : what == 'a' ? h.ana : h.pwr;

			if (write) {
				wr(base, (uint32_t)off & ~3u, (uint32_t)val);
			}
			snprintf(reg_out + at, sizeof(reg_out) - at, "%s%c%03lx=%08x", at ? " " : "", what, off & ~3ul, rd(base, (uint32_t)off & ~3u));
		} else {
			if (write) {
				top_wr((uint32_t)off & ~3u, (uint32_t)val);
			}
			snprintf(reg_out + at, sizeof(reg_out) - at, "%st%03lx=%08x", at ? " " : "", off & ~3ul, top_rd((uint32_t)off & ~3u));
		}
	}
	lck_mtx_unlock(&hdmitx_lock);
	return error;
}

SYSCTL_PROC(_kern, OID_AUTO, hakios_hdmitx_reg, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, reg_sysctl, "A", "HDMI TX registers: \"c OFF [VAL]\" COR byte, \"t\"/\"k\"/\"a\"/\"p\" OFF [VAL] TOP/CLKCTRL/ANACTRL/PWRCTRL word, \"v\" VCBUS 0x1a00-0x3fff (hex), ';' between");
/*
 * The thread: every 100 ms the hot-plug line; 500 ms high after it was low (or at boot with no output) sets the mode.
 * While high: SCDC every second, as before (a TV that forgot scrambling is told again).
 */
static void
hotplug_thread(void *arg __unused, wait_result_t wr_ __unused)
{
	bool hpd_was = hpd_high();
	unsigned fixes = 0, tick = 0;

	printf("hdmitx: hot plug: line %s at start%s\n", hpd_was ? "high" : "low", hp.need_set ? ", mode set pending" : "");
	for (;; tick++) {
		bool hpd = hpd_high();
		int cmd = hp.cmd;

		if (cmd != 0) {
			hp.cmd = 0;
			run_cmd(cmd);
		}
		if (hpd != hpd_was) {
			printf("hdmitx: hot-plug %s\n", hpd ? "high: a TV" : "low: the TV let go");
			hpd_was = hpd;
			hp.high_polls = 0;
			if (!hpd && hp.enabled) {
				hp.need_set = true;
			}
		}
		if (hpd && hp.need_set && hp.enabled && ++hp.high_polls >= 5) {
			hp.need_set = false;
			printf("hdmitx: plug %u: setting the output up\n", ++hp.plugs);
			lck_mtx_lock(&hdmitx_lock);
			plugged();
			lck_mtx_unlock(&hdmitx_lock);
		}
		if (hpd && !hp.need_set && tick % 10 == 0 && output_alive()) {
			uint8_t cfg = 0, st = 0;

			lck_mtx_lock(&hdmitx_lock);
			if ((cor_rd(SCRCTL) & 1) && scdc_read(SCDC_TMDS_CFG, &cfg) && (cfg & 3) != 3) {
				bool ok = scdc_write(SCDC_TMDS_CFG, 0x03);

				(void)scdc_read(SCDC_STATUS0, &st);
				printf("hdmitx: the TV had forgotten scrambling (TMDS_CONFIG %#x): told again, %s; status %#x (%u)\n",
				    cfg, ok ? "acked" : "NOT acked", st, ++fixes);
			}
			lck_mtx_unlock(&hdmitx_lock);
		}
		IOSleep(100);
	}
}

static void
hotplug_start(void)
{
	uint32_t on = 1;
	thread_t thread;

	(void)PE_parse_boot_argn("hdmitx_scdc", &on, sizeof(on));
	if (on == 0) {
		return;
	}
	on = 1;
	(void)PE_parse_boot_argn("hdmitx_hotplug", &on, sizeof(on));
	hp.enabled = on != 0;
	if (kernel_thread_start(hotplug_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_set_thread_name(thread, "hdmitx-hotplug");
		thread_deallocate(thread);
	}
}

void
hakios_hdmitx_init(void)
{
	uint32_t on = 0, bootdump = 0, yuv = 1, hdr = 1;
	bool alive, edid = false;

	if (!PE_parse_boot_argn("hdmitx", &on, sizeof(on)) || on == 0) {
		return;
	}
	h.ana = ml_io_map(ANA_BASE, 0x1000);
	h.clk = ml_io_map(CLK_BASE, 0x1000);
	h.pad = ml_io_map(PAD_BASE, 0x1000);
	h.vpu = ml_io_map(VPU_BASE, 0x40000);
	h.pwr = ml_io_map(PWR_BASE, 0x1000);
	if (!h.ana || !h.clk || !h.pad || !h.vpu || !h.pwr) {
		printf("hdmitx: cannot map the registers\n");
		h.ana = 0;
		return;
	}
	lck_mtx_lock(&hdmitx_lock);
	(void)PE_parse_boot_argn("hdmitx_bootdump", &bootdump, sizeof(bootdump));
	if (bootdump) {
		h.bootdump_len = take_dump(h.bootdump);
	}
	show_state();
	(void)PE_parse_boot_argn("hdmitx_trace", &trace, sizeof(trace));
	(void)PE_parse_boot_argn("hdmitx_hdrmodes", &hdr_modes, sizeof(hdr_modes));
	(void)PE_parse_boot_argn("hdmitx_osdhdr", &osd_hdr_on, sizeof(osd_hdr_on));
	(void)PE_parse_boot_argn("hdmitx_osdgain", &osd_hdr_gain, sizeof(osd_hdr_gain));
	(void)PE_parse_boot_argn("hdmitx_osdchroma", &osd_hdr_chroma, sizeof(osd_hdr_chroma));
	(void)PE_parse_boot_argn("hdmitx_osdsat", &osd_hdr_sat, sizeof(osd_hdr_sat));
	(void)PE_parse_boot_argn("hdmitx_maxlines", &h.max_lines, sizeof(h.max_lines));
	(void)PE_parse_boot_argn("hdmitx_hdr", &hdr, sizeof(hdr));
	h.hdr10 = hdr != 0;
	(void)PE_parse_boot_argn("hdmitx_yuvblend", &yuv, sizeof(yuv));
	h.yuv_blend = yuv != 0;
	alive = snapshot_mode();
	h.dv_path = vpu(DOLBY_PATH_CTRL);
	h.dv_ctrl = vpu(VPP_DOLBY_CTRL);
	if (alive && h.yuv_blend) {
		osd_yuv(true);
		send_frames(&h.mode);
		printf("hdmitx: OSD blended in YCbCr, Dolby Vision cores out of the path (were %#x %#x), SDR\n", h.dv_path, h.dv_ctrl);
	}
	if (hpd_high()) {
		edid = read_edid();
		make_timings();
	}
	lck_mtx_unlock(&hdmitx_lock);
	if (edid) {
		tell_listener();                /* (the framebuffer may have published its modes before the EDID was read) */
	}
	/* no output from U-Boot (the TV was off at power-on): the mode is set as soon as the line is high */
	hp.need_set = !alive || (edid && !mode_ok(&h.mode));
	if (!alive) {
		printf("hdmitx: U-Boot left no output (hot plug %s at power-on?): setting it up when a TV is there\n",
		    hpd_high() ? "high now, low" : "low");
	} else if (hp.need_set) {
		printf("hdmitx: this TV does not list U-Boot's mode (VIC %u, colour space %u): a mode it takes is set\n", h.mode.vic,
		    h.mode.cs);
	}
	hotplug_start();
	hakios_hdmiaudio_init();
	hakios_vdec_init();
}

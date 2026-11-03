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

#define DDC_BUSLOW      (1u << 6)
#define DDC_NACK        (1u << 5)
#define DDC_INPROG      (1u << 4)

static struct {
	vm_offset_t ana, clk, pad, vpu;
	uint8_t edid[HAKIOS_HDMITX_EDID_MAX];
	unsigned edid_length;
	uint32_t physical_address;
} h = { .physical_address = 0xffff };

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
static uint8_t cor_rd(uint32_t off) { return (uint8_t)sip(SIP_READ, COR_BASE + off, 8, 0); }
static void cor_wr(uint32_t off, uint8_t v) { (void)sip(SIP_WRITE, COR_BASE + off, v, 8); }

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
		if (tag == 2) {                                                         /* video data block */
			for (unsigned k = 0; k < len && at < sizeof(vics) - 6; k++) {
				unsigned vic = p[k] >= 129 && p[k] <= 192 ? p[k] & 0x7f : p[k];

				at += snprintf(vics + at, sizeof(vics) - at, " %u%s", vic, p[k] >= 129 && p[k] <= 192 ? "*" : "");
			}
		} else if (tag == 3 && len >= 5 && p[0] == 0x03 && p[1] == 0x0c && p[2] == 0x00) {   /* HDMI 1.4 VSDB */
			h.physical_address = (uint32_t)p[3] << 8 | p[4];
			printf("hdmitx: HDMI VSDB: physical address %x.%x.%x.%x, deep colour %#x, max TMDS %u MHz\n", p[3] >> 4,
			    p[3] & 15, p[4] >> 4, p[4] & 15, len >= 6 ? p[5] : 0, len >= 7 ? p[6] * 5 : 0);
		} else if (tag == 3 && len >= 6 && p[0] == 0xd8 && p[1] == 0x5d && p[2] == 0xc4) {   /* HDMI Forum VSDB */
			printf("hdmitx: HF-VSDB: max TMDS %u MHz, SCDC %u, 4:2:0 deep colour %#x\n", p[4] * 5, !!(p[5] & 0x80),
			    len >= 7 ? p[6] & 7 : 0);
		} else if (tag == 7 && len >= 3 && p[0] == 0x06) {                      /* HDR static metadata */
			printf("hdmitx: HDR static metadata: EOTF SDR %u HDR-gamma %u PQ (HDR10) %u HLG %u, type 1 %u, max %u min %u\n",
			    p[1] & 1, !!(p[1] & 2), !!(p[1] & 4), !!(p[1] & 8), p[2] & 1, len >= 4 ? p[3] : 0, len >= 6 ? p[5] : 0);
		} else if (tag == 7 && len >= 1 && p[0] == 0x0e) {
			printf("hdmitx: YCbCr 4:2:0-only video data block (%u codes)\n", len - 1);
		} else if (tag == 7 && len >= 1 && p[0] == 0x0f) {
			printf("hdmitx: YCbCr 4:2:0 capability map (%u bytes)\n", len - 1);
		} else if (tag == 7 && len >= 2 && p[0] == 0x05) {
			printf("hdmitx: colorimetry: BT.2020 YCC %u RGB %u\n", !!(p[1] & 0x80), !!(p[1] & 0x40));
		} else if (tag == 3 && len >= 3 && p[0] == 0x46 && p[1] == 0xd0 && p[2] == 0x00) {
			printf("hdmitx: Dolby Vision VSVDB (%u bytes)\n", len);
		}
		i += 1 + len;
	}
	printf("hdmitx: video codes (* native):%s\n", vics);
	for (unsigned i = dtd; dtd >= 4 && i + 18 <= 127; i += 18) {
		show_dtd(b + i);
	}
}

static void
read_edid(void)
{
	uint8_t *b = h.edid;
	unsigned blocks;
	char name[14] = "";

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
}

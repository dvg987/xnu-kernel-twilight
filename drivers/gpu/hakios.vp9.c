/*
 * The Amlogic box's VP9 decoder (S7D's HEVC core, its "vp9_mmu" firmware), behind /dev/twl-vdec-probe
 * (hakios.vdec.c) with vdec=7. Profiles 0 and 2 up to 4096x2304, as Amlogic's vvp9.c drives the core: compressed
 * reference pictures (their body in 4 KiB pages the core's MMU finds through a map) and an 8-bit linear NV12 copy
 * of each picture ("double write"), the one players and the video plane get.
 *
 * Per frame: the core is reset, its work space and the frame (behind the 16-byte header the firmware looks for)
 * are set up, the firmware parses the frame's headers and stops. User space (tvos/video/vp9) keeps the reference
 * map and names the slots; everything else comes from here: sizes taken from a reference, the frame before's motion
 * vectors (two buffers in turn, apart from the slots), the probability contexts and their adaptation by the CPU
 * (hakios.vp9_prob.c), the loop filter's tables, and the segmentation quantizers, which the core forgets with each
 * reset and are read back after every frame.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <kern/clock.h>
#include <mach/vm_param.h>
#include <sys/hakios.vp9.h>
#include <sys/hakios.optee.h>
#include <sys/sysctl.h>
#include "hakios.vp9_regs.h"

extern void IOSleep(unsigned int milliseconds);
extern void IODelay(unsigned int microseconds);
extern void FlushPoC_DcacheRegion(vm_offset_t va, size_t length);

#define RD(r)           hakios_dos_rd(r)
#define WR(r, v)        hakios_dos_wr(r, v)
#define SET(r, m)       WR(r, RD(r) | (m))
#define CLEAR(r, m)     WR(r, RD(r) & ~(uint32_t)(m))
#define ALIGN(x, a)     (((x) + (a) - 1) & ~((uint64_t)(a) - 1))

/* the pool while a VP9 stream has it */
#define STREAM_OFF      0x00800000u
#define STREAM_SIZE     0x007f0000u
#define STREAM_PAD      0x00003000u
#define FRAME_HEADER    16u
#define MMU_MAP_OFF     0x00ff0000u
#define MMU_MAP_SIZE    0x00010000u
#define MV_OFF          0x01000000u

#define RPM_WORDS       TWL_VP9_RPM_WORDS
#define REFS            3

/* the firmware's states (HEVC_DEC_STATUS_REG) */
#define ST_DECODE_SLICE 0x05
#define ST_PIC_DONE     0x0a
#define ST_PIC_ERROR    0x0b
#define ST_NAL_DONE     0x0e
#define ST_BUFEMPTY     0x20
#define ST_TIMEOUT      0x21
#define ST_SEARCH_EMPTY 0x22
#define ST_OVER_SIZE    0x23
#define ST_HEAD_DONE    0xf0
#define ST_EOS          0xf2
#define ST_ACTION_DONE  0xff
#define MODE_FRAMEBASE  ((0x80u << 24) | 2)     /* DECODE_MODE_MULTI_FRAMEBASE */
#define ADAPT_REQUEST   0xfd                    /* VP9_ADAPT_PROB_REG [7:0]: the CPU is to adapt */
#define PROB_CUR        0x4000                  /* the frame's context in the probability buffer; [n * 0x1000]: saved */

enum { KEY_FRAME, INTER_FRAME };

#define HEVC_CONFIG_LITTLE_ENDIAN ((0xff0u << 8) | 0xfu)

/* the firmware's parse (vvp9.c union param_u) */
union param {
	uint16_t data[RPM_WORDS];
	struct {
		uint16_t profile, show_existing_frame, frame_to_show_idx, frame_type, show_frame, error_resilient_mode;
		uint16_t intra_only, display_size_present, reset_frame_context, refresh_frame_flags;
		uint16_t width, height, display_width, display_height;
		uint16_t ref_info;              /* [11:8], [7:4], [3:0]: LAST, GOLDEN, ALTREF: index << 1 | sign bias */
		uint16_t same_frame_size;       /* bit 2, 1, 0: LAST, GOLDEN, ALTREF's size */
		uint16_t mode_ref_delta_enabled, ref_deltas[4], mode_deltas[2], filter_level, sharpness_level;
		uint16_t bit_depth, seg_quant_info[8], seg_enabled, seg_abs_delta;
		uint16_t seg_lf_info[8];        /* bit 15: on; bit 8: negative; [5:0] */
	} p;
};

struct buf { uint32_t start, size; };

/* bring-up: the firmware's own trace (DECODE_STOP_POS: bit 0 its checkpoints, each acknowledged and logged), and with
 * bit 2 the registers when a frame stops short */
static int udebug;
/* the NV12 copy of each picture: 0 its full size up to 1080p, half (each way) up to 1440p and a quarter above (Amlogic's
 * double write modes 4 and 2; the plane shows the compressed picture, the copy is for the player's pixel buffers, seen
 * only while the window server draws the video itself, and only for a player that names no size of its own: the frames
 * say so, TWL_VP9_QUARTER); 1 always full size; 2 at most half */
static int dw_full;
SYSCTL_INT(_kern, OID_AUTO, hakios_vp9_dw_full, CTLFLAG_RW | CTLFLAG_LOCKED, &dw_full, 0, "");
/* the pool's room for compressed bodies, in percent of PAGES_RESERVE frames at their largest (from the next stream on) */
static int body_reserve = 100;
SYSCTL_INT(_kern, OID_AUTO, hakios_vp9_body_reserve, CTLFLAG_RW | CTLFLAG_LOCKED, &body_reserve, 0, "");
SYSCTL_INT(_kern, OID_AUTO, hakios_vp9_udebug, CTLFLAG_RW | CTLFLAG_LOCKED, &udebug, 0, "");
/* bring-up: registers written after the set-up, "REG=V" (write), "REG|V" (set bits), "REG&V" (keep bits), "rREG" (log
 * it), "dN" (wait N microseconds); hex DOS word indexes and values */
static char pokes[256];
SYSCTL_STRING(_kern, OID_AUTO, hakios_vp9_poke, CTLFLAG_RW | CTLFLAG_LOCKED, pokes, sizeof(pokes), "");

static uint32_t
hex(const char **at)
{
	uint32_t v = 0;
	for (;; (*at)++) {
		char c = **at;
		if (c >= '0' && c <= '9') v = v << 4 | (uint32_t)(c - '0');
		else if (c >= 'a' && c <= 'f') v = v << 4 | (uint32_t)(c - 'a' + 10);
		else if (c == 'x' || c == 'X') v = 0;
		else return v;
	}
}

static void
run_script(const char *script, const char *tag)
{
	for (const char *at = script; *at;) {
		while (*at == ' ') at++;
		if (!*at) break;
		if (*at == 'r') {
			at++;
			uint32_t reg = hex(&at);
			printf("vp9: %s: %04x = %08x\n", tag, reg, hakios_dos_rd(reg));
			continue;
		}
		if (*at == 'd') {
			at++;
			IODelay(hex(&at));
			continue;
		}
		uint32_t reg = hex(&at);
		char op = *at;
		if (op != '=' && op != '|' && op != '&') break;
		at++;
		uint32_t value = hex(&at);
		if (op == '|') value |= hakios_dos_rd(reg);
		else if (op == '&') value &= hakios_dos_rd(reg);
		hakios_dos_wr(reg, value);
	}
}

static void
apply_pokes(void)
{
	run_script(pokes, "poke");
}

static struct {
	bool fw_loaded, layout, header_pending, running, list_written;
	vm_offset_t pool;
	uint64_t pool_pa;
	uint32_t pool_size;
	struct {
		struct buf ipp, sao_abv, sao_vb, short_term_rps, vps, sps, pps, sao_up, swap_buf, swap_buf2, scalelut,
		    dblk_para, dblk_data, seg_map, mmu_vbh, mpred_above, rpm, lmem, prob, count;
	} ws;
	union param param;
	uint32_t input_size;
	/* the stream */
	int pic_w, pic_h;               /* what the slots were laid out for */
	uint32_t n_pics, stride, rows, uv_offset, cm_header, frame_bytes, mv_bytes;
	int dw, dw_w, dw_h;             /* the copies' scale (1 or 2) and size */
	uint32_t mv[2];
	struct { uint32_t cm, y, uv; int w, h, depth; } pic[TWL_VP9_SLOTS];
	uint32_t seg_quant[8];          /* VP9_QUANT_WR, kept across the core's resets */
	int bit_depth;                  /* the last key or intra-only frame's */
	uint32_t slice_idx;             /* the frames the firmware has run (HEVC_DECODE_COUNT) */
	/* libvpx's VP9_COMMON, what of it the next frame depends on */
	int frame_type, last_frame_type, intra_only, last_intra_only, show_frame, last_show_frame;
	int width, height, last_width, last_height, pic_count;
	uint32_t decoded;               /* frames decoded: the motion vector buffers go in turn */
	int sharpness;                  /* the loop filter's limits as last written */
} V;

static uint32_t
pa(uint32_t offset)
{
	return (uint32_t)(V.pool_pa + offset);
}

static vm_offset_t
va(uint32_t phys)
{
	return V.pool + (phys - (uint32_t)V.pool_pa);
}

static void
pool_zero(uint32_t offset, uint32_t size)
{
	bzero((void *)(V.pool + offset), size);
	FlushPoC_DcacheRegion(V.pool + offset, size);
}

/* ---- sizes (vvp9.c) ---- */

static uint32_t
comp_body_size(int w, int h, bool ten)
{
	return (uint32_t)((ten ? 4096 : 3200) * ((w + 63) >> 6) * ((h + 31) >> 5));
}

static uint32_t
comp_header_size(int w, int h)
{
	return (uint32_t)(32 * ((w + 127) >> 7) * ((h + 63) >> 6));
}

static uint32_t
mv_buf_size(int w, int h)
{
	int wl = (((w + 63) & ~63) + 63) / 64, hl = (((h + 31) & ~31) + 63) / 64;
	return (uint32_t)ALIGN(wl * hl * 36 * 16 + wl * 16 * ((hl >> 3) + (hl & 7)), 0x10000);
}

static uint32_t
uv_pages(int w, int h)
{
	unsigned lw = (w + 63) / 64, lh = (h + 63) / 64;
	return ((lw * lh * 64 * 64 / 2) + 0xfff) >> 12;
}

/* ---- the core ---- */

static void
layout_workspace(void)
{
	uint32_t at = 0;
	struct { struct buf *b; uint32_t size; } spec[] = {     /* amvvp9_workbuff_spec[1] (4K), init_buff_spec's order */
		{ &V.ws.ipp, 0x4000 }, { &V.ws.sao_abv, 0x30000 }, { &V.ws.sao_vb, 0x30000 }, { &V.ws.short_term_rps, 0x800 },
		{ &V.ws.vps, 0x800 }, { &V.ws.sps, 0x800 }, { &V.ws.pps, 0x2000 }, { &V.ws.sao_up, 0x2800 },
		{ &V.ws.swap_buf, 0x800 }, { &V.ws.swap_buf2, 0x800 }, { &V.ws.scalelut, 0x8000 },
		{ &V.ws.dblk_para, 0x80000 }, { &V.ws.dblk_data, 0x80000 }, { &V.ws.seg_map, 0xd800 },
		{ &V.ws.mmu_vbh, 0x5000 }, { &V.ws.mpred_above, 0x10000 }, { &V.ws.rpm, RPM_WORDS * 2 },
		{ &V.ws.lmem, 0x400 * 2 }, { &V.ws.prob, 0x5000 }, { &V.ws.count, 0x300 * 4 * 4 },
	};
	for (unsigned i = 0; i < sizeof(spec) / sizeof(spec[0]); i++) {
		at = (uint32_t)ALIGN(at, PAGE_SIZE);
		spec[i].b->start = pa(at);
		spec[i].b->size = spec[i].size;
		at += spec[i].size;
	}
}

static const uint16_t parser_cmd[37] = {
	0x0401, 0x8401, 0x0800, 0x0402, 0x9002, 0x1423, 0x8CC3, 0x1423, 0x8804, 0x9825, 0x0800, 0x04FE,
	0x8406, 0x8411, 0x1800, 0x8408, 0x8409, 0x8C2A, 0x9C2B, 0x1C00, 0x840F, 0x8407, 0x8000, 0x8408,
	0x2000, 0xA800, 0x8410, 0x04DE, 0x840C, 0x840D, 0xAC00, 0xA000, 0x08C0, 0x08E0, 0xA40E, 0xFC00,
	0x7C00
};

/* vp9_update_sharpness + the limits' half of vp9_loop_filter_init / vp9_loop_filter_frame_init */
static void
write_lf_limits(int sharpness)
{
	uint8_t lim[64], mblim[64];
	for (int lvl = 0; lvl < 64; lvl++) {
		int inside = lvl >> ((sharpness > 0) + (sharpness > 4));
		if (sharpness > 0 && inside > 9 - sharpness) inside = 9 - sharpness;
		if (inside < 1) inside = 1;
		lim[lvl] = (uint8_t)inside;
		mblim[lvl] = (uint8_t)(2 * (lvl + 2) + inside);
	}
	for (int i = 0; i < 32; i++) {
		uint32_t thr = ((lim[i * 2 + 1] & 0x3fu) << 8) | mblim[i * 2 + 1];
		WR(HEVC_DBLK_CFG9, (thr << 16) | ((lim[i * 2] & 0x3fu) << 8) | mblim[i * 2]);
	}
	V.sharpness = sharpness;
}

/* init_pic_list_hw: the slots' compressed headers, by slot, for the motion compensation's reads */
static void
init_pic_list_hw(void)
{
	WR(HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, (1 << 1) | (1 << 2));
	for (uint32_t i = 0; i < V.n_pics; i++) WR(HEVCD_MPP_ANC2AXI_TBL_DATA, V.pic[i].cm >> 5);
	WR(HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);
	WR(HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	for (int i = 0; i < 32; i++) WR(HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);
	V.list_written = true;
}

/* vvp9_prot_init: vp9_config_work_space_hw, init_pic_list_hw, vp9_init_decoder_hw, vp9_loop_filter_init, front */
static void
prot_init(void)
{
	int w = V.layout ? V.pic_w : 1920, h = V.layout ? V.pic_h : 1088;
	WR(HEVC_RPM_BUFFER, V.ws.rpm.start);
	WR(HEVC_SHORT_TERM_RPS, V.ws.short_term_rps.start);
	WR(HEVC_PPS_BUFFER, V.ws.pps.start);
	WR(HEVC_STREAM_SWAP_BUFFER, V.ws.swap_buf.start);
	WR(HEVC_STREAM_SWAP_BUFFER2, V.ws.swap_buf2.start);
	WR(LMEM_DUMP_ADR, V.ws.lmem.start);
	WR(HEVCD_IPP_LINEBUFF_BASE, V.ws.ipp.start);
	WR(HEVC_DBLK_CFGE, V.ws.dblk_para.start);
	WR(HEVC_DBLK_CFG4, V.ws.dblk_para.start);
	WR(HEVC_DBLK_CFG5, V.ws.dblk_data.start);
	WR(HEVC_DBLK_CFG3, 0x404010);
	WR(HEVCD_MPP_DECOMP_CTL1, 1u << 4);             /* paged (MMU) compressed pictures */
	uint32_t body = comp_body_size(w, h, true);
	WR(HEVC_CM_BODY_LENGTH, body);
	WR(HEVC_CM_HEADER_OFFSET, body);
	WR(HEVC_CM_HEADER_LENGTH, comp_header_size(w, h));
	WR(HEVC_SAO_MMU_VH0_ADDR, V.ws.mmu_vbh.start);
	WR(HEVC_SAO_MMU_VH1_ADDR, V.ws.mmu_vbh.start + V.ws.mmu_vbh.size / 2);
	SET(HEVC_SAO_CTRL5, 1u << 10);                  /* HEVC_CM_HEADER_START_ADDR names the header */
	WR(HEVC_MPRED_CTRL3, 0x24122412);
	SET(HEVC_MPRED_CTRL4, 1u << 1);                 /* compressed co-located motion vectors */
	WR(VP9_SEG_MAP_BUFFER, V.ws.seg_map.start);
	WR(VP9_PROB_SWAP_BUFFER, V.ws.prob.start);
	WR(VP9_COUNT_SWAP_BUFFER, V.ws.count.start);
	WR(HEVC_ASSIST_MMU_MAP_ADDR, pa(MMU_MAP_OFF));
	if (V.layout) init_pic_list_hw();

	WR(HEVC_SHIFT_STARTCODE, 0x00000001);
	WR(HEVC_SHIFT_EMULATECODE, 0x00000300);
	WR(HEVC_PARSER_INT_CONTROL, (RD(HEVC_PARSER_INT_CONTROL) & ~(7u << 29)) | (3u << 29) | (1 << 24) | (1 << 22) |
	    (1 << 7) | (1 << 4) | (1 << 0));
	WR(HEVC_SHIFT_STATUS, (RD(HEVC_SHIFT_STATUS) & ~2u) | 1);      /* start codes checked, no emulation prevention */
	WR(HEVC_CABAC_CONTROL, 1);
	WR(HEVC_PARSER_CORE_CONTROL, 1);
	WR(HEVC_STREAM_SWAP_TEST, 0);
	WR(HEVC_SHIFT_CONTROL, (1 << 10) | (1 << 9) | (3 << 6) | (2 << 4) | (3 << 1) | (1 << 0));
	WR(HEVC_DEC_STATUS_REG, 0);
	CLEAR(HEVC_EFFICIENCY_MODE, 1);
	WR(HEVC_IQIT_SCALELUT_WR_ADDR, 0);
	for (int i = 0; i < 1024; i++) WR(HEVC_IQIT_SCALELUT_DATA, 0);
	WR(DECODE_MODE, MODE_FRAMEBASE);
	WR(HEVC_DECODE_SIZE, 0);
	WR(HEVC_DECODE_COUNT, 0);
	WR(HEVC_PARSER_CMD_WRITE, 1u << 16);
	for (int i = 0; i < 37; i++) WR(HEVC_PARSER_CMD_WRITE, parser_cmd[i]);
	WR(HEVC_PARSER_CMD_SKIP_0, 0x0000090b);
	WR(HEVC_PARSER_CMD_SKIP_1, 0x1b14140f);
	WR(HEVC_PARSER_CMD_SKIP_2, 0x001b1910);
	WR(HEVC_PARSER_IF_CONTROL, (1 << 5) | (1 << 2) | (1 << 0));
	WR(HEVCD_IPP_TOP_CNTL, 1 << 0);
	WR(HEVCD_IPP_TOP_CNTL, 1 << 1);

	write_lf_limits(0);
	WR(HEVC_DBLK_CFGB, (3u << 14) | (3u << 12) | (3u << 10) | (3u << 8) | 1);     /* both writes, VP9 */
	WR(HEVCD_MPP_DECOMP_AXIURG_CTL, 0x1 | (0x1 << 2) | (0x1 << 3) | (24 << 4) | (32 << 11) | (24 << 18) | (32u << 25));
	WR(HEVCD_IPP_DYN_CACHE, 0x2b);

	WR(HEVC_WAIT_FLAG, 1);
	WR(HEVC_ASSIST_MBOX0_CLR_REG, 1);
	WR(HEVC_ASSIST_MBOX0_MASK, 1);
	WR(HEVC_PSCALE_CTRL, 0);
	WR(DEBUG_REG1, 0);
	WR(NAL_SEARCH_CTL, 0x8);
	WR(DECODE_STOP_POS, (uint32_t)udebug & 1);
}

static void
core_stop(void)
{
	WR(HEVC_MPSR, 0);
	WR(HEVC_CPSR, 0);
	for (int i = 0; i < 2000 && (RD(HEVC_IMEM_DMA_CTRL) & 0x8000); i++) IODelay(10);
	for (int i = 0; i < 2000 && (RD(HEVC_LMEM_DMA_CTRL) & 0x8000); i++) IODelay(10);
	for (int i = 0; i < 2000 && (RD(HEVC_WRRSP_LMEM) & 0xfff); i++) IODelay(10);
	V.running = false;
}

static void
read_rpm(void)
{
	vm_offset_t rpm = va(V.ws.rpm.start);
	FlushPoC_DcacheRegion(rpm, RPM_WORDS * 2);
	const uint16_t *raw = (const uint16_t *)rpm;
	for (unsigned i = 0; i < RPM_WORDS; i += 4)
		for (unsigned k = 0; k < 4; k++) V.param.data[i + k] = raw[i + 3 - k];
}

/* the firmware asks the CPU to adapt the frame's probabilities (vvp9_isr) */
static void
adapt_probs(uint32_t status)
{
	uint32_t saved = (status >> 8) & 3;
	vm_offset_t prob = va(V.ws.prob.start), count = va(V.ws.count.start);
	unsigned int *prev = (unsigned int *)(prob + saved * 0x1000), *cur = (unsigned int *)(prob + PROB_CUR);
	FlushPoC_DcacheRegion(prob, V.ws.prob.size);
	FlushPoC_DcacheRegion(count, V.ws.count.size);
	int pre_fc = V.frame_type == KEY_FRAME || V.intra_only;
	hakios_vp9_adapt_probs(V.pic_count, V.last_frame_type == KEY_FRAME, pre_fc, (int)(status >> 8), prev, cur,
	    (unsigned int *)count);
	memcpy(prev, cur, HAKIOS_VP9_PROB_SIZE);
	FlushPoC_DcacheRegion(prob, V.ws.prob.size);
	WR(VP9_ADAPT_PROB_REG, 0);
	V.pic_count++;
}

/* bring-up (udebug & 4): the parser's, the assist's and VP9's registers, and where the firmware loops */
static void
dump_regs(void)
{
	static const struct { uint32_t from, n; } ranges[] = {
		{ 0x3000, 0x20 }, { 0x30c0, 0x20 }, { 0x3100, 0x40 }, { 0x3140, 0x10 }, { 0x3170, 0x10 }, { 0x3300, 0x10 },
	};
	for (unsigned r = 0; r < sizeof(ranges) / sizeof(ranges[0]); r++)
		for (uint32_t i = 0; i < ranges[r].n; i += 8) {
			uint32_t a = ranges[r].from + i;
			printf("vp9: %04x: %08x %08x %08x %08x %08x %08x %08x %08x\n", a, hakios_dos_rd(a), hakios_dos_rd(a + 1),
			    hakios_dos_rd(a + 2), hakios_dos_rd(a + 3), hakios_dos_rd(a + 4), hakios_dos_rd(a + 5),
			    hakios_dos_rd(a + 6), hakios_dos_rd(a + 7));
		}
	uint32_t pcs[32];
	for (int i = 0; i < 32; i++) { pcs[i] = RD(HEVC_MPC_E); IODelay(3); }
	printf("vp9: PCs");
	for (int i = 0; i < 32; i++) printf(" %x", pcs[i]);
	printf("\n");
}

/* the state the firmware stops in: the header parsed (0xf0), the frame done (0xa), the data used up, an error */
static uint32_t
wait_status(void)
{
	uint64_t start = mach_absolute_time(), limit;
	nanoseconds_to_absolutetime(2000ull * NSEC_PER_MSEC, &limit);
	unsigned traced = 0;
	for (unsigned spins = 0;; spins++) {
		uint32_t seq = hakios_vdec_irq_seq();
		uint32_t tag = (udebug & 1) ? RD(DEBUG_REG1) : 0;
		if (tag && RD(HEVC_MPC_E) > 0x100) {
			if (traced++ < 48) {
				printf("vp9: ucode %#x %#x: status %#x, shift %u, LCU %#x, PC %#x\n", tag, RD(DEBUG_REG2),
				    RD(HEVC_DEC_STATUS_REG), RD(HEVC_SHIFT_BYTE_COUNT), RD(HEVC_PARSER_LCU_START), RD(HEVC_MPC_E));
			}
			WR(DEBUG_REG1, 0);
			continue;
		}
		uint32_t adapt = RD(VP9_ADAPT_PROB_REG);
		if ((adapt & 0xff) == ADAPT_REQUEST) adapt_probs(adapt);
		uint32_t s = RD(HEVC_DEC_STATUS_REG);
		if (s == ST_HEAD_DONE || s == ST_PIC_DONE || s == ST_PIC_ERROR || s == ST_NAL_DONE || s == ST_BUFEMPTY ||
		    s == ST_TIMEOUT || s == ST_SEARCH_EMPTY || s == ST_OVER_SIZE || s == ST_EOS) {
			return s;
		}
		if (mach_absolute_time() - start > limit) {
			printf("vp9: stuck in status %#x: PC %#x, LCU %#x, shift %u, adapt %#x\n", s, RD(HEVC_MPC_E),
			    RD(HEVC_PARSER_LCU_START), RD(HEVC_SHIFT_BYTE_COUNT), adapt);
			if (udebug & 4) dump_regs();
			return 0;
		}
		if (hakios_vdec_irq_nap(seq, 250)) continue;
		if (spins < 400) IODelay(25); else IOSleep(1);
	}
}

/* ---- the stream's pictures ---- */

/*
 * The compressed pictures' bodies, in 4 KiB pages through the core's MMU (vvp9.c's mmu box): a frame gets its largest
 * size before it is decoded, keeps what the core says it used (HEVC_SAO_MMU_STATUS) and loses it all once no
 * reference names its slot; the copies the plane and the player see are the slots' own memory.
 */
#define PAGE_4K         0x1000u
#define PAGES_RESERVE   6               /* largest bodies the page area is laid out for, at the least */

static struct {
	uint32_t base;                  /* pool offset */
	uint32_t total, top, *free;     /* a stack of free page numbers (pool page index) */
	uint32_t per_slot, *list;       /* each slot's pages: list[slot * per_slot ...] */
	uint32_t count[TWL_VP9_SLOTS];
	uint32_t free_size, list_size;  /* bytes allocated */
} P;

static void
pages_release(void)
{
	if (P.free) kfree_data(P.free, P.free_size);
	if (P.list) kfree_data(P.list, P.list_size);
	bzero(&P, sizeof(P));
}

static bool
pages_setup(uint32_t base, uint32_t total, uint32_t slots, uint32_t per_slot)
{
	P.free_size = total * 4;
	P.list_size = slots * per_slot * 4;
	P.free = kalloc_data(P.free_size, Z_WAITOK);
	P.list = kalloc_data(P.list_size, Z_WAITOK);
	if (P.free == NULL || P.list == NULL) {
		pages_release();
		return false;
	}
	P.base = base;
	P.total = P.top = total;
	P.per_slot = per_slot;
	for (uint32_t i = 0; i < total; i++) P.free[i] = total - 1 - i;
	return true;
}

static void
pages_trim(uint32_t slot, uint32_t keep)
{
	while (P.count[slot] > keep) P.free[P.top++] = P.list[slot * P.per_slot + --P.count[slot]];
}

static bool
pages_take(uint32_t slot, uint32_t n)
{
	pages_trim(slot, 0);
	if (n > P.per_slot || n > P.top) return false;
	for (uint32_t i = 0; i < n; i++) P.list[slot * P.per_slot + i] = P.free[--P.top];
	P.count[slot] = n;
	return true;
}

static bool
layout_pictures(int w, int h, bool quarter)
{
	/* (a 4K copy at 1920x1080 in each of 24 slots was 75 MiB the compressed bodies of busy scenes needed) */
	V.dw = dw_full == 1 || (w <= 1920 && h <= 1088) ? 1 : quarter && dw_full == 0 && (w > 2560 || h > 1440) ? 4 : 2;
	V.dw_w = w / V.dw;
	V.dw_h = h / V.dw;
	uint32_t uvp = uv_pages(V.dw_w, V.dw_h);
	pages_release();
	V.pic_w = w;
	V.pic_h = h;
	V.stride = (uint32_t)ALIGN(V.dw_w, 64);
	V.rows = (uint32_t)ALIGN(V.dw_h, 64);
	V.uv_offset = uvp << 13;
	uint32_t worst = (comp_body_size(w, h, true) + 0xfff) >> 12;    /* a 10-bit frame's body at most, in pages */
	V.cm_header = (uint32_t)(ALIGN(w, 64) * ALIGN(h, 64) > 1920 * 1088 ? 0x48000 : 0x10000);
	V.frame_bytes = V.cm_header + (uint32_t)ALIGN(V.uv_offset + (uvp << 12), 0x10000);
	V.mv_bytes = mv_buf_size(w, h);
	/* a slot: the compressed picture's header and the copy (NV12); the compressed bodies in pages of their own, enough
	 * for the frame being decoded and its references at their largest; the rest of the pool more slots */
	uint32_t frames_off = MV_OFF + 2 * V.mv_bytes;
	uint32_t reserve = (uint32_t)((uint64_t)PAGES_RESERVE * worst * PAGE_4K * body_reserve / 100);
	if (reserve < 2 * worst * PAGE_4K) reserve = 2 * worst * PAGE_4K;
	uint32_t n = V.pool_size > frames_off + reserve ? (V.pool_size - frames_off - reserve) / V.frame_bytes : 0;
	if (n > TWL_VP9_SLOTS) n = TWL_VP9_SLOTS;
	if (n < 10 || worst * 4 > MMU_MAP_SIZE) {
		printf("vp9: %dx%d needs %u bytes a picture and %u for the bodies; the pool (vdec_pool_mb) holds %u\n", w, h,
		    V.frame_bytes, reserve, n);
		return false;
	}
	uint32_t pages_off = ALIGN(frames_off + n * V.frame_bytes, PAGE_4K);
	uint32_t n_pages = (V.pool_size - pages_off) / PAGE_4K;
	if (!pages_setup(pages_off, n_pages, n, worst)) {
		printf("vp9: no memory for the page lists\n");
		return false;
	}
	V.n_pics = n;
	V.mv[0] = pa(MV_OFF);
	V.mv[1] = pa(MV_OFF + V.mv_bytes);
	uint32_t at = frames_off;
	for (uint32_t i = 0; i < n; i++) {
		V.pic[i].cm = pa(at);
		V.pic[i].y = pa(at + V.cm_header);
		V.pic[i].uv = V.pic[i].y + V.uv_offset;
		V.pic[i].w = V.pic[i].h = 0;
		at += V.frame_bytes;
	}
	pool_zero(MV_OFF, at - MV_OFF);
	V.layout = true;
	printf("vp9: %dx%d, %u pictures of %#x (copies %dx%d), %u body pages (%u a frame at most), motion vectors 2 x %#x "
	    "(stride %u, rows %u)\n", w, h, n, V.frame_bytes, V.dw_w, V.dw_h, n_pages, worst, V.mv_bytes, V.stride, V.rows);
	return true;
}

/* ---- one frame's set-up (vvp9.c continue_decoding) ---- */

static void
config_pic_size(int w, int h, int depth)
{
	uint32_t body = comp_body_size(w, h, depth == 10);
	if (depth == 10) CLEAR(HEVC_SAO_CTRL5, 1u << 9); else SET(HEVC_SAO_CTRL5, 1u << 9);
	WR(HEVCD_MPP_DECOMP_CTL1, 1u << 4);
	WR(HEVC_CM_BODY_LENGTH, body);
	WR(HEVC_CM_HEADER_OFFSET, body);
	WR(HEVC_CM_HEADER_LENGTH, comp_header_size(w, h));
}

static void
config_mc_buffer(const struct twl_vp9_frame *f, int w, int h)
{
	uint32_t scale = 0;
	for (int l = 0; l < 2; l++) {
		WR(HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, ((l ? 16u : 0u) << 8) | 1);
		for (int i = 0; i < REFS; i++) {
			uint32_t s = f->ref_slot[i];
			WR(HEVCD_MPP_ANC_CANVAS_DATA_ADDR, (s << 16) | (s << 8) | s);
		}
	}
	WR(VP9D_MPP_REFINFO_TBL_ACCCONFIG, 1u << 2);
	for (int i = 0; i < REFS; i++) {
		int rw = V.pic[f->ref_slot[i]].w, rh = V.pic[f->ref_slot[i]].h;
		WR(VP9D_MPP_REFINFO_DATA, (uint32_t)rw);
		WR(VP9D_MPP_REFINFO_DATA, (uint32_t)rh);
		if (rw != w || rh != h) scale |= 1u << i;
		WR(VP9D_MPP_REFINFO_DATA, (uint32_t)((rw << 14) / w));
		WR(VP9D_MPP_REFINFO_DATA, (uint32_t)((rh << 14) / h));
		WR(VP9D_MPP_REFINFO_DATA, 0);
	}
	WR(VP9D_MPP_REF_SCALE_ENBL, scale);
}

static void
clear_mpred(void)
{
	WR(HEVC_MPRED_CTRL3, 0x24122412);
	CLEAR(HEVC_MPRED_CTRL4, 1u << 6);
	CLEAR(HEVC_MPRED_CTRL0, 3u << 9);
}

static void
config_mpred(int use_prev)
{
	uint32_t cur = V.mv[V.decoded & 1], prev = V.mv[(V.decoded + 1) & 1];
	WR(HEVC_MPRED_CTRL3, 0x24122412);
	WR(HEVC_MPRED_ABV_START_ADDR, V.ws.mpred_above.start);
	WR(HEVC_MPRED_CTRL4, (RD(HEVC_MPRED_CTRL4) & ~(1u << 6)) | ((uint32_t)use_prev << 6));
	WR(HEVC_MPRED_MV_WR_START_ADDR, cur);
	WR(HEVC_MPRED_MV_WPTR, cur);
	WR(HEVC_MPRED_MV_RD_START_ADDR, prev);
	WR(HEVC_MPRED_MV_RPTR, prev);
	WR(HEVC_MPRED_MV_RD_END_ADDR, prev + V.mv_bytes);
}

static void
config_mcrcc(bool inter)
{
	WR(HEVCD_MCRCC_CTL1, 0x2);
	if (!inter) {
		WR(HEVCD_MCRCC_CTL1, 0x0);
		return;
	}
	WR(HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1 << 1);
	uint32_t d = RD(HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
	WR(HEVCD_MCRCC_CTL2, d | d << 16);
	d = RD(HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
	WR(HEVCD_MCRCC_CTL3, d | d << 16);
	WR(HEVCD_MCRCC_CTL1, 0xff0);
}

static void
config_sao(uint32_t slot)
{
	uint32_t uvp = uv_pages(V.dw_w, V.dw_h), d;
	WR(HEVC_SAO_Y_START_ADDR, V.pic[slot].y);
	WR(HEVC_SAO_C_START_ADDR, V.pic[slot].uv);
	WR(HEVC_SAO_Y_WPTR, V.pic[slot].y);
	WR(HEVC_SAO_C_WPTR, V.pic[slot].uv);
	WR(HEVC_CM_HEADER_START_ADDR, V.pic[slot].cm);
	WR(HEVC_SAO_Y_LENGTH, uvp << 13);
	WR(HEVC_SAO_C_LENGTH, uvp << 12);
	d = RD(HEVC_SAO_CTRL1) & ~((3u << 14) | 0x3000u | 0xff0u | 0x3u);
	d |= (2u << 14) | ((HEVC_CONFIG_LITTLE_ENDIAN >> 8) & 0xfff);      /* linear, 64-byte lines; both writes */
	WR(HEVC_SAO_CTRL1, d);
	WR(HEVC_DBLK_CFGB, RD(HEVC_DBLK_CFGB) | (1u << 8) | (1u << 9));
	WR(HEVC_SAO_CTRL26, 0);
	/* the copy full size, half each way ([23:16] dw_v1 dw_v0 dw_h1 dw_h0: 0x33, double write mode 4) or a quarter (0xff, 2) */
	WR(HEVC_SAO_CTRL5, (RD(HEVC_SAO_CTRL5) & ~(0xffu << 16)) | (V.dw == 4 ? 0xffu << 16 : V.dw == 2 ? 0x33u << 16 : 0));
	d = RD(HEVCD_IPP_AXIIF_CONFIG) & ~(0x30u | 0xfu | (3u << 8));
	WR(HEVCD_IPP_AXIIF_CONFIG, d | (HEVC_CONFIG_LITTLE_ENDIAN & 0xf) | (2u << 8));
}

/* vp9_loop_filter_frame_init: the filter level of each segment, reference and mode */
static void
config_loop_filter(int w)
{
	const union param *rp = &V.param;
	int level = rp->p.filter_level, scale = 1 << (level >> 5);
	uint8_t lvl[8][4][2];
	if (rp->p.sharpness_level != V.sharpness) write_lf_limits(rp->p.sharpness_level);
	uint32_t cfgb = RD(HEVC_DBLK_CFGB);
	WR(HEVC_DBLK_CFGB, w >= 1280 ? cfgb | (1u << 4) : cfgb & ~(3u << 4));
	for (int seg = 0; seg < 8; seg++) {
		int lvl_seg = level;
		uint16_t info = rp->p.seg_lf_info[seg];
		if (rp->p.seg_enabled && (info & 0x8000)) {
			int data = (info & 0x100) ? -(int)(info & 0x3f) : (int)(info & 0x3f);
			lvl_seg = rp->p.seg_abs_delta ? data : level + data;
			lvl_seg = lvl_seg < 0 ? 0 : lvl_seg > 63 ? 63 : lvl_seg;
		}
		if (!rp->p.mode_ref_delta_enabled) {
			memset(lvl[seg], lvl_seg, sizeof(lvl[seg]));
			continue;
		}
		int intra = lvl_seg + (int8_t)rp->p.ref_deltas[0] * scale;
		lvl[seg][0][0] = (uint8_t)(intra < 0 ? 0 : intra > 63 ? 63 : intra);
		lvl[seg][0][1] = 0;
		for (int ref = 1; ref < 4; ref++) {
			for (int mode = 0; mode < 2; mode++) {
				int inter = lvl_seg + (int8_t)rp->p.ref_deltas[ref] * scale + (int8_t)rp->p.mode_deltas[mode] * scale;
				lvl[seg][ref][mode] = (uint8_t)(inter < 0 ? 0 : inter > 63 ? 63 : inter);
			}
		}
	}
	for (int i = 0; i < 16; i++) {
		uint32_t v = ((lvl[i >> 1][3][i & 1] & 0x3fu) << 24) | ((lvl[i >> 1][2][i & 1] & 0x3fu) << 16) |
		    ((lvl[i >> 1][1][i & 1] & 0x3fu) << 8) | (lvl[i >> 1][0][i & 1] & 0x3fu);
		WR(HEVC_DBLK_CFGA, level ? v : 0);
	}
}

/* ---- the transport ---- */

bool
hakios_vp9_active(void)
{
	return V.fw_loaded;
}

void
hakios_vp9_cancel(void)
{
	if (V.running) core_stop();
	pages_release();
	bzero(&V, sizeof(V));
}

static int
copy_rpm_out(uint64_t user)
{
	return copyout(V.param.data, (user_addr_t)user, sizeof(V.param.data));
}

static void dump_regs(void);

static void
run(uint32_t size)
{
	hakios_hevc_reset_core();
	prot_init();
	/* the stream once the parser is set up (as hakios.hevc.c): pointed at before it, the shifter takes each 64-bit
	 * word's first byte eight times over and never finds a frame's header */
	uint32_t start = pa(STREAM_OFF);
	WR(HEVC_STREAM_START_ADDR, start);
	WR(HEVC_STREAM_END_ADDR, start + STREAM_SIZE);
	WR(HEVC_STREAM_RD_PTR, start);
	WR(HEVC_STREAM_WR_PTR, (start + size + STREAM_PAD) & ~7u);
	SET(HEVC_STREAM_CONTROL, 7 << 4);
	SET(HEVC_STREAM_FIFO_CTL, 1u << 29);
	SET(HEVC_STREAM_CONTROL, 1);
	apply_pokes();
	WR(HEVC_ASSIST_SCRATCH_C, 0);
	CLEAR(HEVC_EFFICIENCY_MODE, 1u << 1);
	WR(HEVC_DEC_STATUS_REG, ST_ACTION_DONE);
	WR(VP9_CONTROL, 1);
	for (int i = 0; i < 8; i++) WR(VP9_QUANT_WR, V.seg_quant[i]);
	WR(HEVC_SHIFT_BYTE_COUNT, 0);
	WR(HEVC_DECODE_SIZE, size);
	/* the frames run so far (vvp9.c slice_idx): with 0 the firmware starts over, its probability contexts back at
	 * the defaults, which only key and error-resilient frames survive */
	WR(HEVC_DECODE_COUNT, V.slice_idx);
	WR(DOS_SW_RESET3, (1 << 12) | (1 << 11));
	WR(DOS_SW_RESET3, 0);
	SET(HEVC_WRRSP_LMEM, 7u << 25);
	WR(HEVC_MPSR, 1);
	V.running = true;
}

int
hakios_vp9_prepare(const struct twl_h264_prepare *request)
{
	if (V.header_pending) {
		/* the frame before was never answered: let it go */
		core_stop();
		V.header_pending = false;
		V.slice_idx++;
	}
	if (!request->input_size || request->input_size > STREAM_SIZE - STREAM_PAD - 0x1000 - FRAME_HEADER ||
	    request->rpm_size != sizeof(V.param.data)) return EINVAL;
	if (!V.fw_loaded) {
		if (!hakios_vdec_pool(&V.pool, &V.pool_pa, &V.pool_size) || V.pool_size < MV_OFF + (64u << 20)) return ENOMEM;
		int r = hakios_optee_load_video_fw(HAKIOS_VIDEO_DEC_VP9_MMU, HAKIOS_OPTEE_VDEC_HEVC, false);
		if (r) {
			printf("vp9: firmware load: %d\n", r);
			return EIO;
		}
		layout_workspace();
		pool_zero(0, STREAM_OFF);
		pool_zero(MMU_MAP_OFF, MMU_MAP_SIZE);
		V.fw_loaded = true;
	}
	/* the header the firmware looks for in front of each frame (vdec_vp9_if.c add_prefix_data: the length counts it) */
	uint8_t *in = (uint8_t *)(V.pool + STREAM_OFF);
	uint32_t total = request->input_size + FRAME_HEADER;
	uint8_t head[FRAME_HEADER] = {
		(uint8_t)(total >> 24), (uint8_t)(total >> 16), (uint8_t)(total >> 8), (uint8_t)total,
		(uint8_t)~(total >> 24), (uint8_t)~(total >> 16), (uint8_t)~(total >> 8), (uint8_t)~total,
		0, 0, 0, 1, 'A', 'M', 'L', 'V',
	};
	memcpy(in, head, FRAME_HEADER);
	int error = copyin((user_addr_t)request->input, in + FRAME_HEADER, request->input_size);
	if (error) return error;
	bzero(in + total, STREAM_PAD + 0x1000);
	FlushPoC_DcacheRegion((vm_offset_t)in, total + STREAM_PAD + 0x1000);
	V.input_size = total;

	run(total);
	uint32_t s = wait_status();
	if (s == ST_HEAD_DONE) {
		read_rpm();
		V.header_pending = true;
		return copy_rpm_out(request->rpm);
	}
	static unsigned reports;
	if (reports < 6 || (udebug & 4)) {
		reports++;
		printf("vp9: no header in %u bytes: status %#x, PC %#x, shift %u, stream rd %#x wr %#x level %#x, "
		    "parser int %#x shift status %#x\n", request->input_size, s, RD(HEVC_MPC_E), RD(HEVC_SHIFT_BYTE_COUNT),
		    RD(HEVC_STREAM_RD_PTR), RD(HEVC_STREAM_WR_PTR), RD(HEVC_STREAM_LEVEL), RD(HEVC_PARSER_INT_STATUS),
		    RD(HEVC_SHIFT_STATUS));
		if (udebug & 4) dump_regs();
	}
	core_stop();
	if (s == ST_NAL_DONE || s == ST_BUFEMPTY || s == ST_SEARCH_EMPTY || s == ST_EOS) return ENODATA;
	printf("vp9: frame of %u bytes ended in status %#x\n", request->input_size, s);
	return EIO;
}

/* bufmgr_process's start: what every header changes, decoded or not */
static void
header_seen(void)
{
	V.last_frame_type = V.frame_type;
	V.last_intra_only = V.intra_only;
}

int
hakios_vp9_frame(struct twl_vp9_frame *f)
{
	if (!V.header_pending) return EINVAL;
	const union param *rp = &V.param;
	V.header_pending = false;
	V.slice_idx++;
	if (f->flags & ~(TWL_VP9_SKIP | TWL_VP9_NOCOPY | TWL_VP9_SHOW | TWL_VP9_QUARTER)) {
		core_stop();
		return EINVAL;
	}
	if (f->flags & TWL_VP9_SKIP) {
		core_stop();
		if (!(f->flags & TWL_VP9_SHOW)) return ENODATA;
		/* show_existing_frame: the picture already in the slot */
		if (!rp->p.show_existing_frame || !V.layout || f->slot >= V.n_pics || !V.pic[f->slot].w) return EINVAL;
		header_seen();
		f->width = (uint32_t)V.pic[f->slot].w;
		f->height = (uint32_t)V.pic[f->slot].h;
		goto copy;
	}
	if (rp->p.show_existing_frame) {
		core_stop();
		return EINVAL;
	}

	bool key = rp->p.frame_type == KEY_FRAME;
	int intra_only = key ? V.intra_only : (rp->p.show_frame ? 0 : rp->p.intra_only);
	bool inter = !key && !intra_only;
	int w = rp->p.width, h = rp->p.height, depth = rp->p.bit_depth;
	/* the depth is in key and intra-only frames' headers only: the rest keep the stream's (vp9_bufmgr_process) */
	if (intra_only && !rp->p.profile && !depth) depth = 8;        /* (profile 0's intra-only frames: 8 bits) */
	if (key || intra_only) V.bit_depth = depth; else depth = V.bit_depth;
	if (inter) {
		for (int i = 0; i < REFS; i++) {
			if (f->ref_slot[i] >= V.n_pics || !V.pic[f->ref_slot[i]].w) {
				core_stop();
				return EINVAL;
			}
		}
		for (int i = 0; i < REFS; i++) {
			if ((rp->p.same_frame_size >> (REFS - i - 1)) & 1) {
				w = V.pic[f->ref_slot[i]].w;
				h = V.pic[f->ref_slot[i]].h;
				break;
			}
		}
	}
	if (w < 8 || h < 8 || (w > h ? w : h) > 4096 || (w > h ? h : w) > 2304 || (depth != 8 && depth != 10)) {
		printf("vp9: unsupported frame %dx%d, %d bits, profile %u\n", w, h, depth, rp->p.profile);
		core_stop();
		return ENOTSUP;
	}
	if (!V.layout && !layout_pictures(w, h, (f->flags & TWL_VP9_QUARTER) != 0)) {
		core_stop();
		return ENOMEM;
	}
	if (w > V.pic_w || h > V.pic_h) {
		core_stop();
		return ESTALE;
	}
	if (f->slot >= V.n_pics) {
		core_stop();
		return EINVAL;
	}
	if (!V.list_written) init_pic_list_hw();

	/* vp9_bufmgr_process */
	header_seen();
	V.frame_type = rp->p.frame_type;
	V.show_frame = rp->p.show_frame;
	if (!key) V.intra_only = intra_only;
	V.width = w;
	V.height = h;
	int use_prev = !rp->p.error_resilient_mode && w == V.last_width && h == V.last_height && !V.last_intra_only &&
	    V.last_show_frame && V.last_frame_type != KEY_FRAME;

	WR(HEVC_PARSER_PICTURE_SIZE, (uint32_t)h << 16 | (uint32_t)w);
	/* the frame's body pages: the slots no reference names give theirs back first (user space says which do; with
	 * nothing said, all are kept) */
	uint32_t *map = (uint32_t *)(V.pool + MMU_MAP_OFF), pages = (comp_body_size(w, h, depth == 10) + 0xfff) >> 12;
	/* kept: the references' (to predict from) and the pictures a player or the plane may still show (the plane
	 * reads the compressed picture) */
	if (f->referenced & (1u << 31)) {
		uint32_t showing = hakios_vdec_showing(V.n_pics), keep = f->referenced | showing | hakios_vdec_held(V.n_pics);
		for (uint32_t s = 0; s < V.n_pics; s++) {
			if (s != f->slot && !(keep & (1u << s))) pages_trim(s, 0);
		}
		if (P.top < pages) {
			/* short: the held pictures not referenced go back to their copies, never the plane's (it reads them) */
			for (uint32_t s = 0; s < V.n_pics; s++) {
				if (s != f->slot && !((f->referenced | showing) & (1u << s))) pages_trim(s, 0);
			}
		}
	}
	if (!pages_take(f->slot, pages)) {
		printf("vp9: no pages for a %dx%d frame (%u wanted, %u free)\n", w, h, pages, P.top);
		core_stop();
		return ENOMEM;
	}
	for (uint32_t i = 0; i < pages; i++) map[i] = (pa(P.base) >> 12) + P.list[f->slot * P.per_slot + i];
	FlushPoC_DcacheRegion(V.pool + MMU_MAP_OFF, pages * 4);
	V.pic[f->slot].w = 0;           /* (until it is done) */

	config_pic_size(w, h, depth);
	if (inter) {
		config_mc_buffer(f, w, h);
		config_mpred(use_prev);
	} else {
		clear_mpred();
	}
	config_mcrcc(inter);
	config_sao(f->slot);
	config_loop_filter(w);
	WR(HEVC_DEC_STATUS_REG, ST_DECODE_SLICE);

	uint32_t st = wait_status();
	for (int i = 0; i < 8; i++) V.seg_quant[i] = RD(VP9_QUANT_WR);
	uint32_t used = RD(HEVC_SAO_MMU_STATUS) >> 16;
	core_stop();
	if (st == ST_PIC_DONE && used > 0 && used <= pages) pages_trim(f->slot, used);
	if (st == ST_PIC_DONE && used > 0) {
		/* diagnosis: what the bodies really take, against the pool's room for them */
		static uint32_t n_used, max_used, min_free = ~0u;
		static uint64_t sum_used;
		n_used++;
		sum_used += used;
		if (used > max_used) max_used = used;
		if (P.top < min_free) min_free = P.top;
		if (n_used >= 300) {
			printf("vp9: bodies %dx%d: %u pages a frame on average, %u at most (%u reserved a frame); %u of %u free at "
			    "the least, %u slots\n", w, h, (uint32_t)(sum_used / n_used), max_used, pages, min_free, P.total, V.n_pics);
			n_used = max_used = 0;
			sum_used = 0;
			min_free = ~0u;
		}
	}
	else if (st != ST_PIC_DONE) pages_trim(f->slot, 0);
	if (st != ST_PIC_DONE) {
		printf("vp9: frame in slot %u (%dx%d, %s) ended in status %#x (LCU %#x)\n", f->slot, w, h,
		    key ? "key" : intra_only ? "intra" : "inter", st, RD(HEVC_PARSER_LCU_START));
		return EIO;
	}
	/* vp9_bufmgr_postproc */
	V.last_show_frame = V.show_frame;
	V.last_width = w;
	V.last_height = h;
	V.decoded++;
	V.pic[f->slot].w = w;
	V.pic[f->slot].h = h;
	V.pic[f->slot].depth = depth;
	f->width = (uint32_t)w;
	f->height = (uint32_t)h;
copy:;
	vm_offset_t frame = va(V.pic[f->slot].y);
	uint32_t bytes = V.stride * V.rows * 3 / 2;
	/* written behind the caches: for the CPU's copy, and so no stale line goes back over it */
	FlushPoC_DcacheRegion(frame, bytes);
	if (f->flags & TWL_VP9_NOCOPY) return 0;
	if (f->output_size < bytes) return EINVAL;
	return copyout((const void *)frame, (user_addr_t)f->output, bytes);
}

bool
hakios_vp9_shown(void)
{
	return V.show_frame != 0;
}

bool
hakios_vp9_stream(uint32_t *slots, uint32_t *width, uint32_t *height, uint32_t *stride, uint32_t *rows)
{
	if (!V.layout) return false;
	*slots = V.n_pics;
	*width = (uint32_t)V.dw_w;                      /* (the copies') */
	*height = (uint32_t)V.dw_h;
	*stride = V.stride;
	*rows = V.rows;
	return true;
}

bool
hakios_vp9_picture(uint32_t slot, struct hakios_vdec_picture *out)
{
	if (!V.layout || slot >= V.n_pics || !V.pic[slot].w) return false;
	out->luma = V.pic[slot].y;
	out->chroma = V.pic[slot].uv;
	out->stride = V.stride;
	out->rows = V.rows;
	out->width = (uint32_t)V.pic[slot].w / (uint32_t)V.dw;
	/* the compressed picture while its pages are kept (a picture whose pages had to go: its copy) */
	out->compressed = P.count[slot] ? V.pic[slot].cm : 0;
	out->coded_width = (uint32_t)V.pic[slot].w;
	out->coded_height = (uint32_t)V.pic[slot].h;
	out->depth = (uint32_t)V.pic[slot].depth;
	return true;
}

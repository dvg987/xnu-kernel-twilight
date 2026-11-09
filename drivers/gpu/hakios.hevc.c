/*
 * The Amlogic box's HEVC decoder (S7D's HEVC core, its "hevc_mmu_swap" firmware), behind /dev/twl-vdec-probe
 * (hakios.vdec.c) with vdec=7. Main and Main 10 up to 4096x2304 either way round, as Amlogic's vh265.c drives it:
 * compressed reference pictures (their body in 4 KiB pages the core's MMU finds through a map) and an 8-bit
 * linear NV12 copy of each picture ("double write"), the one players get. Above 1080p the copy is half the size
 * each way and the video plane shows the compressed picture; the bodies' pages come from a pool of their own,
 * each picture keeping only what the core used, and only while a reference or the plane may still need it.
 *
 * Per access unit: the core is reset, its work space and the stream are set up, the firmware parses up to the
 * first slice header and stops. User space (tvos/video/hevc) decides which slot the picture goes into and which
 * pictures it refers to; everything else, addresses above all, comes from here and from the header as the
 * firmware parsed it (the kernel's own copy, not one handed back by user space).
 *
 * Things the core needs that vh265.c does not say: the firmware's overlay address (scratch 8) is overwritten during
 * a run and must be put back before the next, or it fetches its code from garbage and hangs the bus; the parser
 * reads ahead about 12 KiB past the unit's end; the MMU's map address must be valid before the first run.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <kern/kalloc.h>
#include <kern/clock.h>
#include <mach/vm_param.h>
#include <sys/hakios.hevc.h>
#include <sys/hakios.optee.h>
#include <sys/sysctl.h>
#include "hakios.hevc_regs.h"

extern void IOSleep(unsigned int milliseconds);
extern void IODelay(unsigned int microseconds);
extern void FlushPoC_DcacheRegion(vm_offset_t va, size_t length);

#define RD(r)           hakios_dos_rd(r)
#define WR(r, v)        hakios_dos_wr(r, v)
#define SET(r, m)       WR(r, RD(r) | (m))
#define CLEAR(r, m)     WR(r, RD(r) & ~(uint32_t)(m))
#define ALIGN(x, a)     (((x) + (a) - 1) & ~((uint64_t)(a) - 1))

#define FW_HEVC_MMU_SWAP 0x32363553u        /* '265S' */

/* the pool while an HEVC stream has it */
#define WS_SIZE         0x00800000u
#define STREAM_OFF      0x00800000u
#define STREAM_SIZE     0x007f0000u
#define STREAM_PAD      0x00003000u
#define MMU_MAP_OFF     0x00ff0000u
#define MMU_MAP_SIZE    0x00010000u
#define FRAMES_OFF      0x01000000u
#define PAGE_4K         0x1000u
#define PAGES_RESERVE   5               /* largest bodies the page area is laid out for, at the least */

/* kern.hakios_hevc_dw_full=1: full-size copies above 1080p too (bit-exact tests; needs a larger vdec_pool_mb) */
static int dw_full;
SYSCTL_INT(_kern, OID_AUTO, hakios_hevc_dw_full, CTLFLAG_RW | CTLFLAG_LOCKED, &dw_full, 0, "");

#define RPM_WORDS       TWL_HEVC_RPM_WORDS
#define RPM_END         0x8000u
#define RPM_LT_BIT      14
#define RPM_USED_BIT    13
#define MAX_SLICES      600

enum { B_SLICE, P_SLICE, I_SLICE };

/* the firmware's parse (vh265.c union param_u) */
union param {
	uint16_t data[RPM_WORDS];
	struct {
		uint16_t CUR_RPS[0x10];
		uint16_t num_ref_idx_l0_active, num_ref_idx_l1_active, slice_type, slice_temporal_mvp_enable_flag;
		uint16_t dependent_slice_segment_flag, slice_segment_address, num_title_rows_minus1;
		uint16_t pic_width_in_luma_samples, pic_height_in_luma_samples;
		uint16_t log2_min_coding_block_size_minus3, log2_diff_max_min_coding_block_size;
		uint16_t log2_max_pic_order_cnt_lsb_minus4, POClsb, collocated_from_l0_flag, collocated_ref_idx;
		uint16_t log2_parallel_merge_level, five_minus_max_num_merge_cand, sps_num_reorder_pics_0;
		uint16_t modification_flag, tiles_enabled_flag, num_tile_columns_minus1, num_tile_rows_minus1;
		uint16_t tile_width[12], tile_height[8];
		uint16_t misc_flag0, pps_beta_offset_div2, pps_tc_offset_div2, slice_beta_offset_div2;
		uint16_t slice_tc_offset_div2, pps_cb_qp_offset, pps_cr_qp_offset, first_slice_segment_in_pic_flag;
		uint16_t m_temporalId, m_nalUnitType;
		uint16_t vui_num_units_in_tick_hi, vui_num_units_in_tick_lo, vui_time_scale_hi, vui_time_scale_lo;
		uint16_t bit_depth, profile_etc, sei_frame_field_info, video_signal_type;
		uint16_t modification_list[0x20];
		uint16_t conformance_window_flag, conf_win_left_offset, conf_win_right_offset, conf_win_top_offset;
		uint16_t conf_win_bottom_offset, chroma_format_idc, color_description, aspect_ratio_idc;
		uint16_t sar_width, sar_height, sps_max_dec_pic_buffering_minus1_0;
	} p;
};

#define PCM_LOOP_FILTER_DISABLED_FLAG_BIT 0
#define PCM_ENABLE_FLAG_BIT 1
#define LOOP_FILER_ACROSS_TILES_ENABLED_FLAG_BIT 2
#define PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT 3
#define DEBLOCKING_FILTER_OVERRIDE_ENABLED_FLAG_BIT 4
#define PPS_DEBLOCKING_FILTER_DISABLED_FLAG_BIT 5
#define DEBLOCKING_FILTER_OVERRIDE_FLAG_BIT 6
#define SLICE_DEBLOCKING_FILTER_DISABLED_FLAG_BIT 7
#define SLICE_SAO_LUMA_FLAG_BIT 8
#define SLICE_SAO_CHROMA_FLAG_BIT 9
#define SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT 10

#define HEVC_CONFIG_LITTLE_ENDIAN ((0xff0u << 8) | 0xfu)

struct buf { uint32_t start, size; };

static struct {
	bool fw_loaded, layout, header_pending, running;
	vm_offset_t pool;
	uint64_t pool_pa;
	uint32_t pool_size;
	uint32_t swap_addr, start_decoding_flag, rps_set_id;
	struct {
		struct buf ipp, sao_abv, sao_vb, short_term_rps, vps, sps, pps, sao_up, swap_buf, swap_buf2, scalelut,
		    dblk_para, dblk_data, dblk_data2, mmu_vbh, mmu_vbh_dw, cm_header_dw, mpred_above, rpm, lmem;
	} ws;
	union param param;
	/* the stream */
	int pic_w, pic_h, depth;        /* depth: 8 or 10 */
	int dw, dw_w, dw_h;             /* the copies' scale (1 or 2) and size */
	uint32_t n_pics, stride, rows, uv_offset, cm_body, cm_header, cm_bytes, frame_bytes, mv_bytes, decode_count;
	struct { uint32_t cm, y, uv, mv; bool done; } pic[TWL_HEVC_SLOTS];
	/* the picture */
	int cur, col, slice_idx, new_pic, cur_poc, col_poc;
	int lcu_size, lcu_size_log2, lcu_x_num, lcu_y_num, slice_addr, slice_type, ref_num[2];
	int32_t ref_poc[2][16];
	uint32_t long_term;
	int tile_enabled, num_tile_col, num_tile_row, new_tile, tile_x, tile_y;
	int tile_start_lcu_x, tile_start_lcu_y, tile_width_lcu, tile_height_lcu;
	struct { int width, height, start_cu_x, start_cu_y; } tile[20][10];
} H;

static uint32_t
pa(uint32_t offset)
{
	return (uint32_t)(H.pool_pa + offset);
}

static void
pool_zero(uint32_t offset, uint32_t size)
{
	bzero((void *)(H.pool + offset), size);
	FlushPoC_DcacheRegion(H.pool + offset, size);
}

/* ---- the compressed bodies' pages (as hakios.vp9.c: vh265.c's mmu box) ---- */

static struct {
	uint32_t base;                  /* pool offset */
	uint32_t total, top, *free;     /* a stack of free page numbers (pool page index) */
	uint32_t per_slot, *list;       /* each slot's pages: list[slot * per_slot ...] */
	uint32_t count[TWL_HEVC_SLOTS];
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

/* ---- the core ---- */

static void
layout_workspace(void)
{
	uint32_t at = 0;
	struct { struct buf *b; uint32_t size; } spec[] = {     /* amvh265_workbuff_spec[5], init_buff_spec's order */
		{ &H.ws.ipp, 0x4000 * 2 }, { &H.ws.sao_abv, 0 }, { &H.ws.sao_vb, 0 }, { &H.ws.short_term_rps, 0x800 },
		{ &H.ws.vps, 0x800 }, { &H.ws.sps, 0x800 }, { &H.ws.pps, 0x2000 }, { &H.ws.sao_up, 0 }, { &H.ws.swap_buf, 0x800 },
		{ &H.ws.swap_buf2, 0x800 }, { &H.ws.scalelut, 0x8000 }, { &H.ws.dblk_para, 0x40100 },
		{ &H.ws.dblk_data, 0x140800 }, { &H.ws.dblk_data2, 0x100800 }, { &H.ws.mmu_vbh, 0xa000 },
		{ &H.ws.mmu_vbh_dw, 0xa000 << 1 }, { &H.ws.cm_header_dw, 0 }, { &H.ws.mpred_above, 0x8000 },
		{ &H.ws.rpm, 0x80 * 2 }, { &H.ws.lmem, 0x500 * 2 },
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

/* vh265_prot_init: hevc_config_work_space_hw, hevc_init_decoder_hw, config_decode_mode */
static void
prot_init(void)
{
	WR(HEVCD_IPP_LINEBUFF_BASE, H.ws.ipp.start);
	WR(HEVC_RPM_BUFFER, H.ws.rpm.start);
	WR(HEVC_SHORT_TERM_RPS, H.ws.short_term_rps.start);
	WR(HEVC_VPS_BUFFER, H.ws.vps.start);
	WR(HEVC_SPS_BUFFER, H.ws.sps.start);
	WR(HEVC_PPS_BUFFER, H.ws.pps.start);
	WR(HEVC_SAO_UP, H.ws.sao_up.start);
	WR(HEVC_ASSIST_MMU_MAP_ADDR, pa(MMU_MAP_OFF));
	WR(HEVC_SCALELUT, H.ws.scalelut.start);
	WR(HEVC_DBLK_CFG3, 0x8020);
	WR(HEVC_DBLK_CFG4, H.ws.dblk_para.start);
	WR(HEVC_DBLK_CFG5, H.ws.dblk_data.start);
	WR(HEVC_DBLK_CFGE, H.ws.dblk_data2.start);
	WR(LMEM_DUMP_ADR, H.ws.lmem.start);

	WR(DOS_SW_RESET3, 1u << 14);
	CLEAR(HEVC_CABAC_CONTROL, 1);
	CLEAR(HEVC_PARSER_CORE_CONTROL, 1);
	WR(HEVC_SHIFT_STARTCODE, 0x00000100);
	WR(HEVC_SHIFT_EMULATECODE, 0x00000300);
	WR(HEVC_PARSER_INT_CONTROL, (RD(HEVC_PARSER_INT_CONTROL) & 0x03ffffffu) | (3u << 29) | (2 << 26) | (1 << 24) |
	    (1 << 22) | (1 << 7) | (1 << 4) | (1 << 0));
	SET(HEVC_SHIFT_STATUS, (1 << 1) | (1 << 0));
	WR(HEVC_SHIFT_CONTROL, (3 << 6) | (2 << 4) | (2 << 1) | (1 << 0));
	WR(HEVC_CABAC_CONTROL, 1);
	WR(HEVC_PARSER_IF_CONTROL, (1 << 5) | (1 << 2) | (1 << 0));
	WR(HEVC_PARSER_CORE_CONTROL, 1);
	WR(HEVC_DEC_STATUS_REG, 0);
	WR(HEVC_IQIT_SCALELUT_WR_ADDR, 0);
	for (int i = 0; i < 1024; i++) WR(HEVC_IQIT_SCALELUT_DATA, 0);
	WR(HEVC_DECODE_SIZE, 0);
	WR(HEVC_DECODE_COUNT, 0);
	WR(HEVC_PARSER_CMD_WRITE, 1u << 16);
	for (int i = 0; i < 37; i++) WR(HEVC_PARSER_CMD_WRITE, parser_cmd[i]);
	WR(HEVC_PARSER_CMD_SKIP_0, 0x0000090b);
	WR(HEVC_PARSER_CMD_SKIP_1, 0x1b14140f);
	WR(HEVC_PARSER_CMD_SKIP_2, 0x001b1910);
	WR(HEVCD_IPP_TOP_CNTL, 1 << 0);
	WR(HEVCD_IPP_TOP_CNTL, 1 << 1);

	WR(HEVC_ASSIST_MBOX0_CLR_REG, 1);
	WR(HEVC_ASSIST_MBOX0_MASK, 1);
	WR(HEVC_PSCALE_CTRL, 0);
	WR(DEBUG_REG1, 0);
	WR(NAL_SEARCH_CTL, 0x8 | (1u << 17));   /* VPS/SPS/PPS/I checked by the firmware; SEI parsed */
	WR(DECODE_STOP_POS, 0);
	WR(HEVC_DECODE_MODE, 0x1 | (H.start_decoding_flag << 16) | (0x80u << 24));    /* frame based */
	WR(HEVC_DECODE_MODE2, H.rps_set_id);
	CLEAR(NAL_SEARCH_CTL, (1u << 22) | (1u << 21));
}

/* init_pic_list_hw + init_decode_head_hw (MMU): lost with the core's reset, so every run */
static void
init_pic_list_hw(void)
{
	WR(HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, (1 << 1) | (1 << 2));
	for (uint32_t i = 0; i < H.n_pics; i++) WR(HEVCD_MPP_ANC2AXI_TBL_DATA, H.pic[i].cm >> 5);     /* headers */
	WR(HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);
	WR(HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	for (int i = 0; i < 32; i++) WR(HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);
	WR(HEVCD_MPP_DECOMP_CTL1, 1u << 4);
	WR(HEVCD_MPP_DECOMP_CTL2, 0);
	WR(HEVC_CM_BODY_LENGTH, H.cm_body);
	WR(HEVC_CM_HEADER_OFFSET, H.cm_body);
	WR(HEVC_CM_HEADER_LENGTH, 32 * ((H.pic_w + 127) >> 7) * ((H.pic_h + 63) >> 6));
	WR(HEVC_SAO_MMU_VH0_ADDR, H.ws.mmu_vbh.start);
	WR(HEVC_SAO_MMU_VH1_ADDR, H.ws.mmu_vbh.start + (H.ws.mmu_vbh.size >> 1));
	SET(HEVC_SAO_CTRL9, 1);
	WR(HEVC_SAO_CTRL5, (RD(HEVC_SAO_CTRL5) & ~(1u << 15)) | (1u << 10));
}

static void
core_stop(void)
{
	WR(HEVC_MPSR, 0);
	WR(HEVC_CPSR, 0);
	for (int i = 0; i < 2000 && (RD(HEVC_IMEM_DMA_CTRL) & 0x8000); i++) IODelay(10);
	for (int i = 0; i < 2000 && (RD(HEVC_LMEM_DMA_CTRL) & 0x8000); i++) IODelay(10);
	for (int i = 0; i < 2000 && (RD(HEVC_WRRSP_LMEM) & 0xfff); i++) IODelay(10);
	H.running = false;
}

static void
read_decode_info(void)
{
	uint32_t info = RD(HEVC_RPM_BUFFER);     /* HEVC_DECODE_INFO: the firmware writes it over the RPM address */
	H.start_decoding_flag |= info & 0xff;
	H.rps_set_id = (info >> 8) & 0xff;
}

static void
read_rpm(void)
{
	vm_offset_t rpm = H.pool + (H.ws.rpm.start - (uint32_t)H.pool_pa);
	FlushPoC_DcacheRegion(rpm, RPM_WORDS * 2);
	const uint16_t *raw = (const uint16_t *)rpm;
	for (unsigned i = 0; i < RPM_WORDS; i += 4)
		for (unsigned k = 0; k < 4; k++) H.param.data[i + k] = raw[i + 3 - k];
}

/* the stream's transfer characteristics from the VUI (vh265.c: video_signal_type [8] colour description present, then
 * color_description [15:8]), 0 when the stream does not say: 16 is PQ (HDR10 and Dolby Vision 8.1's base layer) */
uint32_t
hakios_hevc_transfer(void)
{
	return (H.param.p.video_signal_type & 0x100) ? H.param.p.color_description >> 8 : 0;
}

/* the status the firmware stops in: a slice header (8), the picture done (0xa, 0x50 or the unit used up), an error */
static uint32_t
wait_status(void)
{
	uint64_t start = mach_absolute_time(), limit;
	nanoseconds_to_absolutetime(2000ull * NSEC_PER_MSEC, &limit);
	for (unsigned spins = 0;; spins++) {
		uint32_t seq = hakios_vdec_irq_seq();
		uint32_t s = RD(HEVC_DEC_STATUS_REG);
		if (s == 0xc) {                 /* an SEI: nothing here wants it */
			WR(HEVC_DEC_STATUS_REG, 0xd);
			WR(HEVC_MCPU_INTR_REQ, 0x04);
			continue;
		}
		if (s == 0x8 || s == 0xa || s == 0x50 || s == 0x20 || s == 0x24 || s == 0x22 || s == 0xe || s == 0xb ||
		    s == 0x21 || s == 0x23 || s == 0x25) {
			return s;
		}
		if (mach_absolute_time() - start > limit) {
			printf("hevc: stuck in status %#x: PC %#x, LCU %#x, shift %u\n", s, RD(HEVC_MPC_E),
			    RD(HEVC_PARSER_LCU_START), RD(HEVC_SHIFT_BYTE_COUNT));
			return 0;
		}
		if (hakios_vdec_irq_nap(seq, 250)) continue;
		if (spins < 400) IODelay(25); else IOSleep(1);     /* a picture takes 2 to 8 ms */
	}
}

/* ---- the stream's pictures ---- */

static uint32_t
uv_pages(int w, int h)
{
	unsigned lw = (w + 63) / 64, lh = (h + 63) / 64;
	return ((lw * lh * 64 * 64 / 2) + 0xfff) >> 12;
}

static bool
layout_pictures(int w, int h, int lcu_log2)
{
	H.dw = !dw_full && (w > 1920 || h > 1088) ? 2 : 1;
	H.dw_w = w / H.dw;
	H.dw_h = h / H.dw;
	uint32_t uvp = uv_pages(H.dw_w, H.dw_h);
	pages_release();
	H.pic_w = w;
	H.pic_h = h;
	H.depth = (H.param.p.bit_depth & 0xf) + 8;
	H.stride = (uint32_t)ALIGN(H.dw_w, 64);
	H.rows = (uint32_t)ALIGN(H.dw_h, 64);
	H.uv_offset = uvp << 13;
	H.cm_body = 4096 * ((w + 63) >> 6) * ((h + 31) >> 5);
	H.cm_header = ALIGN(w, 64) * ALIGN(h, 64) > 1920 * 1088 ? 0x48000 : 0x10000;     /* hevc_get_header_size */
	H.cm_bytes = H.cm_header;
	H.frame_bytes = H.cm_bytes + (uint32_t)ALIGN(H.uv_offset + (uvp << 12), 0x10000);
	unsigned unit = lcu_log2 == 6 ? 0x200 : lcu_log2 == 5 ? 0x80 : 0x20;
	H.mv_bytes = (uint32_t)ALIGN(((ALIGN(w, 64) >> lcu_log2) + 1) * ((ALIGN(h, 64) >> lcu_log2) + 1) * unit + 0x40000, 0x10000);
	/* a slot: the compressed picture's header, the copy (NV12) and its motion vectors; the compressed bodies in pages
	 * of their own, enough for a few pictures at their largest; the rest of the pool more slots */
	uint32_t worst = H.cm_body >> 12, reserve = PAGES_RESERVE * worst * PAGE_4K;
	uint32_t slot_bytes = H.frame_bytes + H.mv_bytes;
	uint32_t n = H.pool_size > FRAMES_OFF + reserve ? (H.pool_size - FRAMES_OFF - reserve) / slot_bytes : 0;
	if (n > TWL_HEVC_SLOTS) n = TWL_HEVC_SLOTS;
	if (n < 8 || worst * 4 > MMU_MAP_SIZE) {
		printf("hevc: %dx%d needs %u bytes a picture and %u for the bodies; the pool (vdec_pool_mb) holds %u\n", w, h,
		    slot_bytes, reserve, n);
		return false;
	}
	uint32_t pages_off = (uint32_t)ALIGN(FRAMES_OFF + n * slot_bytes, PAGE_4K);
	uint32_t n_pages = (H.pool_size - pages_off) / PAGE_4K;
	if (!pages_setup(pages_off, n_pages, n, worst)) {
		printf("hevc: no memory for the page lists\n");
		return false;
	}
	H.n_pics = n;
	uint32_t at = FRAMES_OFF;
	for (uint32_t i = 0; i < n; i++) {
		H.pic[i].cm = pa(at);
		H.pic[i].y = pa(at + H.cm_bytes);
		H.pic[i].uv = H.pic[i].y + H.uv_offset;
		at += H.frame_bytes;
		H.pic[i].mv = pa(at);
		at += H.mv_bytes;
		H.pic[i].done = false;
	}
	pool_zero(FRAMES_OFF, at - FRAMES_OFF);
	H.layout = true;
	printf("hevc: %dx%d, %u pictures of %#x + MV %#x (copies %dx%d, stride %u, rows %u), %u body pages (%u a picture "
	    "at most)\n", w, h, n, H.frame_bytes, H.mv_bytes, H.dw_w, H.dw_h, H.stride, H.rows, n_pages, worst);
	return true;
}

static void
update_tile_info(int pic_width_cu, int pic_height_cu)
{
	const union param *rp = &H.param;
	H.tile_enabled = rp->p.tiles_enabled_flag & 1;
	if (H.tile_enabled) {
		H.num_tile_col = rp->p.num_tile_columns_minus1 + 1;
		H.num_tile_row = rp->p.num_tile_rows_minus1 + 1;
		if (H.num_tile_row > 20 || H.num_tile_row <= 0) H.num_tile_row = 1;
		if (H.num_tile_col > 10 || H.num_tile_col <= 0) H.num_tile_col = 1;
		int start_y = 0;
		for (int i = 0; i < H.num_tile_row; i++) {
			int start_x = 0;
			for (int j = 0; j < H.num_tile_col; j++) {
				if (rp->p.tiles_enabled_flag & 2) {
					H.tile[i][j].width = (j + 1) * pic_width_cu / H.num_tile_col - j * pic_width_cu / H.num_tile_col;
					H.tile[i][j].height = (i + 1) * pic_height_cu / H.num_tile_row - i * pic_height_cu / H.num_tile_row;
				} else {
					H.tile[i][j].width = j == H.num_tile_col - 1 ? pic_width_cu - start_x : rp->p.tile_width[j < 12 ? j : 11];
					H.tile[i][j].height = i == H.num_tile_row - 1 ? pic_height_cu - start_y :
					    i < 8 ? rp->p.tile_height[i] : 0;
				}
				H.tile[i][j].start_cu_x = start_x;
				H.tile[i][j].start_cu_y = start_y;
				start_x += H.tile[i][j].width;
			}
			start_y += H.tile[i][0].height;
		}
	} else {
		H.num_tile_col = H.num_tile_row = 1;
		H.tile[0][0].width = pic_width_cu;
		H.tile[0][0].height = pic_height_cu;
		H.tile[0][0].start_cu_x = H.tile[0][0].start_cu_y = 0;
	}
}

static int
tile_index(int cu_adr, int pic_width_lcu)
{
	int cu_x = cu_adr % pic_width_lcu, cu_y = cu_adr / pic_width_lcu, tx = 0, ty = 0;
	if (H.tile_enabled) {
		for (int i = 0; i < H.num_tile_col && cu_x >= H.tile[0][i].start_cu_x; i++) tx = i;
		for (int i = 0; i < H.num_tile_row && cu_y >= H.tile[i][0].start_cu_y; i++) ty = i;
	}
	return tx | ty << 8;
}

/* ---- one slice's set-up (vh265.c config_*) ---- */

static void
config_mc_buffer(const struct twl_hevc_slice *s)
{
	for (int list = 0; list < 2; list++) {
		if ((list == 0 && H.slice_type == 2) || (list == 1 && H.slice_type != 0)) continue;
		WR(HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, ((list ? 16u : 0u) << 8) | 1);
		for (int i = 0; i < H.ref_num[list]; i++) {
			uint32_t index = s->ref_slot[list][i];
			WR(HEVCD_MPP_ANC_CANVAS_DATA_ADDR, (index << 16) | (index << 8) | index);
		}
	}
}

static void
config_mcrcc(void)
{
	uint32_t a, b;
	WR(HEVCD_MCRCC_CTL1, 0x2);
	if (H.slice_type == 2) {
		WR(HEVCD_MCRCC_CTL1, 0x0);
		return;
	}
	if (H.slice_type == 0) {
		WR(HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 0);
		a = RD(HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
		a |= a << 16;
		WR(HEVCD_MCRCC_CTL2, a);
		WR(HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, (16 << 8) | (1 << 1));
		b = RD(HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
		b |= b << 16;
		if (a == b && H.ref_num[1] > 1) {
			b = RD(HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
			b |= b << 16;
		}
		WR(HEVCD_MCRCC_CTL3, b);
	} else {
		WR(HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1 << 1);
		a = RD(HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
		a |= a << 16;
		WR(HEVCD_MCRCC_CTL2, a);
		if (H.ref_num[0] != 1) {
			a = RD(HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
			a |= a << 16;
		}
		WR(HEVCD_MCRCC_CTL3, a);
	}
	WR(HEVCD_MCRCC_CTL1, 0xff0);
}

static void
config_mpred(const struct twl_hevc_slice *s, int ldc, int col_from_l0, int tmvp, int is_next, int plevel, int merge)
{
	const bool col = H.col >= 0;
	uint32_t cur_mv = H.pic[H.cur].mv, rd_start = col ? H.pic[H.col].mv : cur_mv;
	uint32_t unit = H.lcu_size_log2 == 6 ? 0x200 : H.lcu_size_log2 == 5 ? 0x80 : 0x20;
	uint32_t rd_ptr = rd_start + H.slice_addr * unit, rd_end = rd_start + H.mv_bytes, wr_ptr = cur_mv + H.slice_addr * unit;
	WR(HEVC_MPRED_MV_WR_START_ADDR, cur_mv);
	WR(HEVC_MPRED_MV_RD_START_ADDR, rd_start);
	uint32_t d = (H.lcu_x_num - H.tile_width_lcu) * unit;
	WR(HEVC_MPRED_MV_WR_ROW_JUMP, d);
	WR(HEVC_MPRED_MV_RD_ROW_JUMP, d);
	d = (H.slice_type & 3) | (H.new_pic & 1) << 2 | (H.new_tile & 1) << 3 | (is_next & 1) << 4 | (tmvp & 1) << 5 |
	    (ldc & 1) << 6 | (col_from_l0 & 1) << 7 | 1 << 9 | 1 << 10 | (H.slice_type != 2) << 11 | (!col) << 12 |
	    1 << 13 | (H.lcu_size_log2 & 0xf) << 16 | 3 << 20 | (plevel & 7) << 24;
	WR(HEVC_MPRED_CTRL0, d & ~(1u << 28));
	WR(HEVC_MPRED_CTRL1, (uint32_t)merge | 2 << 4 | 3 << 8 | 5 << 12 | 36 << 16);
	WR(HEVC_MPRED_PIC_SIZE, H.pic_w | H.pic_h << 16);
	WR(HEVC_MPRED_PIC_SIZE_LCU, (H.lcu_x_num - 1) | (H.lcu_y_num - 1) << 16);
	WR(HEVC_MPRED_TILE_START, H.tile_start_lcu_x | H.tile_start_lcu_y << 16);
	WR(HEVC_MPRED_TILE_SIZE_LCU, H.tile_width_lcu | H.tile_height_lcu << 16);
	WR(HEVC_MPRED_REF_NUM, H.ref_num[0] | H.ref_num[1] << 8);
	uint32_t lt = 0;
	for (int i = 0; i < H.ref_num[0]; i++) if (s->long_term & (1u << i)) lt |= 1u << i;
	for (int i = 0; i < H.ref_num[1]; i++) if (s->long_term & (1u << (16 + i))) lt |= 1u << (16 + i);
	WR(HEVC_MPRED_LT_REF, lt);
	WR(HEVC_MPRED_REF_EN_L0, (1u << H.ref_num[0]) - 1);
	WR(HEVC_MPRED_REF_EN_L1, (1u << H.ref_num[1]) - 1);
	WR(HEVC_MPRED_CUR_POC, (uint32_t)H.cur_poc);
	if (col) WR(HEVC_MPRED_COL_POC, (uint32_t)H.col_poc);
	for (int i = 0; i < 16; i++) {          /* in pairs, L0 then L1 */
		WR(HEVC_MPRED_L0_REF00_POC + i, (uint32_t)(i < H.ref_num[0] ? s->ref_poc[0][i] : 0));
		WR(HEVC_MPRED_L1_REF00_POC + i, (uint32_t)(i < H.ref_num[1] ? s->ref_poc[1][i] : 0));
	}
	if (H.new_pic) {
		WR(HEVC_MPRED_ABV_START_ADDR, H.ws.mpred_above.start);
		WR(HEVC_MPRED_MV_WPTR, wr_ptr);
		WR(HEVC_MPRED_MV_RPTR, rd_start);
	} else if (!is_next) {
		WR(HEVC_MPRED_MV_RPTR, rd_ptr);
	}
	WR(HEVC_MPRED_MV_RD_END_ADDR, rd_end);
}

static void
config_sao(void)
{
	const union param *rp = &H.param;
	int misc = rp->p.misc_flag0;
	uint32_t uvp = uv_pages(H.dw_w, H.dw_h), d, d2;
	WR(HEVC_SAO_CTRL0, (RD(HEVC_SAO_CTRL0) & ~0xfu) | (uint32_t)H.lcu_size_log2);
	WR(HEVC_SAO_PIC_SIZE, H.pic_w | H.pic_h << 16);
	WR(HEVC_SAO_PIC_SIZE_LCU, (H.lcu_x_num - 1) | (H.lcu_y_num - 1) << 16);
	if (H.new_pic) WR(HEVC_SAO_Y_START_ADDR, 0xffffffff);
	WR(HEVC_SAO_CTRL26, 0);
	/* the copy full size, or half each way ([23:16] dw_v1 dw_v0 dw_h1 dw_h0: 0x33, double write mode 4); 10-bit bodies */
	WR(HEVC_SAO_CTRL5, (RD(HEVC_SAO_CTRL5) & ~((0xffu << 16) | (1u << 9))) | (H.dw == 2 ? 0x33u << 16 : 0));
	WR(HEVC_SAO_Y_START_ADDR, H.pic[H.cur].y);
	WR(HEVC_CM_BODY_START_ADDR, 0);
	WR(HEVC_CM_HEADER_START_ADDR, H.pic[H.cur].cm);
	WR(HEVC_SAO_Y_LENGTH, uvp << 13);
	WR(HEVC_SAO_C_START_ADDR, H.pic[H.cur].uv);
	WR(HEVC_SAO_C_LENGTH, uvp << 12);
	WR(HEVC_SAO_Y_WPTR, H.pic[H.cur].y);
	WR(HEVC_SAO_C_WPTR, H.pic[H.cur].uv);
	if (H.new_pic) {
		d = (0xff << 8) | (1 << 8) | (1 << 9);  /* compressed (first) and linear (double) writes */
		if (H.pic_w >= 1280) d |= 1 << 4;
		WR(HEVC_DBLK_CFGB, d);
		WR(HEVC_DBLK_CFG2, H.pic_w | H.pic_h << 16);
		d = ((misc >> PCM_ENABLE_FLAG_BIT) & 1) ? ((misc >> PCM_LOOP_FILTER_DISABLED_FLAG_BIT) & 1) << 3 : 0;
		d |= ((rp->p.pps_cb_qp_offset & 0x1f) << 4) | ((rp->p.pps_cr_qp_offset & 0x1f) << 9);
		d |= H.lcu_size == 64 ? 0 : H.lcu_size == 32 ? 1 : 2;
		d |= H.pic_w <= 64 ? 1 << 20 : 0;
		WR(HEVC_DBLK_CFG1, d);
		WR(HEVC_DBLK_STS1 + 4, 1u << 28);
	}
	d = RD(HEVC_SAO_CTRL1) & ~(0x3000u | 0xff0u | 0x3u | (3u << 14));
	d |= ((HEVC_CONFIG_LITTLE_ENDIAN >> 8) & 0xfff) | (2u << 14);         /* linear, 64-byte lines */
	WR(HEVC_SAO_CTRL1, d);
	d = RD(HEVCD_IPP_AXIIF_CONFIG) & ~(0x30u | 0xfu | (3u << 8));
	WR(HEVCD_IPP_AXIIF_CONFIG, d | (HEVC_CONFIG_LITTLE_ENDIAN & 0xf) | (2u << 8));

	d = 0;
	d2 = RD(HEVC_SAO_CTRL0) & ~0x300u;
	if (H.tile_enabled) {
		d |= ((misc >> LOOP_FILER_ACROSS_TILES_ENABLED_FLAG_BIT) & 1) << 0;
		d2 |= ((misc >> LOOP_FILER_ACROSS_TILES_ENABLED_FLAG_BIT) & 1) << 8;
	}
	int slice_dbk_disabled = (misc >> SLICE_DEBLOCKING_FILTER_DISABLED_FLAG_BIT) & 1;
	if ((misc & (1 << DEBLOCKING_FILTER_OVERRIDE_ENABLED_FLAG_BIT)) && (misc & (1 << DEBLOCKING_FILTER_OVERRIDE_FLAG_BIT))) {
		d |= slice_dbk_disabled << 2;
		if (!slice_dbk_disabled) d |= ((rp->p.slice_beta_offset_div2 & 0xf) << 3) | ((rp->p.slice_tc_offset_div2 & 0xf) << 7);
	} else {
		d |= ((misc >> PPS_DEBLOCKING_FILTER_DISABLED_FLAG_BIT) & 1) << 2;
		if (!((misc >> PPS_DEBLOCKING_FILTER_DISABLED_FLAG_BIT) & 1))
			d |= ((rp->p.pps_beta_offset_div2 & 0xf) << 3) | ((rp->p.pps_tc_offset_div2 & 0xf) << 7);
	}
	if ((misc & (1 << PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)) &&
	    ((misc & (1 << SLICE_SAO_LUMA_FLAG_BIT)) || (misc & (1 << SLICE_SAO_CHROMA_FLAG_BIT)) || !slice_dbk_disabled)) {
		d |= ((misc >> SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT) & 1) << 1;
		d2 |= ((misc >> SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT) & 1) << 9;
	} else {
		d |= ((misc >> PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT) & 1) << 1;
		d2 |= ((misc >> PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT) & 1) << 9;
	}
	WR(HEVC_DBLK_CFG9, d);
	WR(HEVC_SAO_CTRL0, d2);
}

/* ---- the transport ---- */

bool
hakios_hevc_active(void)
{
	return H.fw_loaded;
}

/* the stream is over: the next one loads the firmware again (its overlay buffer may move) and lays out anew */
void
hakios_hevc_cancel(void)
{
	if (H.running) core_stop();
	pages_release();
	bzero(&H, sizeof(H));
}

static int check_stream(void);

static int
copy_rpm_out(uint64_t user)
{
	return copyout(H.param.data, (user_addr_t)user, sizeof(H.param.data));
}

int
hakios_hevc_prepare(const struct twl_h264_prepare *request)
{
	if (H.header_pending) return EBUSY;
	if (!request->input_size || request->input_size > STREAM_SIZE - STREAM_PAD - 0x1000 ||
	    request->rpm_size != sizeof(H.param.data)) return EINVAL;
	if (!H.fw_loaded) {
		if (!hakios_vdec_pool(&H.pool, &H.pool_pa, &H.pool_size) || H.pool_size < FRAMES_OFF + (64u << 20)) return ENOMEM;
		int r = hakios_optee_load_video_fw(FW_HEVC_MMU_SWAP, HAKIOS_OPTEE_VDEC_HEVC, true);
		if (r) {
			printf("hevc: firmware load: %d\n", r);
			return EIO;
		}
		H.swap_addr = RD(HEVC_STREAM_SWAP_BUFFER2);
		layout_workspace();
		pool_zero(0, STREAM_OFF);
		pool_zero(MMU_MAP_OFF, MMU_MAP_SIZE);
		H.fw_loaded = true;
	}
	int error = copyin((user_addr_t)request->input, (void *)(H.pool + STREAM_OFF), request->input_size);
	if (error) return error;
	bzero((void *)(H.pool + STREAM_OFF + request->input_size), STREAM_PAD + 0x1000);
	FlushPoC_DcacheRegion(H.pool + STREAM_OFF, request->input_size + STREAM_PAD + 0x1000);

	hakios_hevc_reset_core();
	WR(HEVC_STREAM_SWAP_BUFFER2, H.swap_addr);
	prot_init();
	uint32_t start = pa(STREAM_OFF);
	WR(HEVC_STREAM_START_ADDR, start);
	WR(HEVC_STREAM_END_ADDR, start + STREAM_SIZE);
	WR(HEVC_STREAM_RD_PTR, start);
	WR(HEVC_STREAM_WR_PTR, (start + request->input_size + STREAM_PAD) & ~7u);
	SET(HEVC_STREAM_CONTROL, 7 << 4);
	SET(HEVC_STREAM_FIFO_CTL, 1u << 29);
	SET(HEVC_STREAM_CONTROL, 1);
	WR(HEVC_DEC_STATUS_REG, 0xff);
	WR(HEVC_SHIFT_BYTE_COUNT, 0);
	WR(HEVC_DECODE_SIZE, request->input_size);
	WR(HEVC_DECODE_COUNT, H.decode_count);
	if (H.layout) init_pic_list_hw();
	WR(DOS_SW_RESET3, (1 << 12) | (1 << 11));
	WR(DOS_SW_RESET3, 0);
	SET(HEVC_WRRSP_LMEM, 7u << 25);
	WR(HEVC_MPSR, 1);
	H.running = true;
	uint32_t s = wait_status();
	read_decode_info();
	if (s == 0x8) {
		read_rpm();
		int error = check_stream();
		if (error) {
			core_stop();
			return error;
		}
		H.header_pending = true;
		return copy_rpm_out(request->rpm);
	}
	core_stop();
	if (s == 0x20 || s == 0x24 || s == 0x22 || s == 0xe) return ENODATA;
	printf("hevc: access unit of %u bytes ended in status %#x\n", request->input_size, s);
	return EIO;
}

/* a slice header the hardware takes; the stream's pictures laid out with its first (a new size: ESTALE, the
 * caller starts the stream again) */
static int
check_stream(void)
{
	const union param *rp = &H.param;
	int w = rp->p.pic_width_in_luma_samples, h = rp->p.pic_height_in_luma_samples;
	int lcu_log2 = rp->p.log2_min_coding_block_size_minus3 + 3 + rp->p.log2_diff_max_min_coding_block_size;
	if (!w || !h || (w > h ? w : h) > 4096 || (w > h ? h : w) > 2304 || lcu_log2 < 4 || lcu_log2 > 6 ||
	    ((rp->p.bit_depth & 0xf) > 2) || rp->p.chroma_format_idc != 1) {
		printf("hevc: unsupported stream %dx%d, CTB %d, depth %#x, chroma %u\n", w, h, 1 << lcu_log2, rp->p.bit_depth,
		    rp->p.chroma_format_idc);
		return ENOTSUP;
	}
	if (!H.layout && !layout_pictures(w, h, lcu_log2)) return ENOMEM;
	return w != H.pic_w || h != H.pic_h ? ESTALE : 0;
}

int
hakios_hevc_slice(struct twl_hevc_slice *s)
{
	if (!H.header_pending) return EINVAL;
	const union param *rp = &H.param;
	int w = rp->p.pic_width_in_luma_samples, h = rp->p.pic_height_in_luma_samples;
	int lcu_log2 = rp->p.log2_min_coding_block_size_minus3 + 3 + rp->p.log2_diff_max_min_coding_block_size;
	int error = check_stream();
	if (error) return error;
	bool new_pic = rp->p.slice_segment_address == 0;
	if (s->slot >= H.n_pics || (!new_pic && (int)s->slot != H.cur) || s->ref_num[0] > 16 || s->ref_num[1] > 16 ||
	    s->col_slot >= (int32_t)H.n_pics || s->col_slot < -1 || (s->flags & ~(TWL_HEVC_SKIP | TWL_HEVC_NOCOPY)))
		return EINVAL;
	for (int l = 0; l < 2; l++)
		for (uint32_t i = 0; i < s->ref_num[l]; i++)
			if (s->ref_slot[l][i] >= H.n_pics) return EINVAL;
	if (s->flags & TWL_HEVC_SKIP) {
		WR(HEVC_WAIT_FLAG, RD(HEVC_WAIT_FLAG) & ~0x2u);
		WR(HEVC_DEC_STATUS_REG, 0xff);
		WR(HEVC_MCPU_INTR_REQ, 0x04);
	} else {
		H.lcu_size_log2 = lcu_log2;
		H.lcu_size = 1 << lcu_log2;
		H.lcu_x_num = (w + H.lcu_size - 1) >> lcu_log2;
		H.lcu_y_num = (h + H.lcu_size - 1) >> lcu_log2;
		H.slice_type = rp->p.slice_type == I_SLICE ? 2 : rp->p.slice_type == P_SLICE ? 1 : 0;
		H.ref_num[0] = H.slice_type == 2 ? 0 : (int)s->ref_num[0];
		H.ref_num[1] = H.slice_type == 0 ? (int)s->ref_num[1] : 0;
		if (rp->p.first_slice_segment_in_pic_flag) {
			H.slice_addr = 0;
		} else if (!rp->p.dependent_slice_segment_flag) {
			H.slice_addr = rp->p.slice_segment_address;
		}
		H.new_pic = new_pic;
		int poc_changed = new_pic || s->poc != H.cur_poc;
		H.cur_poc = s->poc;
		WR(HEVC_WAIT_FLAG, RD(HEVC_WAIT_FLAG) | 0x2);
		if (new_pic) {
			H.cur = (int)s->slot;
			H.slice_idx = 0;
			H.decode_count++;
			/* H265_alloc_mmu: the picture's body pages at their largest (trimmed to what the core used once done); the
			 * slots no reference names and the plane does not hold give theirs back first (user space says which: with
			 * nothing said, all are kept) */
			uint32_t *map = (uint32_t *)(H.pool + MMU_MAP_OFF), pages = H.cm_body >> 12;
			if (s->referenced & (1u << 31)) {
				uint32_t showing = hakios_vdec_showing(H.n_pics), keep = s->referenced | showing | hakios_vdec_held(H.n_pics);
				for (uint32_t i = 0; i < H.n_pics; i++)
					if (i != s->slot && !(keep & (1u << i))) pages_trim(i, 0);
				/* short: the held pictures no reference names go back to their copies, never the plane's (it reads them) */
				if (P.top < pages)
					for (uint32_t i = 0; i < H.n_pics; i++)
						if (i != s->slot && !((s->referenced | showing) & (1u << i))) pages_trim(i, 0);
			}
			H.pic[H.cur].done = false;
			if (!pages_take(H.cur, pages)) {
				printf("hevc: no pages for a %dx%d picture (%u wanted, %u free)\n", w, h, pages, P.top);
				core_stop();
				H.header_pending = false;
				return ENOMEM;
			}
			for (uint32_t i = 0; i < pages; i++) map[i] = (pa(P.base) >> 12) + P.list[H.cur * P.per_slot + i];
			FlushPoC_DcacheRegion(H.pool + MMU_MAP_OFF, pages * 4);
			int sao_mem_unit = (H.lcu_size == 16 ? 9 : H.lcu_size == 32 ? 14 : 24) << 4;
			update_tile_info(H.lcu_x_num, H.lcu_y_num);
			WR(HEVC_sao_mem_unit, sao_mem_unit);
			WR(HEVC_SAO_ABV, H.ws.sao_abv.start);
			WR(HEVC_sao_vb_size, (sao_mem_unit + (2 << 4)) * H.lcu_y_num);
			WR(HEVC_SAO_VB, H.ws.sao_vb.start);
		} else if (++H.slice_idx >= MAX_SLICES) {
			return EINVAL;
		}
		if (poc_changed) {
			H.new_tile = 1;
			H.tile_x = H.tile_y = 0;
		} else if (H.tile_enabled) {
			int t = tile_index(rp->p.slice_segment_address, H.lcu_x_num);
			H.new_tile = t != (H.tile_x | H.tile_y << 8);
			H.tile_x = t & 0xff;
			H.tile_y = (t >> 8) & 0xff;
		} else {
			H.new_tile = 0;
		}
		if (H.new_tile) {
			H.tile_start_lcu_x = H.tile[H.tile_y][H.tile_x].start_cu_x;
			H.tile_start_lcu_y = H.tile[H.tile_y][H.tile_x].start_cu_y;
			H.tile_width_lcu = H.tile[H.tile_y][H.tile_x].width;
			H.tile_height_lcu = H.tile[H.tile_y][H.tile_x].height;
		}
		if (!H.tile_width_lcu || !H.tile_height_lcu) return EINVAL;
		int ldc = 0;
		if (H.slice_type != 2) {
			ldc = 1;
			for (int i = 0; i < H.ref_num[0] && ldc; i++) if (s->ref_poc[0][i] > s->poc) ldc = 0;
			for (int i = 0; i < H.ref_num[1] && ldc; i++) if (s->ref_poc[1][i] > s->poc) ldc = 0;
		}
		H.col = H.slice_type != 2 ? s->col_slot : -1;
		H.col_poc = s->col_poc;
		int merge = 5 - rp->p.five_minus_max_num_merge_cand;
		if (merge > 5 || merge < 1) merge = 5;
		config_mc_buffer(s);
		config_mcrcc();
		config_mpred(s, ldc, rp->p.collocated_from_l0_flag, rp->p.slice_temporal_mvp_enable_flag,
		    rp->p.dependent_slice_segment_flag ? 1 : 0, rp->p.log2_parallel_merge_level, merge);
		config_sao();
		WR(HEVC_DEC_STATUS_REG, 0x5);           /* HEVC_CODED_SLICE_SEGMENT_DAT */
		WR(HEVC_MCPU_INTR_REQ, 0x04);
	}
	H.header_pending = false;
	uint32_t st = wait_status();
	read_decode_info();
	uint32_t used = RD(HEVC_SAO_MMU_STATUS) >> 16;
	if (st == 0x8) {
		read_rpm();
		error = check_stream();
		if (error) {
			core_stop();
			return error;
		}
		H.header_pending = true;
		error = copy_rpm_out(s->rpm);
		return error ? error : EINPROGRESS;
	}
	core_stop();
	if (!(st == 0xa || st == 0x50 || st == 0x20 || st == 0x24 || st == 0x22 || st == 0xe)) {
		printf("hevc: picture in slot %u ended in status %#x (LCU %#x)\n", s->slot, st, RD(HEVC_PARSER_LCU_START));
		if (!(s->flags & TWL_HEVC_SKIP) && H.cur >= 0) pages_trim(H.cur, 0);
		return EIO;
	}
	if (s->flags & TWL_HEVC_SKIP) return ENODATA;
	/* the picture's body: what the core used of it (HEVC_SAO_MMU_STATUS[31:16], in pages) */
	if (used > 0 && used <= P.count[H.cur]) pages_trim(H.cur, used);
	H.pic[H.cur].done = true;
	vm_offset_t frame = H.pool + (H.pic[H.cur].y - (uint32_t)H.pool_pa);
	uint32_t bytes = H.stride * H.rows * 3 / 2;
	/* written behind the caches: for the CPU's copy, and so no stale line goes back over it */
	FlushPoC_DcacheRegion(frame, bytes);
	if (s->flags & TWL_HEVC_NOCOPY) return 0;
	if (s->output_size < bytes) return EINVAL;
	return copyout((const void *)frame, (user_addr_t)s->output, bytes);
}

bool
hakios_hevc_stream(uint32_t *slots, uint32_t *width, uint32_t *height, uint32_t *stride, uint32_t *rows)
{
	if (!H.layout) return false;
	*slots = H.n_pics;
	*width = (uint32_t)H.dw_w;                      /* (the copies') */
	*height = (uint32_t)H.dw_h;
	*stride = H.stride;
	*rows = H.rows;
	return true;
}

bool
hakios_hevc_picture(uint32_t slot, struct hakios_vdec_picture *out)
{
	if (!H.layout || slot >= H.n_pics) return false;
	out->luma = H.pic[slot].y;
	out->chroma = H.pic[slot].uv;
	out->stride = H.stride;
	out->rows = H.rows;
	out->width = (uint32_t)H.dw_w;
	/* the compressed picture while its pages are kept (a picture whose pages had to go: its copy) */
	out->compressed = H.pic[slot].done && P.count[slot] ? H.pic[slot].cm : 0;
	out->depth = (uint32_t)H.depth;
	out->coded_width = (uint32_t)H.pic_w;
	out->coded_height = (uint32_t)H.pic_h;
	return true;
}

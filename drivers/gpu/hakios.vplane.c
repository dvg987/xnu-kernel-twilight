/*
 * The Amlogic box's video plane (VD1): decoded pictures shown by the VPU under the OSD that carries tvOS, at the
 * HDMI output's own resolution.
 *
 * /dev/twl-vplane takes pictures from the decoder by token (sys/hakios.vdec.h); the plane itself is further down. For
 * bring-up, sysctl kern.hakios_vpu takes commands as a string (root) and gives the last command's results when read.
 *
 *   r REG [N]        read N (default 1, at most 64) VCBUS registers from REG (word index: 0xff000000 + REG * 4),
 *                    in the blocks vc_ok() lists only
 *   w REG VALUE      write one
 *   m REG MASK VALUE write VALUE into the bits of MASK
 *   p ADDR [N]       read N 32-bit words of RAM at physical ADDR
 *   a                alpha of what OSD1 scans: a histogram over a grid of its 1920x1080 ARGB frame
 *   b                the test picture buffer (16 MiB at the end of the decoder's pool): its address
 *   f W H PATTERN    a W x H NV12 test picture in it (0 colour bars, 1 luma ramp), stride W rounded up to 64
 *   s, h, o, k       the plane: see plane_command
 *
 * Numbers are hexadecimal; commands are separated by ';'.
 */
#include <arm/caches_internal.h>
#include <kern/locks.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <sys/errno.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>
#include <stdarg.h>
#include <os/atomic_private.h>
#include <sys/hakios.vplane.h>
#include <sys/hakios.vdec.h>
#include <sys/conf.h>
#include <kern/clock.h>
#include <mach/mach_time.h>
#include <miscfs/devfs/devfs.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern unsigned int ml_phys_read_word(vm_offset_t paddr);
extern uint64_t kvtophys(vm_offset_t va);
extern void IOSleep(unsigned int milliseconds);
extern void IODelay(unsigned int microseconds);
extern kern_return_t kernel_thread_start(thread_continue_t continuation, void *parameter, thread_t *new_thread);
extern void thread_deallocate(thread_t thread);
extern uint64_t hakios_hdmitx_frame_ns(void);
extern void hakios_vdec_plane_vsync(void);
extern wait_result_t assert_wait_deadline(event_t event, wait_interrupt_t interruptible, uint64_t deadline);
extern wait_result_t thread_block(thread_continue_t continuation);
extern kern_return_t thread_wakeup_prim(event_t event, boolean_t one_thread, wait_result_t result);
extern kern_return_t clear_wait(thread_t thread, wait_result_t result);
static volatile uint32_t vsync_seq;     /* the worker's vertical blankings */

#define VCBUS_BASE      0xff000000ull
#define VCBUS_SIZE      0x40000u
#define RAM_END         0x7f800000ull
#define BUFFER_SIZE     (16u << 20)

#define VIU_OSD1_BLK1_CFG_W4    0x1a14
#define VIU_OSD1_BLK2_CFG_W4    0x1a15

static vm_offset_t vcbus;
static vm_offset_t buffer;
static uint64_t buffer_pa;
static char result[4096];
static size_t result_len;
static LCK_GRP_DECLARE(vpu_grp, "hakios-vpu");
static LCK_MTX_DECLARE(vpu_lock, &vpu_grp);

static void
out(const char *format, ...)
{
	va_list args;

	if (result_len >= sizeof(result) - 1) {
		return;
	}
	va_start(args, format);
	int n = vsnprintf(result + result_len, sizeof(result) - result_len, format, args);
	va_end(args);
	if (n > 0) {
		result_len += (size_t)n;
		if (result_len > sizeof(result) - 1) {
			result_len = sizeof(result) - 1;
		}
	}
}

static uint32_t
vc_rd(uint32_t reg)
{
	return *(volatile uint32_t *)(vcbus + (vm_offset_t)reg * 4);
}

static void
vc_wr(uint32_t reg, uint32_t value)
{
	*(volatile uint32_t *)(vcbus + (vm_offset_t)reg * 4) = value;
	__builtin_arm_dsb(0xf);
}

/*
 * Only the blocks the plane and the OSD use: a read of an address nothing answers stalls the bus, and the box with it
 * (a sweep of whole ranges did, 2026-11-07).
 */
static bool
vc_ok(uint64_t reg, uint64_t count)
{
	static const uint32_t blocks[][2] = {
		{ 0x1a00, 0x1aff },     /* VIU, OSD1/2 */
		{ 0x1b00, 0x1cff },     /* ENCP and the other encoders */
		{ 0x1d00, 0x1dff },     /* VPP */
		{ 0x2700, 0x27ff },     /* VPU arbitration, ENCP_INFO_READ */
		{ 0x3200, 0x32ff },     /* video adjust and colour matrices */
		{ 0x3800, 0x39ff },     /* HDR, OSD blend, post-blend */
		{ 0x3d00, 0x3dff },     /* OSD wrap matrices */
		{ 0x4800, 0x48ff },     /* VD1 memory interface, AFBC */
		{ 0x5000, 0x52ff },     /* VSR, SAFA, sharpness */
	};
	if (count == 0 || count > 64) {
		return false;
	}
	for (unsigned i = 0; i < sizeof(blocks) / sizeof(blocks[0]); i++) {
		if (reg >= blocks[i][0] && reg + count - 1 <= blocks[i][1]) {
			return true;
		}
	}
	return false;
}

static uint32_t
osd_pixel(uint64_t base, uint64_t stride, unsigned x, unsigned y)
{
	return ml_phys_read_word(base + (uint64_t)y * stride + (uint64_t)x * 4);
}

static void
osd_alpha(void)
{
	uint64_t base = (uint64_t)vc_rd(VIU_OSD1_BLK1_CFG_W4) << 4;
	uint64_t stride = (uint64_t)(vc_rd(VIU_OSD1_BLK2_CFG_W4) & 0xfff) * 16;
	unsigned bins[4] = { 0 }, samples = 0;

	if (!base || stride < 1920 * 4 || base + stride * 1080 > RAM_END) {
		out("a: OSD1 frame at %llx, stride %llu: not a 1920x1080 frame in RAM\n", base, stride);
		return;
	}
	for (unsigned y = 5; y < 1080; y += 20) {
		for (unsigned x = 5; x < 1920; x += 20) {
			unsigned alpha = osd_pixel(base, stride, x, y) >> 24;
			bins[alpha == 0 ? 0 : alpha < 128 ? 1 : alpha < 255 ? 2 : 3]++;
			samples++;
		}
	}
	out("a: OSD1 %llx stride %llu: %u samples, alpha 0: %u, 1-127: %u, 128-254: %u, 255: %u; centre %08x\n",
	    base, stride, samples, bins[0], bins[1], bins[2], bins[3], osd_pixel(base, stride, 960, 540));
}

static bool
buffer_get(void)
{
	if (buffer) {
		return true;
	}
	/* contiguous memory this large is not to be had after boot: the end of the decoder's pool */
	int error = hakios_vdec_scratch(BUFFER_SIZE, &buffer, &buffer_pa);
	if (error) {
		buffer = 0;
		out("b: no test picture memory: %d\n", error);
		return false;
	}
	return true;
}

static void
fill(unsigned width, unsigned height, unsigned pattern)
{
	static const uint8_t bars[8][3] = {     /* BT.709 limited range: white yellow cyan green magenta red blue black */
		{ 235, 128, 128 }, { 219, 16, 138 }, { 188, 154, 16 }, { 173, 42, 26 },
		{ 78, 214, 230 }, { 63, 102, 240 }, { 32, 240, 118 }, { 16, 128, 128 },
	};
	unsigned stride = (width + 63) & ~63u;

	if (width < 16 || height < 16 || width > 4096 || height > 2304 || (size_t)stride * height * 3 / 2 > BUFFER_SIZE) {
		out("f: bad size %ux%u\n", width, height);
		return;
	}
	uint8_t *y = (uint8_t *)buffer, *uv = y + (size_t)stride * height;
	/* 0: the bars across, 1: a luma ramp across (no colour), 2: the bars down (eight bands, top white to bottom black) */
	for (unsigned row = 0; row < height; row++) {
		for (unsigned x = 0; x < width; x++) {
			y[(size_t)row * stride + x] = pattern == 1 ? (uint8_t)(16 + x * 219 / width) :
			    pattern == 2 ? bars[row * 8 / height][0] : bars[x * 8 / width][0];
		}
	}
	for (unsigned row = 0; row < height / 2; row++) {
		for (unsigned x = 0; x < width; x += 2) {
			const uint8_t *c = pattern == 1 ? bars[7] : pattern == 2 ? bars[row * 2 * 8 / height] : bars[x * 8 / width];
			uv[(size_t)row * stride + x] = c[1];
			uv[(size_t)row * stride + x + 1] = c[2];
		}
	}
	FlushPoC_DcacheRegion(buffer, (unsigned)((size_t)stride * height * 3 / 2));
	out("f: %ux%u NV12, Y %llx, CbCr %llx, stride %u\n", width, height, buffer_pa,
	    buffer_pa + (uint64_t)stride * height, stride);
}

static void plane_command(char op, const uint64_t *v, unsigned n);

/* the secure monitor's power control (Amlogic's sec_pwr_ctrl): 0x82000093 switches a domain, 0x82000095 reads one */
#define SIP_PWR_SET     0x82000093u
#define SIP_PWR_GET     0x82000095u
static uint64_t
sip(uint64_t function, uint64_t a1, uint64_t a2)
{
	register uint64_t x0 __asm__("x0") = function;
	register uint64_t x1 __asm__("x1") = a1;
	register uint64_t x2 __asm__("x2") = a2;
	register uint64_t x3 __asm__("x3") = 0;

	__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3) : : "x4", "x5", "x6", "x7", "x8", "x9",
	    "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return x0;
}

static void
command(char *line)
{
	char *word[8];
	unsigned n = 0;
	uint64_t v[7] = { 0 };

	for (char *p = line; *p && n < 8;) {
		while (*p == ' ' || *p == '\t' || *p == '\n') {
			p++;
		}
		if (!*p) {
			break;
		}
		word[n++] = p;
		while (*p && *p != ' ' && *p != '\t' && *p != '\n') {
			p++;
		}
		if (*p) {
			*p++ = 0;
		}
	}
	if (n == 0) {
		return;
	}
	for (unsigned i = 1; i < n; i++) {
		v[i - 1] = strtoq(word[i], NULL, 16);
	}
	switch (word[0][0]) {
	case 'r': {
		uint64_t count = n > 2 ? v[1] : 1;
		if (!vc_ok(v[0], count) || count > 64) {
			out("r: bad range\n");
			break;
		}
		for (uint64_t i = 0; i < count; i++) {
			out("%04llx=%08x%s", v[0] + i, vc_rd((uint32_t)(v[0] + i)), (i % 4 == 3 || i + 1 == count) ? "\n" : " ");
		}
		break;
	}
	case 'w':
		if (n != 3 || !vc_ok(v[0], 1)) {
			out("w: bad arguments\n");
			break;
		}
		vc_wr((uint32_t)v[0], (uint32_t)v[1]);
		out("%04llx<-%08x (now %08x)\n", v[0], (uint32_t)v[1], vc_rd((uint32_t)v[0]));
		break;
	case 'm':
		if (n != 4 || !vc_ok(v[0], 1)) {
			out("m: bad arguments\n");
			break;
		}
		vc_wr((uint32_t)v[0], (vc_rd((uint32_t)v[0]) & ~(uint32_t)v[1]) | ((uint32_t)v[2] & (uint32_t)v[1]));
		out("%04llx=%08x\n", v[0], vc_rd((uint32_t)v[0]));
		break;
	case 'p': {
		uint64_t count = n > 2 ? v[1] : 1;
		/* not the secure world's RAM (0x05000000-0x06c00000): a normal-world read of it faults */
		if (count > 64 || (v[0] & 3) || v[0] + count * 4 > RAM_END ||
		    (v[0] < 0x06c00000ull && v[0] + count * 4 > 0x05000000ull)) {
			out("p: bad range\n");
			break;
		}
		for (uint64_t i = 0; i < count; i++) {
			out("%08x%s", ml_phys_read_word(v[0] + i * 4), (i % 8 == 7 || i + 1 == count) ? "\n" : " ");
		}
		break;
	}
	case 'a':
		osd_alpha();
		break;
	case 'b':
		if (buffer_get()) {
			out("b: %llx, %u bytes\n", buffer_pa, BUFFER_SIZE);
		}
		break;
	case 'f':
		if (n == 4 && buffer_get()) {
			fill((unsigned)v[0], (unsigned)v[1], (unsigned)v[2]);
		} else if (n != 4) {
			out("f: W H PATTERN\n");
		}
		break;
	case 'P':
		/* P ID [ON]: a power domain's or (domain count + VPU module) memory's state, switched first with ON */
		if (n < 2 || v[0] > 255) {
			out("P: ID [0|1]\n");
			break;
		}
		if (n > 2) {
			out("P %llx set %llx: %llx; ", v[0], v[1], sip(SIP_PWR_SET, v[0], v[1] ? 1 : 0));
		}
		out("P %llx state %llx\n", v[0], sip(SIP_PWR_GET, v[0], 0));
		break;
	case 's':
	case 'h':
	case 'o':
	case 'k':
	case 'c':
		plane_command(word[0][0], v, n - 1);
		break;
	default:
		out("%s: unknown command\n", word[0]);
	}
}


/* ------------------------------------------------------------------------------------------------------------ */
/*
 * The plane. VD1's memory interface reads the picture straight from the decoder's slot (linear NV12, no canvas), the
 * VSR/SAFA scaler takes it to the rectangle, and the post-blend puts it under OSD1. Register recipe: Amlogic's
 * video_sink driver, S7D paths (video_hw.c, video_safa.c, vpp.c); indexes in VCBUS words.
 *
 * Where it goes: the player draws the key colour there (twlcvgl.c). OSD1's colour key makes those pixels transparent,
 * and the worker finds the rectangle by looking for them in the frame OSD1 scans (1920x1080, scaled to the output).
 * When the key has gone from there for a while the plane goes off.
 */
#define VPP_LINE_IN_LENGTH      0x1d01
#define VPP_PIC_IN_HEIGHT       0x1d02
#define VPP_PREBLEND_VD1_H      0x1d1a
#define VPP_PREBLEND_VD1_V      0x1d1b
#define VPP_POSTBLEND_VD1_H     0x1d1c
#define VPP_POSTBLEND_VD1_V     0x1d1d
#define VPP_PREBLEND_H_SIZE     0x1d20
#define VPP_HOLD_LINES          0x1d22
#define VPP_MISC                0x1d26
#define VPP_OFIFO_SIZE          0x1d27
#define VPP_MATRIX_CTRL         0x1d5f
#define VPP_MISC1               0x1d76
#define VPP_VE_H_V_SIZE         0x1da4
#define VPP_OUT_H_V_SIZE        0x1da5
#define VPP_IN_H_V_SIZE         0x1da6
#define VPP_INPUT_CTRL          0x1dab
#define VPP_VD1_CLIP_MISC0      0x1de1
#define VPP_VD1_CLIP_MISC1      0x1de2
#define VD1_BLEND_SRC_CTRL      0x1dfb
#define VPP_POST_BLEND_DUMMY_ALPHA 0x3969
#define AMDV_PATH_CTRL          0x1a0c
#define VD1_AFBCD0_MISC_CTRL    0x1a0a
#define VIU_OSD1_CTRL_STAT      0x1a10
#define VIU_OSD1_TCOLOR_AG0     0x1a17
#define VIU_OSD1_BLK0_CFG_W0    0x1a1b
#define VIU_OSD1_CTRL_STAT2     0x1a2d
#define VD1_IF0_GEN_REG         0x4800
#define VD1_IF0_LUMA_X0         0x4803
#define VD1_IF0_LUMA_Y0         0x4804
#define VD1_IF0_CHROMA_X0       0x4805
#define VD1_IF0_CHROMA_Y0       0x4806
#define VD1_IF0_LUMA_X1         0x4807
#define VD1_IF0_LUMA_Y1         0x4808
#define VD1_IF0_CHROMA_X1       0x4809
#define VD1_IF0_CHROMA_Y1       0x480a
#define VD1_IF0_RPT_LOOP        0x480b
#define VD1_IF0_LUMA_PSEL       0x4810
#define VD1_IF0_CHROMA_PSEL     0x4811
#define VD1_IF0_GEN_REG2        0x4819
#define VD1_IF0_GEN_REG3        0x481c
#define VIU_VD1_FMT_CTRL        0x481d
/* VD1's decompressor for the decoders' compressed pictures (vd1_set_dcu, vd_afbc_setting_tl1, vd1_path_select). On
 * the S7D it is SC2's, at 0x4840 (the vendor kernel's vd_afbc_reg_sc2_array; 0x1ae0, where U-Boot's register.h has
 * it, reads back 0) */
#define VD1_AFBCD0_MISC_CTRL    0x1a0a
#define AFBC_ENABLE             0x4840
#define AFBC_MODE               0x4841
#define AFBC_SIZE_IN            0x4842
#define AFBC_DEC_DEF_COLOR      0x4843
#define AFBC_CONV_CTRL          0x4844
#define AFBC_HEAD_BADDR         0x4846
#define AFBC_SIZE_OUT           0x4848
#define AFBC_VD_CFMT_W          0x484c
#define AFBC_VD_CFMT_H          0x4851
#define AFBC_BODY_BADDR         0x4847
#define AFBC_VD_CFMT_CTRL       0x484b
#define AFBC_MIF_HOR_SCOPE      0x484d
#define AFBC_MIF_VER_SCOPE      0x484e
#define AFBC_PIXEL_HOR_SCOPE    0x484f
#define AFBC_PIXEL_VER_SCOPE    0x4850
#define AFBCDEC_IQUANT_ENABLE   0x4852
#define VIU_VD1_FMT_W           0x481e
#define VD1_IF0_BADDR_Y         0x4820
#define VD1_IF0_BADDR_CB        0x4821
#define VD1_IF0_BADDR_CR        0x4822
#define VD1_IF0_STRIDE_0        0x4823
#define VD1_IF0_STRIDE_1        0x4824
#define VD1_AFBC_ENABLE         0x4840
#define VPP_VSR_TOP_MISC        0x5000
#define VPP_VSR_TOP_IN_SIZE     0x5002
#define VPP_VSR_TOP_OUT_SIZE    0x5004
#define VPP_VSR_TOP_C42C44_MODE 0x5008
#define VPP_PI_EN_MODE          0x5048
#define SAFA_PPS_SR_422_EN      0x5100
#define SAFA_PPS_VSC_STEP       0x5101
#define SAFA_PPS_HSC_STEP       0x5102
#define SAFA_PPS_SC_MISC        0x5103
#define SAFA_PPS_PRE_HSCALE_Y1  0x5105
#define SAFA_PPS_PRE_HSCALE_Y0  0x5106
#define SAFA_PPS_PRE_HSCALE_C1  0x5107
#define SAFA_PPS_PRE_HSCALE_C0  0x5108
#define SAFA_PPS_PRE_SCALE      0x5109
#define SAFA_PPS_PRE_VSCALE     0x510a
#define SAFA_PPS_VSC_INIT       0x510b
#define SAFA_PPS_HSC_INIT       0x510c
#define SAFA_PPS_INTERP_EN_MODE 0x510d
#define SAFA_PPS_YUV_SHARPEN_EN 0x5116
#define SAFA_PPS_DIR_EN_MODE    0x511e
#define SAFA_PPS_SR_ALP_INFO    0x515b
#define SAFA_PPS_PI_INFO        0x515c
#define SAFA_PPS_HW_CTRL        0x5190
#define SAFA_COEF_Y_IDX         0x5197
#define SAFA_COEF_Y             0x5198
#define SAFA_COEF_C_IDX         0x5199
#define SAFA_COEF_C             0x519a
#define VPP_SR_EN               0x5204
#define ENCP_VIDEO_MAX_LNCNT    0x1bae
#define ENCP_DE_V_BEGIN         0x1c3c
#define ENCP_DE_V_END           0x1c3d
#define ENCP_INFO_READ          0x271d

#define OSD_WIDTH       1920
#define OSD_HEIGHT      1080
#define KEY_RGB         TWL_VPLANE_KEY_RGB
/* a pixel of the key: the player's conversion of the key's YCbCr lands near magenta, not always on it (fe00ff with a
 * limited-range clip), and the window server keeps it as it is */
static inline bool
key_pixel(uint32_t p)
{
	/* 0: one hakios_vplane_osd_frame() has cleared already (the window server's pixels are all opaque) */
	return p == 0 || ((p >> 16 & 0xff) >= 0xe8 && (p >> 8 & 0xff) <= 0x18 && (p & 0xff) >= 0xe8);
}

/* the key under something grey the player drew over it, partly transparent (its bars, their text): R = B = a*c +
 * (1 - a)*255, G = a*c. Pure grey (R = G = B) is not; nor a colour (R and B apart) */
static inline bool
key_tinted(uint32_t p)
{
	int r = p >> 16 & 0xff, g = p >> 8 & 0xff, b = p & 0xff;
	/* (not opaque: hakios_vplane_osd_frame's own work, a frame the player has not drawn again; the window server's
	 * pixels are all opaque) */
	return (p >> 24) != 0xff || ((r > b ? r - b : b - r) <= 6 && r >= g + 0x20);
}

/* Lanczos (a = taps / 2), 33 phases from 0 to half a pixel (the hardware mirrors the rest), summing to 512 */
static const int16_t lanczos4[33][4] = {
	{0, 512, 0, 0},
	{-5, 512, 5, 0},
	{-10, 511, 11, 0},
	{-14, 509, 17, 0},
	{-18, 508, 23, -1},
	{-21, 505, 29, -1},
	{-25, 502, 36, -1},
	{-28, 499, 43, -2},
	{-31, 494, 51, -2},
	{-33, 490, 58, -3},
	{-35, 484, 66, -3},
	{-37, 478, 75, -4},
	{-39, 473, 83, -5},
	{-40, 466, 92, -6},
	{-41, 459, 101, -7},
	{-42, 452, 110, -8},
	{-43, 445, 119, -9},
	{-43, 436, 129, -10},
	{-44, 428, 139, -11},
	{-44, 420, 149, -13},
	{-44, 411, 159, -14},
	{-43, 401, 169, -15},
	{-43, 392, 180, -17},
	{-42, 382, 190, -18},
	{-41, 372, 201, -20},
	{-41, 363, 211, -21},
	{-40, 353, 222, -23},
	{-39, 342, 233, -24},
	{-37, 331, 244, -26},
	{-36, 321, 255, -28},
	{-35, 310, 266, -29},
	{-33, 299, 277, -31},
	{-32, 288, 288, -32},
};
static const int16_t lanczos6[33][6] = {
	{0, 0, 512, 0, 0, 0},
	{2, -6, 511, 7, -2, 0},
	{3, -13, 511, 14, -3, 0},
	{5, -19, 510, 21, -5, 0},
	{6, -24, 508, 29, -7, 0},
	{7, -29, 506, 37, -9, 0},
	{8, -34, 503, 45, -11, 1},
	{9, -39, 501, 53, -13, 1},
	{10, -44, 499, 62, -16, 1},
	{11, -48, 495, 71, -18, 1},
	{12, -51, 490, 80, -20, 1},
	{13, -55, 486, 89, -23, 2},
	{14, -58, 480, 99, -25, 2},
	{14, -61, 475, 108, -27, 3},
	{15, -64, 470, 118, -30, 3},
	{15, -66, 464, 128, -32, 3},
	{15, -68, 457, 139, -35, 4},
	{16, -70, 450, 149, -37, 4},
	{16, -72, 443, 160, -40, 5},
	{16, -73, 436, 170, -42, 5},
	{16, -74, 428, 181, -45, 6},
	{16, -75, 420, 192, -47, 6},
	{16, -75, 411, 203, -50, 7},
	{16, -76, 403, 214, -52, 7},
	{16, -76, 393, 225, -54, 8},
	{15, -76, 384, 236, -56, 9},
	{15, -75, 375, 247, -59, 9},
	{15, -75, 365, 258, -61, 10},
	{14, -74, 356, 269, -63, 10},
	{14, -73, 344, 281, -65, 11},
	{13, -72, 335, 291, -66, 11},
	{13, -71, 324, 302, -68, 12},
	{13, -70, 313, 313, -70, 13},
};
static const int16_t lanczos8[33][8] = {
	{0, 0, 0, 512, 0, 0, 0, 0},
	{-1, 3, -7, 512, 7, -3, 1, 0},
	{-2, 5, -14, 511, 15, -5, 2, 0},
	{-2, 7, -20, 509, 23, -8, 3, 0},
	{-3, 10, -27, 509, 31, -11, 3, 0},
	{-4, 12, -32, 507, 39, -14, 4, 0},
	{-4, 14, -38, 504, 48, -17, 5, 0},
	{-5, 16, -43, 500, 57, -19, 6, 0},
	{-5, 17, -48, 498, 66, -22, 7, -1},
	{-6, 19, -53, 495, 75, -25, 8, -1},
	{-6, 21, -58, 491, 84, -29, 10, -1},
	{-6, 22, -62, 486, 94, -32, 11, -1},
	{-7, 24, -66, 481, 104, -35, 12, -1},
	{-7, 25, -69, 475, 114, -38, 13, -1},
	{-7, 26, -72, 470, 124, -41, 14, -2},
	{-8, 27, -75, 465, 134, -44, 15, -2},
	{-8, 28, -78, 458, 145, -47, 16, -2},
	{-8, 29, -80, 451, 155, -50, 17, -2},
	{-8, 30, -82, 444, 166, -53, 18, -3},
	{-8, 31, -84, 436, 177, -56, 19, -3},
	{-8, 31, -86, 430, 187, -59, 20, -3},
	{-8, 32, -87, 419, 198, -61, 22, -3},
	{-8, 32, -88, 412, 209, -64, 23, -4},
	{-8, 32, -89, 404, 220, -67, 24, -4},
	{-8, 32, -89, 395, 231, -69, 24, -4},
	{-8, 33, -89, 386, 242, -72, 25, -5},
	{-8, 32, -89, 377, 253, -74, 26, -5},
	{-8, 32, -89, 367, 264, -76, 27, -5},
	{-7, 32, -89, 357, 274, -78, 28, -5},
	{-7, 32, -88, 347, 285, -80, 29, -6},
	{-7, 32, -87, 337, 296, -82, 29, -6},
	{-7, 31, -86, 328, 306, -84, 30, -6},
	{-6, 31, -85, 315, 317, -85, 31, -6},
};


struct plane_picture {
	struct hakios_vdec_picture picture;
	uint32_t width, height, slot;           /* shown: the top left width x height */
};
struct rect { uint32_t x, y, w, h; };

static struct {
	bool on, ports, pending, stop, key, tables, inited, test, reprogram, afbc;
	bool direct;                            /* a window server's video layer: its place is given, no key */
	struct plane_picture shown, next;
	struct rect dst, test_dst;              /* on the output; the bring-up command's */
	struct rect direct_rect;                /* the video layer's, in OSD1's frame */
	struct rect key_rect;                   /* in OSD1's frame; w == 0: not found */
	struct rect key_seen;                   /* the last scan's, which the next must match to become key_rect */
	uint64_t key_scanned;                   /* mach_absolute_time */
	unsigned key_misses, frames;
	uint32_t misc_saved;
	/* cadence, reported every five seconds while on: vsyncs with a new picture, with the same one again, pictures
	 * replaced before a vsync showed them, and the gaps between the window server's pictures (ms: <12, <20, <30, <45,
	 * more) */
	struct { uint32_t vsyncs, shown, repeated, overwritten, gaps[5], gap_max, writes_blank, writes_active, write_line_max; uint64_t last_in, since; } c;
} plane;
static LCK_MTX_DECLARE(plane_lock, &vpu_grp);
static bool worker_started;
/* the scaler's line buffer: wider pictures it has to scale are made narrower first (compressed ones by the
 * decompressor's skip, copies by the pre-scaler) */
static unsigned wide_px = 2048;
SYSCTL_UINT(_kern, OID_AUTO, hakios_vplane_wide, CTLFLAG_RW | CTLFLAG_LOCKED, &wide_px, 0, "");
static bool use_compressed = true;      /* the decoder's compressed picture when there is one (kern.hakios_vpu "c 0/1") */

static void
vc_m(uint32_t reg, uint32_t mask, uint32_t value)
{
	vc_wr(reg, (vc_rd(reg) & ~mask) | (value & mask));
}

static bool
vpu_map(void)
{
	if (!vcbus) {
		vcbus = ml_io_map(VCBUS_BASE, VCBUS_SIZE);
	}
	return vcbus != 0;
}

/* until the encoder's next vertical blanking begins (as MiniIOMobileFramebuffer's flips), at most ~2 frames */
static void
wait_blanking(void)
{
	uint32_t lines = vc_rd(ENCP_VIDEO_MAX_LNCNT) + 1, begin = vc_rd(ENCP_DE_V_BEGIN), end = vc_rd(ENCP_DE_V_END);
	if (end <= begin || end >= lines || lines > 8192) {
		IOSleep(16);
		return;
	}
	/* (the mode's own frame: at 23.976 Hz one is 41.7 ms, longer than the 40 ms this waited at most) */
	uint64_t frame = hakios_hdmitx_frame_ns(), per_line, deadline, ticks;
	if (frame == 0) frame = NSEC_PER_SEC / 60;
	per_line = frame / lines;
	clock_interval_to_deadline(2 * frame + 2 * NSEC_PER_MSEC, 1, &deadline);
	bool seen_picture = false;
	while (mach_absolute_time() < deadline) {
		uint32_t line = vc_rd(ENCP_INFO_READ) >> 16 & 0x1fff;
		bool in_picture = line >= begin && line < end;
		if (!in_picture && seen_picture) {
			return;
		}
		seen_picture |= in_picture;
		uint32_t left = in_picture ? end - line : 0;
		if (left * per_line > 2500 * NSEC_PER_USEC) {
			nanoseconds_to_absolutetime(left * per_line - 2000 * NSEC_PER_USEC, &ticks);
			clock_delay_until(mach_absolute_time() + ticks);
		} else {
			IODelay(20);
		}
	}
}

static uint32_t
coef_word(int a, int b)
{
	return ((uint32_t)(uint16_t)(int16_t)a << 16) | (uint16_t)(int16_t)b;
}

static void
load_bank(uint32_t idx_reg, uint32_t data_reg, uint32_t bank, const int16_t *table, unsigned taps, unsigned first,
    bool twice)
{
	vc_wr(idx_reg, bank);
	for (unsigned i = 0; i < 33; i++) {
		const int16_t *c = table + i * taps + first;
		vc_wr(data_reg, coef_word(c[0], c[1]));
		vc_wr(data_reg, twice ? coef_word(c[0], c[1]) : coef_word(c[2], c[3]));
	}
}

static void
load_tables(void)
{
	static const uint32_t index[2] = { SAFA_COEF_Y_IDX, SAFA_COEF_C_IDX }, data[2] = { SAFA_COEF_Y, SAFA_COEF_C };
	for (unsigned i = 0; i < 2; i++) {
		load_bank(index[i], data[i], 0x000, &lanczos4[0][0], 4, 0, false);
		load_bank(index[i], data[i], 0x080, &lanczos8[0][0], 8, 0, false);
		load_bank(index[i], data[i], 0x100, &lanczos8[0][0], 8, 4, false);
		load_bank(index[i], data[i], 0x180, &lanczos4[0][0], 4, 0, false);
		load_bank(index[i], data[i], 0x200, &lanczos6[0][0], 6, 0, false);
		load_bank(index[i], data[i], 0x280, &lanczos6[0][0], 6, 4, true);
		load_bank(index[i], data[i], 0x300, &lanczos4[0][0], 4, 0, false);
	}
	plane.tables = true;
}

static void
plane_init_once(void)
{
	if (plane.inited) {
		return;
	}
	vc_m(VPP_OFIFO_SIZE, 0x1fff, 0x1000);
	vc_m(VPP_MATRIX_CTRL, 0x7c00, 0);
	vc_wr(VPP_POST_BLEND_DUMMY_ALPHA, 0x7fffffff);
	vc_m(VPP_MISC1, 0x1ff, 0x100);
	/* AMDV_PATH_CTRL is left as U-Boot set it (0xb): 0xf, the recipe's Dolby bypass, changes the OSD's output too */
	vc_wr(VPP_INPUT_CTRL, 0x440);
	vc_wr(VD1_AFBCD0_MISC_CTRL, 0);
	vc_wr(VD1_AFBC_ENABLE, 0);
	vc_wr(VPP_VD1_CLIP_MISC0, 0x3fffffff);
	vc_wr(VPP_VD1_CLIP_MISC1, 0);
	plane.inited = true;
}

/* phase step and initial phase of a SAFA scaler from in to out pixels (set_cfg_pi_safa) */
static void
scaler(uint32_t in, uint32_t out, uint32_t *step, uint32_t *init)
{
	*step = (uint32_t)(((uint64_t)in << 24) / out) & 0x0fffffff;
	if (in < out) {
		*init = 0x1f << 16 | (((in << 16) / out + 0x10000) / 2 & 0xffff);
	} else {
		uint32_t t = ((in << 16) / out - 0x10000) / 2;
		*init = (t >> 16 & 0x1f) << 16 | (t & 0xffff);
	}
}

static void
program_picture(const struct plane_picture *p)
{
	if (plane.afbc) {
		/* the header names the body's pages (the decoder's MMU wrote them): no body address */
		vc_wr(AFBC_HEAD_BADDR, (uint32_t)(p->picture.compressed >> 4));
		vc_wr(AFBC_BODY_BADDR, 0);
		vc_wr(AFBC_ENABLE, 0x1700 | 1u << 19);  /* decoder on, its input the DOS's (uncompressed path off) */
		return;
	}
	uint32_t s = (p->picture.stride + 15) >> 4;
	/* bit 16 of STRIDE_1: the addresses here, not a canvas. The vendor passes it in a 16-bit field (so it seems never
	 * written), yet without it VD1 reads whatever canvas VD1_IF0_CANVAS0 names. The second set (_F1, at +5) is the
	 * same picture: VD1 reads it every other frame (left over at 0 it showed canvas memory between the frames). */
	for (uint32_t set = 0; set <= 5; set += 5) {
		vc_wr(VD1_IF0_BADDR_Y + set, (uint32_t)(p->picture.luma >> 4));
		vc_wr(VD1_IF0_BADDR_CB + set, (uint32_t)(p->picture.chroma >> 4));
		vc_wr(VD1_IF0_BADDR_CR + set, 0);
		vc_wr(VD1_IF0_STRIDE_0 + set, s | s << 16);
		vc_m(VD1_IF0_STRIDE_1 + set, 0x1ffff, 1u << 16 | s);
	}
}

/* everything for a picture of this size in this rectangle; VD1 reads from then on */
static void
program_layer(const struct plane_picture *p, const struct rect *d)
{
	uint32_t w = p->width, h = p->height, skip = 1;

	plane_init_once();
	plane.afbc = use_compressed && p->picture.compressed;
	/* the scaler's line buffer holds 2048 pixels: a wider line it scales wraps round (a 1440p picture onto the 4K
	 * output came out with the 512 pixels past 2048 on its left). A compressed picture wider than that, scaled either
	 * way, is read every other pixel by the decompressor (the vendor's hscale_skip); 4K onto 4K is not scaled */
	if (plane.afbc && w > wide_px && (w != d->w || h != d->h)) {
		skip = 2;
	}
	if (plane.afbc) {
		/* AFBC0 into VD1: AXI and data from it, its line memory (bits 12, 10, 22) */
		vc_m(VD1_AFBCD0_MISC_CTRL, 0x7fffff, 1u << 22 | 1u << 12 | 1u << 10);
		uint32_t depth = p->picture.depth == 10 ? 2u : 0u;    /* BITDEPTH_Y/U/V 10 */
		/* mif urgent 3, hold 16 lines (8: a letterboxed 4K picture after a scene cut came out with a line of coloured
		 * dots a few dozen lines down, the decompressor short of data where it starts after the black bar), burst 2,
		 * the sample sizes, scatter (the body in pages) */
		vc_wr(AFBC_MODE, 3u << 24 | 16u << 16 | 2u << 14 | (depth << 8 | depth << 10 | depth << 12) | 1u << 29 |
		    (skip == 2 ? 0x33u : 0));
		vc_wr(AFBC_CONV_CTRL, 0x100 | 2u << 12);              /* line buffer 0x100, 4:2:0 */
		vc_wr(AFBC_DEC_DEF_COLOR, 0x3ffu << 20 | 0x80u << (depth + 10) | 0x80u << depth);
		/* chroma formatter: repeat, Y:C 2:1 horizontally, vertical phase 8, initial 0xc, line 0 repeated */
		vc_wr(AFBC_VD_CFMT_CTRL, 1u << 28 | 1u << 21 | 1u << 20 | 0x8u << 1 | 1u << 16 | 0xcu << 8 | 1u << 0);
		vc_wr(AFBCDEC_IQUANT_ENABLE, 0);
		uint32_t wa = (uint32_t)((w + 31) & ~31u), ha = (h + 3) & ~3u;
		vc_wr(AFBC_MIF_HOR_SCOPE, (w - 1) / 32);
		/* the sizes in the picture's pixels, skip or not: the decompressor drops every other pixel itself (AFBC_MODE
		 * above); halved here as well, it gave lines a quarter as wide, two of them side by side (vd_afbc_setting_tl1
		 * writes only the scopes and the input size) */
		vc_wr(AFBC_PIXEL_HOR_SCOPE, w - 1);
		vc_wr(AFBC_MIF_VER_SCOPE, (h - 1) / 4);
		vc_wr(AFBC_PIXEL_VER_SCOPE, h - 1);
		vc_wr(AFBC_SIZE_IN, wa << 16 | ha);
		vc_wr(AFBC_SIZE_OUT, wa << 16 | ha);
		vc_wr(AFBC_VD_CFMT_W, wa << 16 | wa / 2);
		vc_wr(AFBC_VD_CFMT_H, ha);
		w /= skip;                              /* what the rest of the path takes */
		vc_wr(VD1_IF0_GEN_REG, 0);
		program_picture(p);
	} else {
		vc_wr(AFBC_ENABLE, 0);
		vc_m(VD1_AFBCD0_MISC_CTRL, 1u << 22 | 1u << 12 | 1u << 10, 0);
		program_picture(p);
	}
	/* burst length in [2:1]: 2 for a stride of whole 64 bytes, 1 for 32, else 0 (vd1_set_dcu) */
	vc_wr(VD1_IF0_GEN_REG3, 0xa000 | (p->picture.stride % 32 ? 0 : p->picture.stride % 64 ? 1u << 1 : 2u << 1));
	vc_m(VD1_IF0_GEN_REG2, 0x3f, 2);        /* NV12 */
	vc_wr(VIU_VD1_FMT_CTRL, 0x20310c11);    /* 4:2:0 to 4:4:4 */
	vc_wr(VD1_IF0_RPT_LOOP, 0);
	for (uint32_t r = 0x480c; r <= 0x480f; r++) {
		vc_wr(r, 0);
	}
	/* one picture: the vendor's "progressive frame in two pictures" (0x0a000201) gave every line twice, the top half */
	vc_wr(VD1_IF0_LUMA_PSEL, 0);
	vc_wr(VD1_IF0_CHROMA_PSEL, 0);
	vc_wr(VD1_IF0_LUMA_X0, (w - 1) << 16);
	vc_wr(VD1_IF0_LUMA_X1, (w - 1) << 16);
	vc_wr(VD1_IF0_CHROMA_X0, ((w - 1) >> 1) << 16);
	vc_wr(VD1_IF0_CHROMA_X1, ((w - 1) >> 1) << 16);
	vc_wr(VIU_VD1_FMT_W, w << 16 | ((w - 1) / 2 + 1));
	vc_wr(VD1_IF0_LUMA_Y0, (h - 1) << 16);
	vc_wr(VD1_IF0_LUMA_Y1, (h - 1) << 16);
	vc_wr(VD1_IF0_CHROMA_Y0, ((h - 1) >> 1) << 16);
	vc_wr(VD1_IF0_CHROMA_Y1, ((h - 1) >> 1) << 16);
	uint32_t hold = vc_rd(VPP_HOLD_LINES) & 0xff;
	if (!plane.afbc) vc_wr(VD1_IF0_GEN_REG, 0x38000053 | (hold > 0x1f ? 0x1f : hold) << 19);

	/* the scaler: VSR before VE, SAFA when the size changes */
	vc_wr(VPP_VE_H_V_SIZE, d->w << 16 | d->h);
	vc_wr(VPP_IN_H_V_SIZE, w << 16 | h);      /* (not there on the S7D: reads 0) */
	/* what the scaler takes and gives: left at 3840x2160, a 1920x1080 picture came out as its top half, twice as tall */
	vc_wr(VPP_VSR_TOP_IN_SIZE, h << 16 | w);
	vc_wr(VPP_VSR_TOP_OUT_SIZE, d->h << 16 | d->w);
	vc_wr(VPP_LINE_IN_LENGTH, w);
	vc_wr(VPP_PIC_IN_HEIGHT, h);
	vc_wr(VPP_PREBLEND_H_SIZE, h << 16 | w);
	vc_wr(VPP_VSR_TOP_C42C44_MODE, 0x44);
	vc_wr(VPP_VSR_TOP_MISC, 0x7);
	vc_m(VPP_SR_EN, 1, 0);
	vc_m(VPP_PI_EN_MODE, 1u << 4, 0);
	bool scaled = w != d->w || h != d->h, wide = w > wide_px, up = d->w >= w && d->h >= h;
	/* a picture this wide made smaller is more than the scaler reads in a 4K line time (safa_speed_up_handle): the
	 * pre-scaler halves it first (else it came out torn, or twice side by side) */
	bool halve = wide && (h > d->h || w > d->w);
	vc_m(SAFA_PPS_INTERP_EN_MODE, 3u << 25, scaled && up && !wide ? 1u << 25 : 0);
	if (scaled) {
		uint32_t hstep, hinit, vstep, vinit;
		if (!plane.tables) {
			load_tables();
		}
		scaler(halve ? (w + 1) / 2 : w, d->w, &hstep, &hinit);
		scaler(h, d->h, &vstep, &vinit);
		vc_wr(SAFA_PPS_PRE_SCALE, 0x00042400 | (halve ? 1u << 4 : 0));     /* [7:4] horizontal pre ratio, 1: 1/2 */
		vc_wr(SAFA_PPS_PRE_HSCALE_Y1, 0);
		vc_wr(SAFA_PPS_PRE_HSCALE_Y0, 0x100);
		vc_wr(SAFA_PPS_PRE_HSCALE_C1, 0);
		vc_wr(SAFA_PPS_PRE_HSCALE_C0, 0x100);
		vc_wr(SAFA_PPS_PRE_VSCALE, 0x100);
		vc_m(SAFA_PPS_HW_CTRL, 3u << 9, 2u << 9);
		vc_m(SAFA_PPS_INTERP_EN_MODE, 3u << 8 | 3u << 12 | 1u << 24, 1u << 8 | 1u << 12 | 1u << 24);
		vc_m(SAFA_PPS_YUV_SHARPEN_EN, 1u << 4, 1u << 4);
		vc_m(SAFA_PPS_DIR_EN_MODE, 1u << 24, 1u << 24);
		vc_m(SAFA_PPS_SR_ALP_INFO, 3u << 24 | 3u << 8, 1u << 24 | 1u << 8);
		vc_m(SAFA_PPS_PI_INFO, 3u << 20 | 3u << 8, 1u << 20 | 1u << 8);
		vc_wr(SAFA_PPS_HSC_STEP, hstep);
		vc_wr(SAFA_PPS_VSC_STEP, vstep);
		vc_wr(SAFA_PPS_VSC_INIT, vinit);
		vc_wr(SAFA_PPS_HSC_INIT, hinit);
	}
	vc_m(SAFA_PPS_SR_422_EN, 1, 1);
	vc_m(SAFA_PPS_SC_MISC, 1u << 4 | 1u << 8, halve ? 1u << 8 : 0);     /* [8] horizontal pre-scaler */
	vc_m(SAFA_PPS_HW_CTRL, 1u << 2, scaled ? 1u << 2 : 0);
	vc_m(SAFA_PPS_INTERP_EN_MODE, 1u << 26, 0);
	vc_m(SAFA_PPS_HW_CTRL, 1u << 19 | 1u << 4 | 1u << 8 | 1u << 1,
	    (wide ? 0 : 1u << 4) | (scaled ? 1u << 8 : 0) | (!scaled && wide ? 1u << 1 : 0));

	/* blend: the picture in its rectangle, under OSD1 */
	vc_wr(VPP_PREBLEND_VD1_H, w - 1);
	vc_wr(VPP_PREBLEND_VD1_V, 0x8ff);
	vc_wr(VPP_POSTBLEND_VD1_H, d->x << 16 | (d->x + d->w - 1));
	vc_wr(VPP_POSTBLEND_VD1_V, d->y << 16 | (d->y + d->h - 1));
}

static void
ports_on(void)
{
	plane.misc_saved = vc_rd(VPP_MISC);
	vc_wr(VD1_BLEND_SRC_CTRL, 0x00010111);
	vc_wr(VPP_MISC, (plane.misc_saved & 0x27fc0e05) | 0xca);
	plane.ports = true;
}

/*
 * The key. OSD1's own colour key works only for 16- and 24-bit pixels (osd_fb.c), and the window server's are 32-bit:
 * instead the flip path hands each frame to hakios_vplane_osd_frame(), which clears the alpha of the key's pixels in
 * the plane's rectangle, and OSD1 uses its pixels' alpha (VIU_OSD1_CTRL_STAT2 [14:6] 0) while the plane is on rather
 * than taking every pixel as opaque (0x1ff there, as U-Boot leaves it).
 */
static uint32_t stat2_saved;
static void
key_enable(bool on)
{
	if (on == plane.key) {
		return;
	}
	if (on) {
		stat2_saved = vc_rd(VIU_OSD1_CTRL_STAT2) & (0x1ffu << 6);
		vc_m(VIU_OSD1_CTRL_STAT2, 0x1ffu << 6, 0);
	} else {
		vc_m(VIU_OSD1_CTRL_STAT2, 0x1ffu << 6, stat2_saved ? stat2_saved : 0x1ffu << 6);
	}
	plane.key = on;
}

void
hakios_vplane_osd_frame(uint8_t *pixels, uint32_t row_bytes, uint32_t width, uint32_t height)
{
	struct rect k = plane.key_rect;         /* the worker's last look (a racy copy is harmless) */

	if (!plane.on || !plane.key || plane.test || !k.w || width != OSD_WIDTH || height != OSD_HEIGHT ||
	    row_bytes < OSD_WIDTH * 4 || k.x + k.w > OSD_WIDTH || k.y + k.h > OSD_HEIGHT) {
		return;
	}
	for (uint32_t y = k.y; y < k.y + k.h; y++) {
		uint32_t *row = (uint32_t *)(pixels + (size_t)y * row_bytes) + k.x;
		/* the GPU drew it: no stale line of ours may be read, and ours go back to memory before the scan */
		FlushPoC_DcacheRegion((vm_offset_t)row, k.w * 4);
		for (uint32_t x = 0; x < k.w; x++) {
			/* the grey and its coverage back from what was drawn over the key, premultiplied as OSD1 blends:
			 * coverage a = 1 - (R - G), colour a*c = G (the key itself: transparent black; transparent magenta's
			 * colour was added to the picture, white came out pink) */
			uint32_t p = row[x];
			int r = p >> 16 & 0xff, g = p >> 8 & 0xff, b = p & 0xff;
			if ((p >> 24) == 0xff && (r > b ? r - b : b - r) <= 6 && r > g) {
				int a = 255 - ((r + b) / 2 - g);
				if (a < g) a = g;
				row[x] = (uint32_t)a << 24 | (uint32_t)g << 16 | (uint32_t)g << 8 | (uint32_t)g;
			}
		}
		FlushPoC_DcacheRegion((vm_offset_t)row, k.w * 4);
	}
}

static void
layer_off(void)
{
	if (plane.ports) {
		vc_wr(VD1_BLEND_SRC_CTRL, 0x00010000);
		vc_wr(VPP_MISC, plane.misc_saved);
		plane.ports = false;
	}
	vc_wr(VD1_IF0_GEN_REG, 0);
	if (plane.afbc) {
		vc_wr(AFBC_ENABLE, 0);
		vc_m(VD1_AFBCD0_MISC_CTRL, 1u << 22 | 1u << 12 | 1u << 10, 0);
		plane.afbc = false;
	}
	plane.on = false;
	key_enable(false);
}

/* ---- the key in OSD1's frame ---- */

static bool
osd_frame(uint64_t *base, uint64_t *stride)
{
	*base = (uint64_t)vc_rd(VIU_OSD1_BLK1_CFG_W4) << 4;
	*stride = (uint64_t)(vc_rd(VIU_OSD1_BLK2_CFG_W4) & 0xfff) * 16;
	return *base && *stride >= OSD_WIDTH * 4 && *base + *stride * OSD_HEIGHT <= RAM_END;
}

/* the key's bounding box: a 16-pixel grid, then each edge to the pixel along the middle row and column */
static bool
find_key(struct rect *found)
{
	uint64_t base, stride;
	unsigned x0 = OSD_WIDTH, x1 = 0, y0 = OSD_HEIGHT, y1 = 0;

	if (!osd_frame(&base, &stride)) {
		return false;
	}
	/* the box around the key and what the player drew over it partly transparent (its bars: the picture goes under
	 * them), if there is some key itself */
	unsigned pure = 0;
	for (unsigned y = 8; y < OSD_HEIGHT; y += 16) {
		for (unsigned x = 8; x < OSD_WIDTH; x += 16) {
			uint32_t p = osd_pixel(base, stride, x, y);
			if (key_tinted(p)) {
				pure += key_pixel(p);
				x0 = x < x0 ? x : x0;
				x1 = x > x1 ? x : x1;
				y0 = y < y0 ? y : y0;
				y1 = y > y1 ? y : y1;
			}
		}
	}
	if (!pure || x1 < x0 || y1 < y0) {
		return false;
	}
	unsigned cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
	/* out to the edges */
	while (x0 > 0 && key_tinted(osd_pixel(base, stride, x0 - 1, cy))) x0--;
	while (x1 + 1 < OSD_WIDTH && key_tinted(osd_pixel(base, stride, x1 + 1, cy))) x1++;
	while (y0 > 0 && key_tinted(osd_pixel(base, stride, cx, y0 - 1))) y0--;
	while (y1 + 1 < OSD_HEIGHT && key_tinted(osd_pixel(base, stride, cx, y1 + 1))) y1++;
	/* near an edge of the screen: on it (opaque controls drawn over the picture's edge, Kodi's at the bottom, cut the box
	 * short and squashed the picture) */
	x0 = x0 < 32 ? 0 : x0;
	y0 = y0 < 32 ? 0 : y0;
	x1 = x1 + 32 >= OSD_WIDTH ? OSD_WIDTH - 1 : x1;
	y1 = y1 + 32 >= OSD_HEIGHT ? OSD_HEIGHT - 1 : y1;
	*found = (struct rect){ x0, y0, x1 - x0 + 1, y1 - y0 + 1 };
	return found->w >= 16 && found->h >= 16;
}

/* the key's rectangle on the output (OSD1 is scaled to all of it) */
static struct rect
output_rect(const struct rect *k)
{
	uint32_t size = vc_rd(VPP_OUT_H_V_SIZE), ow = size >> 16 & 0x1fff, oh = size & 0x1fff;
	if (ow < 640 || oh < 480) {
		ow = 3840;
		oh = 2160;
	}
	struct rect r = { k->x * ow / OSD_WIDTH, k->y * oh / OSD_HEIGHT, k->w * ow / OSD_WIDTH, k->h * oh / OSD_HEIGHT };
	r.w &= ~1u;
	r.h &= ~1u;
	return r;
}

/* ---- the worker: everything that touches the layer, at the vertical blanking ---- */

static void
plane_worker(__unused void *arg, __unused wait_result_t result)
{
	uint64_t scan_every;
	nanoseconds_to_absolutetime(250 * NSEC_PER_MSEC, &scan_every);
	for (;;) {
		lck_mtx_lock(&plane_lock);
		bool busy = plane.on || plane.pending || plane.stop;
		lck_mtx_unlock(&plane_lock);
		if (!busy) {
			IOSleep(20);
			continue;
		}
		/* the key's rectangle, now and then: the player may have moved or covered it (outside the lock: it reads
		 * a few thousand words of the OSD's frame) */
		struct rect key;
		bool look = !plane.direct && (mach_absolute_time() - plane.key_scanned > scan_every || !plane.key_rect.w);
		bool found = look && plane.key && find_key(&key);
		wait_blanking();
		lck_mtx_lock(&plane_lock);
		if (plane.stop) {
			printf("vplane: off: stopped\n");
			bzero(&plane.c, sizeof(plane.c));
			layer_off();
			plane.stop = plane.pending = plane.direct = false;
			plane.key_rect.w = 0;
			hakios_vdec_plane_showing(-1, -1);
			lck_mtx_unlock(&plane_lock);
			continue;
		}
		if (look && plane.key) {
			plane.key_scanned = mach_absolute_time();
			if (found) {
				/* a frame read while the window server draws over it gives a wrong rectangle now and then: one is
				 * taken only when two scans in a row agree (or there is none yet) */
				bool again = key.x == plane.key_seen.x && key.y == plane.key_seen.y && key.w == plane.key_seen.w &&
				    key.h == plane.key_seen.h;
				if (again || !plane.key_rect.w) {
					plane.key_rect = key;
				}
				plane.key_seen = key;
				plane.key_misses = 0;
			} else if (++plane.key_misses >= 8 && !plane.test) {
				/* two seconds without the key: the player has stopped drawing the picture */
				printf("vplane: off: no key for two seconds\n");
				layer_off();
				plane.pending = false;
				plane.key_rect.w = 0;
				hakios_vdec_plane_showing(-1, -1);
				lck_mtx_unlock(&plane_lock);
				continue;
			}
		}
		key_enable(true);
		/* a refresh went by (TWL_VPLANE_WAIT's callers wake) */
		os_atomic_inc(&vsync_seq, relaxed);
		thread_wakeup_prim((event_t)&vsync_seq, FALSE, THREAD_AWAKENED);
		hakios_vdec_plane_vsync();
		if (plane.on) {
			uint64_t now = mach_absolute_time(), ns;
			plane.c.vsyncs++;
			if (plane.pending && (plane.key_rect.w || plane.test || plane.direct)) plane.c.shown++;
			else plane.c.repeated++;
			if (!plane.c.since) plane.c.since = now;
			absolutetime_to_nanoseconds(now - plane.c.since, &ns);
			if (ns >= 5 * NSEC_PER_SEC) {
				printf("vplane: cadence: %u vsyncs in %llu ms: %u new, %u again, %u replaced unshown; gaps <12 %u, <20 %u, "
				    "<30 %u, <45 %u, more %u, max %u ms; pictures written in the blanking %u, during the picture %u "
				    "(as late as line %u)\n", plane.c.vsyncs, ns / NSEC_PER_MSEC, plane.c.shown,
				    plane.c.repeated, plane.c.overwritten, plane.c.gaps[0], plane.c.gaps[1], plane.c.gaps[2],
				    plane.c.gaps[3], plane.c.gaps[4], plane.c.gap_max, plane.c.writes_blank, plane.c.writes_active,
				    plane.c.write_line_max);
				uint64_t last_in = plane.c.last_in;
				bzero(&plane.c, sizeof(plane.c));
				plane.c.last_in = last_in;
				plane.c.since = now;
			}
		}
		if (plane.pending && (plane.key_rect.w || plane.test || plane.direct)) {
			struct rect d = plane.test ? plane.test_dst : output_rect(plane.direct ? &plane.direct_rect : &plane.key_rect);
			bool same = plane.on && !plane.reprogram && plane.shown.width == plane.next.width && plane.shown.height == plane.next.height &&
			    plane.shown.picture.stride == plane.next.picture.stride &&
			    plane.afbc == (use_compressed && plane.next.picture.compressed) && d.x == plane.dst.x && d.y == plane.dst.y &&
			    d.w == plane.dst.w && d.h == plane.dst.h;
			if (same) {
				program_picture(&plane.next);
				/* (diagnosis: where the scan was when the new picture's registers were written) */
				uint32_t line = vc_rd(ENCP_INFO_READ) >> 16 & 0x1fff, begin = vc_rd(ENCP_DE_V_BEGIN), end = vc_rd(ENCP_DE_V_END);
				if (line >= begin && line < end) {
					plane.c.writes_active++;
					if (line - begin > plane.c.write_line_max) plane.c.write_line_max = line - begin;
				} else {
					plane.c.writes_blank++;
				}
			} else {
				/* (diagnosis: why the layer is set up again; a set-up shows black for a frame or more) */
				static uint32_t setups;
				if (setups++ < 200) {
					printf("vplane: set-up %u: %s%s%s%s%s%s %ux%u -> %d,%d %dx%d (was %d,%d %dx%d), direct %d\n", setups,
					    !plane.on ? "off " : "", plane.reprogram ? "asked " : "",
					    plane.on && (plane.shown.width != plane.next.width || plane.shown.height != plane.next.height) ? "size " : "",
					    plane.on && plane.shown.picture.stride != plane.next.picture.stride ? "stride " : "",
					    plane.on && plane.afbc != (use_compressed && plane.next.picture.compressed) ? "afbc " : "",
					    plane.on && (d.x != plane.dst.x || d.y != plane.dst.y || d.w != plane.dst.w || d.h != plane.dst.h) ? "rect " : "",
					    plane.next.width, plane.next.height, d.x, d.y, d.w, d.h, plane.dst.x, plane.dst.y, plane.dst.w, plane.dst.h,
					    plane.direct);
				}
				program_layer(&plane.next, &d);
				plane.dst = d;
				plane.reprogram = false;
			}
			plane.shown = plane.next;
			plane.pending = false;
			if (plane.on && !plane.ports) {
				ports_on();
			}
			plane.on = true;
			plane.frames++;
			hakios_vdec_plane_showing(plane.test ? -1 : (int)plane.shown.slot, -1);
		} else if (plane.on && !plane.ports) {
			ports_on();             /* a vsync after the layer was first set up, as the vendor does */
		}
		lck_mtx_unlock(&plane_lock);
	}
}

static void
plane_queue(const struct plane_picture *p, bool test, const struct rect *dst)
{
	if (!worker_started) {
		thread_t thread;
		if (kernel_thread_start(plane_worker, NULL, &thread) != KERN_SUCCESS) {
			return;
		}
		thread_deallocate(thread);
		worker_started = true;
	}
	if (!test) {
		uint64_t now = mach_absolute_time(), ns = 0;
		if (plane.pending) plane.c.overwritten++;
		if (plane.c.last_in) {
			absolutetime_to_nanoseconds(now - plane.c.last_in, &ns);
			uint32_t ms = (uint32_t)(ns / NSEC_PER_MSEC);
			plane.c.gaps[ms < 12 ? 0 : ms < 20 ? 1 : ms < 30 ? 2 : ms < 45 ? 3 : 4]++;
			if (ms > plane.c.gap_max && ms < 1000) plane.c.gap_max = ms;
		}
		plane.c.last_in = now;
	}
	plane.next = *p;
	plane.pending = true;
	plane.stop = false;
	plane.test = test;
	if (test) {
		plane.test_dst = *dst;
		plane.reprogram = true;
	}
	hakios_vdec_plane_showing(plane.on && !plane.test ? (int)plane.shown.slot : -1, test ? -1 : (int)p->slot);
}

/*
 * A window server's video layer. The copies go round three pieces of the copy memory: the one written is neither the
 * picture shown nor the one waiting for the vsync.
 */
static vm_offset_t copy_va;
static uint64_t copy_pa;
static uint32_t copy_size;
static unsigned copy_turn;
static uint32_t surface_reported;

extern void hakios_hdmitx_video_pq(bool pq);

/* (diagnosis: why a window server's video picture was not shown; the first few of each reason) */
static void
refused(unsigned reason, const char *what, uint32_t a, uint32_t b)
{
	static unsigned said[8];
	if (reason < 8 && said[reason]++ < 6) printf("vplane: a video layer's picture not shown: %s (%u, %u)\n", what, a, b);
}

/* VD1's HDR core for what the picture is (an SDR picture on an HDR10 output is made PQ; hakios.hdmitx.c), on a change */
static void
picture_kind(bool pq)
{
	static int last = -1;
	if (last != (int)pq) {
		last = (int)pq;
		hakios_hdmitx_video_pq(pq);
	}
}

void
hakios_vplane_surface(const struct hakios_vplane_surface *v)
{
	struct plane_picture p = { .slot = UINT32_MAX };

	if (v == NULL) {
		if (plane.direct) {
			hakios_vplane_stop();
		}
		return;
	}
	if (!vpu_map() || v->w < 16 || v->h < 16 || v->x + v->w > OSD_WIDTH || v->y + v->h > OSD_HEIGHT || v->width < 16 || v->height < 16 ||
	    v->width > 4096 || v->height > 2304) {
		refused(0, "its place or size", v->x + v->w, v->width);
		return;
	}
	static uint64_t token_seen;
	uint64_t now = mach_absolute_time(), half_second;
	nanoseconds_to_absolutetime(500 * NSEC_PER_MSEC, &half_second);
	if (v->token != NULL) {
		const struct twl_vplane_token *t = v->token;
		int e = t->magic != TWL_VPLANE_MAGIC ? EINVAL : hakios_vdec_plane_picture(t->slot, t->generation, &p.picture);
		if (e != 0) {
			refused(1, "its token", t->slot, (uint32_t)e);
			return;
		}
		picture_kind((t->flags & TWL_VPLANE_PQ) != 0);
		static uint64_t compressed_seen;
		if (use_compressed && p.picture.compressed) {
			compressed_seen = now;
		} else if (use_compressed && compressed_seen && now - compressed_seen < half_second) {
			/* a picture whose compressed body the decoder had to give back (its page pool short): only its smaller
			 * copy is left. Shown, it was a set-up to the copy and one back a frame later, each a black frame */
			refused(2, "no compressed body, shortly after one with", t->slot, 0);
			return;
		}
		token_seen = now;
		if (use_compressed && p.picture.compressed) {
			/* the compressed picture at the decoder's size (the pixel buffer may hold a smaller copy) */
			p.width = t->width < p.picture.coded_width ? t->width : p.picture.coded_width;
			p.height = t->height < p.picture.coded_height ? t->height : p.picture.coded_height;
		} else {
			p.width = t->width < v->width ? t->width : v->width;
			p.height = t->height < v->height ? t->height : v->height;
		}
		p.slot = t->slot;
	} else if (token_seen && now - token_seen < half_second) {
		/* a picture without the decoder's token among the decoder's own (the player's layer shows one of its buffers
		 * again now and then): the one on the plane stays. Taken, it was a set-up to the smaller copy and one back to
		 * the compressed picture a frame later, each showing black: YouTube's video flickered black */
		refused(3, "no token, shortly after a decoder's picture", v->width, v->height);
		return;
	} else {
		uint32_t w = v->width & ~1u, h = v->height & ~1u, stride = (w + 63) & ~63u;
		uint32_t piece = ((stride * h * 3 / 2) + 0xffffu) & ~0xffffu;
		if (copy_va == 0 && hakios_vdec_copy_memory(&copy_va, &copy_pa, &copy_size) != 0) {
			copy_va = 0;
			refused(4, "no copy memory", 0, 0);
			return;
		}
		picture_kind(false);
		if (piece * 3 > copy_size || v->luma == NULL || v->chroma == NULL || v->luma_stride < w || v->chroma_stride < w) {
			if (surface_reported++ < 4) {
				printf("vplane: a video layer of %ux%u is more than the copy memory holds (%u bytes)\n", w, h, copy_size);
			}
			return;
		}
		uint32_t off = piece * (copy_turn++ % 3);
		uint8_t *luma = (uint8_t *)(copy_va + off), *chroma = luma + (size_t)stride * h;
		for (uint32_t y = 0; y < h; y++) {
			memcpy(luma + (size_t)y * stride, v->luma + (size_t)y * v->luma_stride, w);
		}
		for (uint32_t y = 0; y < h / 2; y++) {
			memcpy(chroma + (size_t)y * stride, v->chroma + (size_t)y * v->chroma_stride, w);
		}
		/* the display reads memory, not the cache */
		FlushPoC_DcacheRegion((vm_offset_t)luma, (size_t)stride * h * 3 / 2);
		p.picture = (struct hakios_vdec_picture){ copy_pa + off, copy_pa + off + (uint64_t)stride * h, stride, h, w, 0, 8 };
		p.width = w;
		p.height = h;
	}
	struct rect r = { v->x & ~1u, v->y & ~1u, v->w & ~1u, v->h & ~1u };
	lck_mtx_lock(&plane_lock);
	if (!plane.direct) {
		printf("vplane: a window server's video layer: %ux%u%s in %u,%u %ux%u\n", p.width, p.height,
		    v->token ? " (the decoder's)" : " (copied)", r.x, r.y, r.w, r.h);
	}
	if (!plane.direct || r.x != plane.direct_rect.x || r.y != plane.direct_rect.y || r.w != plane.direct_rect.w ||
	    r.h != plane.direct_rect.h) {
		plane.reprogram = true;
	}
	plane.direct = true;
	plane.direct_rect = r;
	plane.key_rect.w = 0;           /* (no key: hakios_vplane_osd_frame leaves the frame alone) */
	plane_queue(&p, false, NULL);
	lck_mtx_unlock(&plane_lock);
}

void
hakios_vplane_stop(void)
{
	lck_mtx_lock(&plane_lock);
	if (plane.on || plane.pending) {
		plane.stop = true;
		plane.pending = false;
	}
	lck_mtx_unlock(&plane_lock);
}

/*
 * Bring-up commands: s W H X Y DW DH (the test picture's top left W x H shown in that output rectangle, no key
 * needed), h (off), o [ALPHA] (OSD1's global alpha, 0..1ff, 0x100 opaque: the default), k (the key rectangle found now, and the plane's state)
 */
static void
plane_command(char op, const uint64_t *v, unsigned n)
{
	if (op == 's') {
		if (n != 6 || !buffer_get() || v[0] < 16 || v[1] < 16 || v[0] > 4096 || v[1] > 2304 || !v[4] || !v[5]) {
			out("s: W H X Y DW DH (after f)\n");
			return;
		}
		uint32_t stride = ((uint32_t)v[0] + 63) & ~63u;
		struct plane_picture p = { { buffer_pa, buffer_pa + (uint64_t)stride * v[1], stride, (uint32_t)v[1], (uint32_t)v[0] },
			(uint32_t)v[0], (uint32_t)v[1], 0 };
		struct rect d = { (uint32_t)v[2], (uint32_t)v[3], (uint32_t)v[4], (uint32_t)v[5] };
		lck_mtx_lock(&plane_lock);
		plane_queue(&p, true, &d);
		lck_mtx_unlock(&plane_lock);
		out("s: %ux%u to %u,%u %ux%u queued\n", p.width, p.height, d.x, d.y, d.w, d.h);
	} else if (op == 'h') {
		hakios_vplane_stop();
		lck_mtx_lock(&plane_lock);
		plane.test = false;
		lck_mtx_unlock(&plane_lock);
		out("h: off\n");
	} else if (op == 'o') {
		/* 9 bits, 0x100 opaque (what U-Boot and the vendor leave; 0xff is about half) */
		vc_m(VIU_OSD1_CTRL_STAT, 0x1ff << 12, (uint32_t)(n ? v[0] : 0x100) << 12);
		out("o: OSD1 global alpha %llx\n", n ? v[0] : 0x100);
	} else if (op == 'c') {
		/* compressed pictures from the decoder (1), or its linear 8-bit copies (0); from the next picture on */
		use_compressed = n ? v[0] != 0 : !use_compressed;
		plane.reprogram = true;
		out("c: %s pictures\n", use_compressed ? "compressed" : "linear");
	} else if (op == 'k') {
		struct rect k = { 0, 0, 0, 0 };
		bool found = find_key(&k);
		out("k: key %s %u,%u %ux%u; plane on %d ports %d key %d afbc %d frames %u, rect %u,%u %ux%u, key rect %u,%u %ux%u\n",
		    found ? "at" : "not found", k.x, k.y, k.w, k.h, plane.on, plane.ports, plane.key, plane.afbc, plane.frames,
		    plane.dst.x, plane.dst.y, plane.dst.w, plane.dst.h, plane.key_rect.x, plane.key_rect.y, plane.key_rect.w,
		    plane.key_rect.h);
	}
}

/* ---- /dev/twl-vplane ---- */

/*
 * TWL_VPLANE_WAIT: while a picture waits for the next vertical blanking, until it has been taken (100 ms at most); at
 * once otherwise. A player that draws the picture's key itself (Kodi, tvos/src/twlcvgl.c) asks before each SHOW: it
 * presented some 36 frames a second into a 24 Hz output, and which of them each refresh took wandered (24p judder);
 * waiting a whole refresh after every present instead halved its rate (it times its frames itself as well).
 */
static int
plane_wait(void)
{
	if (!plane.on || !plane.pending) {
		return 0;
	}
	uint32_t seq = os_atomic_load(&vsync_seq, acquire);
	uint64_t deadline;
	clock_interval_to_deadline(100, NSEC_PER_MSEC, &deadline);
	assert_wait_deadline((event_t)&vsync_seq, THREAD_INTERRUPTIBLE, deadline);
	if (os_atomic_load(&vsync_seq, acquire) != seq) {
		clear_wait(current_thread(), THREAD_AWAKENED);
		return 0;
	}
	return thread_block(THREAD_CONTINUE_NULL) == THREAD_INTERRUPTED ? EINTR : 0;
}

static int
vplane_ioctl(__unused dev_t dev, u_long command, caddr_t data, __unused int flag, __unused struct proc *p)
{
	if (command == TWL_VPLANE_HIDE) {
		hakios_vplane_stop();
		return 0;
	}
	if (command == TWL_VPLANE_WAIT) {
		return plane_wait();
	}
	if (command != TWL_VPLANE_SHOW) {
		return ENOTTY;
	}
	const struct twl_vplane_token *token = (const struct twl_vplane_token *)data;
	struct plane_picture picture = { .width = token->width, .height = token->height, .slot = token->slot };
	if (token->magic != TWL_VPLANE_MAGIC || token->width < 16 || token->height < 16 || token->width > 4096 ||
	    token->height > 2304) {
		return EINVAL;
	}
	int error = hakios_vdec_plane_picture(token->slot, token->generation, &picture.picture);
	if (error) {
		return error;
	}
	picture_kind((token->flags & TWL_VPLANE_PQ) != 0);
	if (use_compressed && picture.picture.compressed) {
		if (picture.width > picture.picture.coded_width || picture.height > picture.picture.coded_height) {
			return EINVAL;
		}
	} else if (picture.width > picture.picture.width || picture.height > picture.picture.rows) {
		return EINVAL;
	}
	if (!vpu_map()) {
		return ENXIO;
	}
	lck_mtx_lock(&plane_lock);
	if (plane.direct) {
		plane.direct = false;   /* a player's key from now on */
		plane.reprogram = true;
	}
	plane_queue(&picture, false, NULL);
	lck_mtx_unlock(&plane_lock);
	return 0;
}

static int
vplane_open(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	return 0;
}

static const struct cdevsw vplane_cdevsw = {
	.d_open = vplane_open, .d_close = (d_close_t *)vplane_open,
	.d_read = eno_rdwrt, .d_write = eno_rdwrt, .d_ioctl = vplane_ioctl,
	.d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

/* the node, once the decoder's transport is up (hakios.vdec.c) */
void
hakios_vplane_publish(void)
{
	static bool published;
	if (published) {
		return;
	}
	int major = cdevsw_add(-1, &vplane_cdevsw);
	if (major >= 0 && devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "twl-vplane")) {
		published = true;
		printf("vplane: /dev/twl-vplane: VD1 under OSD1, colour key %06x\n", KEY_RGB);
	} else if (major >= 0) {
		cdevsw_remove(major, &vplane_cdevsw);
	}
}

static int
vpu_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char input[512];
	int error;

	lck_mtx_lock(&vpu_lock);
	if (!vpu_map()) {
		lck_mtx_unlock(&vpu_lock);
		return ENXIO;
	}
	error = SYSCTL_OUT(req, result, result_len + 1);
	if (!error && req->newptr) {
		size_t len = req->newlen;
		if (len >= sizeof(input)) {
			error = E2BIG;
		} else {
			error = SYSCTL_IN(req, input, len);
		}
		if (!error) {
			input[len] = 0;
			result_len = 0;
			result[0] = 0;
			for (char *line = input, *next; line; line = next) {
				next = strchr(line, ';');
				if (next) {
					*next++ = 0;
				}
				command(line);
			}
		}
	}
	lck_mtx_unlock(&vpu_lock);
	return error;
}

SYSCTL_PROC(_kern, OID_AUTO, hakios_vpu, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, vpu_sysctl, "A", "the Amlogic VPU's registers and the video plane (bring-up commands: hakios.vplane.c)");

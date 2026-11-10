/*
 * The Amlogic box's video decoders (S905X5M "S7D": the DOS block, VDEC_1 for H.264 and HEVC for H.265/VP9/AV1),
 * vdec=1: power, clocks, reset and register tests; vdec=2: also load H.264/HEVC/VP9 microcode (optee=2 required).
 * vdec=3: feed fw/vdec.h264 and stop at the first parameter request; vdec=4: a single 64x64 I-picture test.
 * vdec_delay=N waits N seconds first, in a thread of its own.
 *
 * DOS registers are 32 bit, at 0xfe320000 + reg * 4 (Amlogic's dos_registers_v1.h numbers). Each core has a power
 * domain, switched by the SiP call 0x82000093 (domain, 1: on) and read by 0x82000095 (domain: 1 if off); VDEC is
 * domain 2, HEVC 1 (s7d-pd.h). Their clocks are CLKCTRL VDEC_CLK_CTRL (0x140, VDEC) and VDEC2_CLK_CTRL (0x144,
 * HEVC): bits 9-11 the source (0: fclk_div2p5, 800 MHz; 1: fclk_div3, 667), 0-6 the divider, 8 the gate; bit 15 of VDEC3/VDEC4_CLK_CTRL
 * picks between those and a second copy (0: the first). The DOS bus clock is SYS_CLK_EN0_REG0 bit 1. The order is
 * Amlogic's media_modules (vdec_power_ctrl.c: pm_vdec_clock_on, the power domain, dos_local_config).
 *
 * The cores' microcode is encrypted and loaded by the secure world (hakios.optee.c). These are diagnostic
 * stages, not a VideoToolbox decoder. No hardware capability is advertised to applications.
 */
#include <kern/clock.h>
#include <mach/mach_time.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <sys/hakios.hdmitx.h>
#include <sys/hakios.optee.h>
#include <vm/vm_kern.h>
#include <arm/caches_internal.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <sys/conf.h>
#include <sys/errno.h>
#include <sys/proc.h>
#include <sys/hakios.vdec.h>
#include <sys/hakios.vplane.h>
#include <sys/hakios.hevc.h>
#include <sys/hakios.vp9.h>
#include <miscfs/devfs/devfs.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void IOSleep(unsigned int milliseconds);
extern void IODelay(unsigned int microseconds);
extern kern_return_t kernel_thread_start(thread_continue_t continuation, void *parameter, thread_t *new_thread);
extern void thread_deallocate(thread_t thread);
extern uint64_t kvtophys(vm_offset_t va);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

#define DOS_BASE        0xfe320000UL
#define DOS_SIZE        0x10000UL
#define CLK_BASE        0xfe000000UL
#define RESET_BASE      0xfe002000UL
#define RESET1_LEVEL    0x44
#define RESET5_LEVEL    0x54
#define RESET_DOS_CAPB3  (1u << 5)
#define RESET_DOS       (1u << 6)
#define RESET_BRG_VDEC  (1u << 0)
#define RESET_BRG_HEVCF (1u << 12)

#define SIP_PWRC_SET    0x82000093UL
#define SIP_PWRC_GET    0x82000095UL
#define PD_HEVC         1
#define PD_VDEC         2

/* CLKCTRL */
#define SYS_CLK_EN0_REG0 0x044
#define SYS_CLK_DOS     (1u << 1)
#define VDEC_CLK_CTRL   0x140
#define VDEC2_CLK_CTRL  0x144
#define VDEC3_CLK_CTRL  0x148
#define VDEC4_CLK_CTRL  0x14c
#define CLK_SRC_FCLK_DIV2P5 0u    /* 800 MHz: the vendor's S7D maximum for every core (decoder_cpu_ver_info.c) */
#define CLK_SRC_FCLK_DIV3 1u      /* 667 MHz */
/* boot-arg vdec_667=1: the cores at 667 MHz (as before); a 4K60 VP9 picture took 11.5-17 ms of the 16.7 there */
static uint32_t clk_src = CLK_SRC_FCLK_DIV2P5;

/* DOS register numbers */
#define MPSR            0x0301
#define CPSR            0x0321
#define IMEM_DMA_CTRL   0x0340
#define LMEM_DMA_CTRL   0x0350
#define AV_SCRATCH_0    0x09c0
#define HEVC_ASSIST_SCRATCH_0 0x30c0
#define HEVC_PARSER_VERSION 0x3100
#define HEVC_MPSR       0x3301
#define HEVC_CPSR       0x3321
#define DOS_SW_RESET0   0x3f00
#define DOS_GCLK_EN0    0x3f01
#define DOS_MEM_PD_VDEC 0x3f30
#define DOS_MEM_PD_HEVC 0x3f33
#define DOS_SW_RESET3   0x3f34
#define DOS_GCLK_EN3    0x3f35

/* H.264 multi firmware interface (Amlogic vmh264, frame input). */
#define VDEC_ASSIST_MMC_CTRL1 0x0002
#define ASSIST_MBOX1_CLR 0x0075
#define ASSIST_MBOX1_MASK 0x0076
#define MPC_E           0x0308
#define PSCALE_CTRL     0x0911
#define MDEC_PIC_DC_MUX_CTRL 0x098d
#define MDEC_PIC_DC_CTRL 0x098e
#define MDEC_PIC_DC_THRESH 0x09b8
#define MDEC_EXTIF_CFG1 0x09e5
#define MDEC_EXTIF_CFG2 0x09e6
#define POWER_CTL_VLD   0x0c08
#define VIFF_BIT_CNT    0x0c1a
#define H264_DECODE_INFO 0x0c29
#define FIFO_START      0x0c40
#define FIFO_CURR       0x0c41
#define FIFO_END        0x0c42
#define FIFO_CONTROL    0x0c44
#define FIFO_WP         0x0c45
#define FIFO_RP         0x0c46
#define FIFO_LEVEL      0x0c47
#define FIFO_BUF_CNTL   0x0c48
#define H264_WORK       (AV_SCRATCH_0 + 8)
#define H264_STATUS     (AV_SCRATCH_0 + 19)
#define H264_LMEM       (AV_SCRATCH_0 + 21)
#define H264_CONFIG_REQUEST 0x11
#define H264_SLICE_HEAD_DONE 1
#define H264_SEI_READY   0x53
#define H264_SEI_DONE    0x54
#define H264_SEARCH_HEAD 0xf0
/* The pool (vdec_pool_mb=, 256 MiB by default): firmware workspace from 64 KiB, input, LMEM, then the pictures and
 * after them their co-located motion vectors (one more, for pictures no other refers to). Slots are sized for the
 * stream (h264_test_configure): 24 of them up to 1920x1088, about ten at 3840x2160. VP9 at 3840x2160 needs all 24
 * (8 references and YouTube's 6-16 pictures on their way to the plane, ~6.5 MiB of compressed body each): with 176
 * MiB it had 18 and most pictures were decoded over before the plane showed them. */
#define TEST_INPUT_OFF  (3u * 1024 * 1024)
#define TEST_INPUT_SIZE (4u * 1024 * 1024)
#define TEST_LMEM_OFF   (7u * 1024 * 1024)
#define TEST_PICTURE_OFF (8u * 1024 * 1024)
#define POOL_MB_DEFAULT 256u
#define COPY_MB_DEFAULT 12u     /* three 1920 x 1088 NV12 pictures */
static uint32_t copy_size;
#define SLOTS_MIN       8u
static uint32_t test_pool_size;
static uint32_t picture_bytes = 0x300000u, mv_bytes = 0xc0000u, slot_count = TWL_H264_SLOTS;
#define PICTURE_SLOT    picture_bytes
#define MV_SLOT         mv_bytes
#define TEST_MV_OFF     (TEST_PICTURE_OFF + slot_count * PICTURE_SLOT)
#define TEST_POOL_SIZE  test_pool_size
#define MDEC_CAV_DATAL  0x09ea
#define MDEC_CAV_DATAH  0x09eb
#define MDEC_CAV_ADDR   0x09ec
#define MDEC_CAV_CFG0   0x09ed
#define CANVAS_BLK32    0x0005
#define ANC0_CANVAS    0x0990
#define REC_CANVAS     0x09b2
#define CURR_CANVAS    0x09b3
#define VCOP_CONTROL   0x0e00
#define VLD_CONTROL    0x0c18
#define DBKR_CANVAS    0x09b0
#define DBKW_CANVAS    0x09b1
#define BUFFER_INFO_INDEX 0x0c24
#define BUFFER_INFO_DATA  0x0c22
#define POC_RESET       0x0c30
#define POC_DATA        0x0c32
#define CO_MB_WR_ADDR   0x0c38
#define CO_MB_RD_ADDR   0x0c39
#define IQIDCT_CONTROL 0x0e0e
#define DCAC_BYTE64    0x0e1d

static vm_offset_t dos, clk, reset;
static unsigned int test_level;
static vm_offset_t test_pool;
static uint64_t test_pool_pa;
static bool vdec_powered, hevc_powered;
static unsigned dump_rpm, ucode_debug, debug_access;
static unsigned picture_slot;    /* the slot of the picture whose slices are being decoded */
/*
 * For the video plane (hakios.vplane.c): each slot's generation (a token names a picture by slot and generation, so
 * one decoded over since is refused), and holds. While the plane shows this decoder's pictures, the slot on the
 * screen, the one waiting for the next vsync and the last HOLD_PICTURES pictures handed out (a player queues a few
 * before it shows them) are decoded into only when nothing else is free. The plane's own fields are atomics: it
 * sets them under its lock, never under this driver's.
 */
#define HOLD_PICTURES   32u     /* at most; YouTube's pictures reach the plane 100-270 ms (6-16 pictures) after they leave */
static uint32_t plane_passed;   /* the hand-out number of the newest picture the plane took (diagnosis) */
/*
 * Each slot's picture until the plane has taken it: held so long. (Not "everything handed out before the newest one
 * taken": HEVC hands pictures out in decode order and they are shown in display order, so a B picture taken released
 * the earlier-decoded pictures still to be shown, and Kodi found every other picture decoded over: 12 of 24 a second.)
 */
#define LAT_SLOTS 32
static bool out_unshown[LAT_SLOTS];
static uint32_t slot_generation[TWL_H264_SLOTS], slot_handed[TWL_H264_SLOTS], handed_count, generation_base;
static _Atomic int plane_slot = -1, plane_pending = -1;
static _Atomic uint32_t plane_claimed;  /* slots handed to the plane that it has not yet marked pending */
/*
 * The picture the plane showed before the one now latched: the VPU's registers take a new picture at the next vsync, so
 * the old one is still scanned for a frame after it was replaced. Released to the decoder only after that (the vendor's
 * video.c puts a frame back a vsync after the next one shows): decoded into at once, its top rows -- decoded first --
 * showed as coloured lines over the picture for a frame, most at scene changes.
 */
static _Atomic int plane_prev = -1;
static uint32_t plane_prev_vsyncs;
static uint32_t lat_copies;             /* diagnosis: pictures the plane got as their copy (pages taken back) */
static int plane_only = 1;              /* 1: no copies into the player's pixel buffers while the plane shows the pictures (2 ms of a 4K picture's 16) */
SYSCTL_INT(_kern, OID_AUTO, hakios_vdec_plane_only, CTLFLAG_RW | CTLFLAG_LOCKED, &plane_only, 0, "");
static _Atomic uint64_t plane_seen;     /* mach_absolute_time of the plane's last picture from this decoder */
static _Atomic uint64_t plane_wanted;   /* and of the last time a player asked for one, shown or not */
/* The stream's coded picture: macroblocks, and the canvas stride (64-byte aligned) and rows of its luma. */
static unsigned stream_mb_width = 4, stream_mb_total = 16, stream_stride = 64, stream_rows = 64;
#define STREAM_FRAME_BYTES (stream_stride * stream_rows * 3 / 2)
#define vdec_trace(...) do { if (dump_rpm) vdec_log(__VA_ARGS__); } while (0)
static unsigned test_picture_count;
static uint32_t test_sequence_info;
static uint32_t test_sequence_control, test_bitstream_restriction;
static uint32_t test_saved_reg_f;
static bool stateful_decode, header_pending;
enum { CODEC_H264, CODEC_HEVC, CODEC_VP9 };
static int core_codec;     /* the stream's: HEVC (hakios.hevc.c) and VP9 (hakios.vp9.c) have their own slots */
static uint32_t saved_iqidct_control, saved_vcop_control, saved_vld_control;

/* Preserve probe results in the loader mailbox even if Wi-Fi drops or /dev/console is redirected. */
extern void loader_log_line(const char *line);
extern void hakios_wdt_init(void);
extern void hakios_amlcpu_init(void);
extern void hakios_wdt_stop(void);
static void
vdec_log(const char *format, ...)
{
	char line[256];
	va_list args;

	va_start(args, format);
	vsnprintf(line, sizeof(line), format, args);
	va_end(args);
	printf("%s", line);
	loader_log_line(line);
}

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

static uint32_t clk_rd(uint32_t off) { return *(volatile uint32_t *)(clk + off); }
static void clk_wr(uint32_t off, uint32_t v) { *(volatile uint32_t *)(clk + off) = v; }
static uint32_t dos_rd(uint32_t reg) { return *(volatile uint32_t *)(dos + reg * 4); }
static void dos_wr(uint32_t reg, uint32_t v) { *(volatile uint32_t *)(dos + reg * 4) = v; }

static bool
h264_firmware_debug(void)
{
	uint32_t tag = dos_rd(AV_SCRATCH_0 + 22);
	if (tag <= 1) return false; /* Bit zero is the initial CSD-compatibility flag. */
	vdec_log("vdec: firmware debug tag %#x value %#x PC %#x\n", tag,
	    dos_rd(AV_SCRATCH_0 + 23), dos_rd(MPC_E));
	if (tag & 0x20000) return true; /* Firmware explicitly requested a fatal pause. */
	dos_wr(ASSIST_MBOX1_CLR, 1);
	dos_wr(AV_SCRATCH_0 + 22, 0);
	return false;
}

static void
h264_test_canvas(unsigned index, uint32_t pa, unsigned stride, unsigned height)
{
	/* S7D's DOS-local canvas, linear, little-endian byte output (vdec_canvas_utils.c's packing). */
	dos_wr(MDEC_CAV_CFG0, 0x1ffu << 17);
	dos_wr(MDEC_CAV_DATAL, (pa >> 3) | (((stride >> 3) & 7u) << 29));
	dos_wr(MDEC_CAV_DATAH, (((stride >> 3) >> 3) & 0x1ffu) | ((height & 0x1fffu) << 9) | (7u << 26));
	dos_wr(MDEC_CAV_ADDR, index);
	dos_wr(CANVAS_BLK32, (1u << 11) | (1u << 8) | index);
}

static void
h264_test_canvases(unsigned count)
{
	for (unsigned i = 0; i < count; i++) {
		uint32_t pa = (uint32_t)test_pool_pa + TEST_PICTURE_OFF + i * (stateful_decode ? PICTURE_SLOT : 0x20000);

		h264_test_canvas(i * 2, pa, stream_stride, stream_rows);
		h264_test_canvas(i * 2 + 1, pa + (stateful_decode ? stream_stride * stream_rows : 0x10000), stream_stride, stream_rows / 2);
		dos_wr(ANC0_CANVAS + i, i * 2 | ((i * 2 + 1) << 8) | ((i * 2 + 1) << 16));
	}
}

static bool
h264_test_configure(void)
{
	uint32_t seq = dos_rd(AV_SCRATCH_0 + 1), flags = dos_rd(AV_SCRATCH_0 + 2);
	test_sequence_info = seq;
	test_sequence_control = dos_rd(AV_SCRATCH_0 + 11);
	FlushPoC_DcacheRegion(test_pool + TEST_LMEM_OFF, 4096);
	const uint16_t *rpm = (const uint16_t *)(test_pool + TEST_LMEM_OFF);
	test_bitstream_restriction = (rpm[(0x6c & ~3) + 3 - (0x6c & 3)] >> 3) & 1;

	vdec_log("vdec: H.264 SPS: info %#x flags %#x level/ref %#x\n", seq, flags, dos_rd(AV_SCRATCH_0 + 11));
	unsigned mb_width = seq & 0xff, mb_total = (seq >> 8) & 0xffff;
	if (!mb_width && mb_total) mb_width = 256;  /* 4096 wide: the field is 8 bits (vmh264 does the same) */
	unsigned mb_height = mb_width ? mb_total / mb_width : 0, stride = (mb_width * 16 + 63) & ~63u;
	/* a slot: the picture (luma, then chroma) and its motion vectors, each 64 KiB aligned */
	uint32_t want_picture = ((stride * mb_height * 16 * 3 / 2) + 0xffffu) & ~0xffffu;
	uint32_t want_mv = (mb_total * 96 + 0xffffu) & ~0xffffu;
	uint32_t room = test_pool_size > TEST_PICTURE_OFF ? test_pool_size - TEST_PICTURE_OFF : 0;
	uint32_t fits = want_picture + want_mv ? (room - want_mv) / (want_picture + want_mv) : 0;
	if (room < want_mv) fits = 0;
	if (fits > TWL_H264_SLOTS) fits = TWL_H264_SLOTS;
	/* Progressive 4:2:0 only (Baseline's implicit 4:2:0 may be reported as zero); the single-picture
	 * modes are 64x64 only, the reference-picture transport anything that fits a slot. */
	if (!mb_width || !mb_height || mb_height * mb_width != mb_total || !(flags & (1u << 15)) ||
	    ((flags >> 13) & 3) > 1 || (!stateful_decode && (mb_width != 4 || mb_total != 16)) ||
	    mb_width * 16 > 4096 || mb_height * 16 > 4096 || (stateful_decode && fits < SLOTS_MIN)) {
		vdec_log("vdec: stream of %ux%u macroblocks (flags %#x) does not fit the decoder's buffers (%u slots)\n",
		    mb_width, mb_height, flags, fits);
		return false;
	}
	if (stateful_decode) {
		picture_bytes = want_picture;
		mv_bytes = want_mv;
		slot_count = fits;
	}
	if (mb_total != stream_mb_total || mb_width != stream_mb_width)
		vdec_log("vdec: H.264 stream %ux%u (canvas stride %u), %u slots\n", mb_width * 16, mb_height * 16, stride,
		    slot_count);
	stream_mb_width = mb_width;
	stream_mb_total = mb_total;
	stream_stride = stride;
	stream_rows = mb_height * 16;
	if (!stateful_decode) {
		memset((void *)(test_pool + TEST_PICTURE_OFF), 0xa5, 0x40000);
		FlushPoC_DcacheRegion(test_pool + TEST_PICTURE_OFF, 0x40000);
	}
	unsigned count = stateful_decode ? slot_count : 2;
	unsigned references = stateful_decode ? 16 : 1;
	h264_test_canvases(count);
	uint32_t buffers = (references << 24) | (count << 16) | (count << 8);
	dos_wr(AV_SCRATCH_0 + 7, buffers);
	dos_wr(ASSIST_MBOX1_CLR, 1);
	dos_wr(H264_STATUS, 0xf2);
	dos_wr(AV_SCRATCH_0, buffers);
	return true;
}

static bool
h264_test_slice(void)
{
	FlushPoC_DcacheRegion(test_pool + TEST_LMEM_OFF, 4096);
	const uint16_t *rpm = (const uint16_t *)(test_pool + TEST_LMEM_OFF);
	if (dump_rpm) {
		for (unsigned at = 0; at < 0x400; at += 16) {
			char line[192];
			int used = snprintf(line, sizeof(line), "vdec: RPM %03x:", at);
			for (unsigned i = at; i < at + 16; i++)
				used += snprintf(line + used, sizeof(line) - used, " %04x", rpm[(i & ~3u) + 3 - (i & 3)]);
			vdec_log("%s\n", line);
		}
	}
	/* Firmware LMEM packs each group of four words in reverse order. */
	unsigned slice = rpm[(0x82 & ~3) + 3 - (0x82 & 3)];
	unsigned frame = rpm[(0x84 & ~3) + 3 - (0x84 & 3)];
	vdec_log("vdec: H.264 slice: type %u frame-only %u\n", slice, frame);
	if (slice != 2 || frame != 1) return false;
	dos_wr(POC_RESET, 0);
	dos_wr(POC_DATA, 0);
	dos_wr(POC_DATA, 0);
	dos_wr(POC_DATA, 0);
	dos_wr(REC_CANVAS, 0x010100);
	dos_wr(DBKR_CANVAS, 0x010100);
	dos_wr(DBKW_CANVAS, 0x010100);
	dos_wr(BUFFER_INFO_INDEX, 16);
	for (unsigned i = 0; i < 2; i++) {
		dos_wr(BUFFER_INFO_DATA, 0xf480 | (i == 0 ? 0xf : 0));
		dos_wr(BUFFER_INFO_DATA, 0);
		dos_wr(BUFFER_INFO_DATA, 0);
	}
	dos_wr(BUFFER_INFO_INDEX, 0);
	for (unsigned i = 0; i < 16; i++) dos_wr(BUFFER_INFO_DATA, 0);
	dos_wr(IQIDCT_CONTROL, dos_rd(IQIDCT_CONTROL) | (1u << 16));
	dos_wr(DCAC_BYTE64, (dos_rd(DCAC_BYTE64) & ~0xfu) | 0xa);
	dos_wr(CO_MB_WR_ADDR, (uint32_t)test_pool_pa + TEST_MV_OFF);
	dos_wr(CO_MB_RD_ADDR, 0);
	dos_wr(ASSIST_MBOX1_CLR, 1);
	dos_wr(H264_STATUS, 0xf3);
	return true;
}


/* vdec=3 stops at configuration; vdec=4 supplies fixed buffers for a single 64x64 I picture. */
static int
h264_decode64(const uint8_t *data, uint64_t size, uint8_t *output)
{
	uint32_t status = 0, previous = ~0u;
	bool restore = (stateful_decode || dump_rpm > 1) && test_picture_count;
	static vm_offset_t dmc;         /* mapped once: a mapping per packet runs the kernel out of address space */
	if (!dmc) dmc = ml_io_map(0xfe036000, 0x2000);

	if (!test_pool || !dmc || !data || size == 0 || size > TEST_INPUT_SIZE - 1024) {
		vdec_log("vdec: decode needs its DMA pool and an input packet under 1 MiB\n");
		return EINVAL;
	}
	uint32_t input = (uint32_t)(test_pool_pa + TEST_INPUT_OFF);
	uint32_t lmem = (uint32_t)(test_pool_pa + TEST_LMEM_OFF);
	/* Diagnostic mode 2 exercises firmware workspace restoration between packets. */
	if (restore) {
		memset((void *)(test_pool + TEST_INPUT_OFF), 0, (size_t)size + 2048);
		memcpy((void *)(test_pool + TEST_INPUT_OFF), data, (size_t)size);
		FlushPoC_DcacheRegion(test_pool + TEST_INPUT_OFF, (size_t)size + 2048);
	} else {
		/* A new stream: the firmware workspace, input and LMEM start out clear. */
		memset((void *)test_pool, 0, TEST_PICTURE_OFF);
		memcpy((void *)(test_pool + TEST_INPUT_OFF), data, (size_t)size);
		FlushPoC_DcacheRegion(test_pool, TEST_PICTURE_OFF);
	}
	vdec_trace("vdec: H.264 input test: %llu bytes, workspace %#llx, input %#x, LMEM %#x\n", size, test_pool_pa, input, lmem);
	*(volatile uint32_t *)dmc |= 1u << 8;
	__builtin_arm_dsb(0xf);
	dos_wr(DOS_SW_RESET0, (1u << 3) | (1u << 4) | (1u << 5) | (1u << 7) | (1u << 8) | (1u << 9));
	dos_wr(DOS_SW_RESET0, 0);
	dos_wr(VDEC_ASSIST_MMC_CTRL1, dos_rd(VDEC_ASSIST_MMC_CTRL1) & ~(1u << 3));
	dos_wr(MDEC_PIC_DC_MUX_CTRL, dos_rd(MDEC_PIC_DC_MUX_CTRL) & ~(1u << 31));
	dos_wr(MDEC_EXTIF_CFG1, 0);
	/* vdec_prepare_input initializes VLD control before selecting the manual FIFO. */
	dos_wr(POWER_CTL_VLD, 1u << 4);
	dos_wr(FIFO_START, input);
	dos_wr(FIFO_END, input + TEST_INPUT_SIZE - 8);
	dos_wr(FIFO_CURR, input);
	dos_wr(FIFO_CONTROL, 1);
	dos_wr(FIFO_CONTROL, 0);
	dos_wr(FIFO_BUF_CNTL, 2);
	dos_wr(FIFO_RP, input);
	dos_wr(FIFO_WP, input + (((uint32_t)size + 1023) & ~63u));
	dos_wr(FIFO_BUF_CNTL, 3);
	dos_wr(FIFO_BUF_CNTL, 2);
	dos_wr(FIFO_CONTROL, (0x11u << 16) | (1u << 10) | (7u << 3));
	dos_wr(POWER_CTL_VLD, dos_rd(POWER_CTL_VLD) | (1u << 9) | (1u << 6));
	dos_wr(PSCALE_CTRL, 0);
	dos_wr(ASSIST_MBOX1_MASK, 0); /* Polling only; no unhandled interrupt on the GIC. */
	dos_wr(ASSIST_MBOX1_CLR, 1);
	dos_wr(MDEC_PIC_DC_CTRL, (dos_rd(MDEC_PIC_DC_CTRL) | (1u << 17)) & ~((1u << 16) | (1u << 31)));
	dos_wr(MDEC_PIC_DC_THRESH, 0x404038aa);
	uint32_t buffers = restore && stateful_decode ?
	    (16u << 24) | (slot_count << 16) | (slot_count << 8) : 0;
	/* vh264_hw_ctx_restore: once the DPB exists only AV_SCRATCH_7 carries its size; AV_SCRATCH_0 keeps
	 * what the firmware left there. */
	if (!(restore && stateful_decode)) dos_wr(AV_SCRATCH_0, buffers);
	dos_wr(AV_SCRATCH_0 + 7, buffers);
	dos_wr(H264_WORK, (uint32_t)test_pool_pa + 0x10000);
	/* vh264_hw_ctx_restore: the saved value, bits 2-5 and 6 clear, bit 4 set: the firmware's own error
	 * recovery, as vmh264 always sets it (without it the third picture of a GOP is never found) */
	dos_wr(AV_SCRATCH_0 + 15, ((restore ? test_saved_reg_f : 0) & 0xffffff83u) | (1u << 4));
	dos_wr(H264_LMEM, lmem);
	dos_wr(AV_SCRATCH_0 + 22, 1);
	dos_wr(AV_SCRATCH_0 + 23, 0);
	dos_wr(AV_SCRATCH_0 + 18, restore ? test_picture_count : 0);
	dos_wr(AV_SCRATCH_0 + 1, 0); /* bits of this packet already consumed (vmh264 run) */
	dos_wr(AV_SCRATCH_0 + 2, restore ? 1 : 0);
	dos_wr(AV_SCRATCH_0 + 3, 0);
	dos_wr(AV_SCRATCH_0 + 4, 1); /* MULTI_FRAMEBASE */
	dos_wr(AV_SCRATCH_0 + 5, restore ? test_sequence_info : 0);
	dos_wr(H264_STATUS, restore && stateful_decode ? 0xff : 0);
	dos_wr(AV_SCRATCH_0 + 12, 0); /* no auxiliary output */
	dos_wr(AV_SCRATCH_0 + 17, 0);
	dos_wr(AV_SCRATCH_0 + 20, restore ? ucode_debug & 3 : 0); /* vmh264 udebug_flag (print bits only) */
	dos_wr(H264_DECODE_INFO, 1u << 13);
	dos_wr(AV_SCRATCH_0 + 14, (uint32_t)size);
	dos_wr(VIFF_BIT_CNT, (uint32_t)size * 8);
	dos_wr(AV_SCRATCH_0 + 9, (1u << 2) | (restore && stateful_decode ?
	    ((test_sequence_control & 0xff) << 7) | (test_bitstream_restriction << 15) : 0));
	dos_wr(MDEC_EXTIF_CFG2, dos_rd(MDEC_EXTIF_CFG2) | 0x20);
	if (restore && stateful_decode) {
		/* Core reset clears the reference canvas table as well as the pipeline. */
		h264_test_canvases(slot_count);
		if (saved_iqidct_control) dos_wr(IQIDCT_CONTROL, saved_iqidct_control);
		if (saved_vcop_control) dos_wr(VCOP_CONTROL, saved_vcop_control);
		if (saved_vld_control) dos_wr(VLD_CONTROL, saved_vld_control);
	}
	dos_wr(FIFO_CONTROL, dos_rd(FIFO_CONTROL) | 6);
	dos_wr(DOS_SW_RESET0, (1u << 12) | (1u << 11));
	dos_wr(DOS_SW_RESET0, 0);
	(void)dos_rd(DOS_SW_RESET0);
	(void)dos_rd(DOS_SW_RESET0);
	(void)dos_rd(DOS_SW_RESET0);
	__builtin_arm_dsb(0xf);
	dos_wr(MPSR, 1);
	__builtin_arm_dsb(0xf);
	dos_wr(H264_STATUS, H264_SEARCH_HEAD);
	__builtin_arm_dsb(0xf);
	for (unsigned ticks = 0; ticks < 2000; ticks++) {
		status = dos_rd(H264_STATUS);
		if (h264_firmware_debug()) break;
		if (status != previous) {
			vdec_trace("vdec: H.264 status %#x, PC %#x, FIFO %#x, bitcount %u\n", status,
			    dos_rd(MPC_E), dos_rd(FIFO_LEVEL), dos_rd(VIFF_BIT_CNT));
			previous = status;
		}
		if (status == H264_CONFIG_REQUEST) {
			if (test_level < 4 || !h264_test_configure()) break;
		}
		if (status == H264_SLICE_HEAD_DONE) {
			if (stateful_decode) {
				saved_iqidct_control = dos_rd(IQIDCT_CONTROL);
				saved_vcop_control = dos_rd(VCOP_CONTROL);
				saved_vld_control = dos_rd(VLD_CONTROL);
				vdec_trace("vdec: slice context: IQ %#x VCOP %#x VLD %#x\n",
				    saved_iqidct_control, saved_vcop_control, saved_vld_control);
				header_pending = true;
				return EINPROGRESS; /* Firmware waits for the validated reference configuration. */
			}
			if (test_level < 4 || !h264_test_slice()) break;
		}
		if (status == 2 || status == 0x20 || status == 0x21 || status == 0x22 || status == 0x23) break;
		if (status == H264_SEI_READY) {
			/* No user-data consumer in this probe; acknowledge the optional SEI. */
			dos_wr(ASSIST_MBOX1_CLR, 1);
			dos_wr(H264_STATUS, H264_SEI_DONE);
		}
		if (status == 0x13) { /* Non-MMU firmware write-response handshake. */
			dos_wr(ASSIST_MBOX1_CLR, 1);
			dos_wr(H264_STATUS, 0x14);
		}
		IOSleep(1);
	}
	dos_wr(MPSR, 0);
	dos_wr(CPSR, 0);
	dos_wr(ASSIST_MBOX1_MASK, 0);
	dos_wr(ASSIST_MBOX1_CLR, 1);
	__builtin_arm_dsb(0xf);
	FlushPoC_DcacheRegion(test_pool + TEST_LMEM_OFF, 4096);
	const uint16_t *rpm = (const uint16_t *)(test_pool + TEST_LMEM_OFF);
	vdec_log("vdec: H.264 input test stopped at %#x: RPM %04x %04x %04x %04x; FIFO RP %#x WP %#x\n",
	    status, rpm[0], rpm[1], rpm[2], rpm[3], dos_rd(FIFO_RP), dos_rd(FIFO_WP));
	if (status != 2) return EIO;
	test_saved_reg_f = dos_rd(AV_SCRATCH_0 + 15);
	if (dump_rpm) vdec_log("vdec: saved H.264 context: F %#x, init %#x, seq %#x\n",
	    test_saved_reg_f, dos_rd(AV_SCRATCH_0 + 2), test_sequence_info);
	test_picture_count++;
	if (output) {
		FlushPoC_DcacheRegion(test_pool + TEST_PICTURE_OFF, 0x20000);
		memcpy(output, (const void *)(test_pool + TEST_PICTURE_OFF), 4096);
		memcpy(output + 4096, (const void *)(test_pool + TEST_PICTURE_OFF + 0x10000), 2048);
		return 0;
	}
	if (test_level >= 4) {
		FlushPoC_DcacheRegion(test_pool + TEST_PICTURE_OFF, 0x20000);
		const uint8_t *y = (const uint8_t *)(test_pool + TEST_PICTURE_OFF);
		const uint8_t *uv = y + 0x10000;
		uint64_t reference_size = 0;
		const uint8_t *reference = hakios_image_file("fw", "vdec.nv12", &reference_size, NULL);
		unsigned y_bad = 0, uv_bad = 0;

		if (reference && reference_size != 6144) {
			vdec_log("vdec: H.264 reference has wrong size %llu (expected 6144)\n", reference_size);
			return EINVAL;
		}
		for (unsigned i = 0; i < 4096; i++) y_bad += y[i] != (reference ? reference[i] : 81);
		for (unsigned i = 0; i < 2048; i++) uv_bad += uv[i] != (reference ? reference[4096 + i] : (i & 1 ? 240 : 90));
		vdec_log("vdec: H.264 %s NV12: Y %u UV %u,%u, mismatches %u/4096 + %u/2048: %s\n",
		    reference ? "reference64" : "red64",
		    y[0], uv[0], uv[1], y_bad, uv_bad, y_bad == 0 && uv_bad == 0 ? "PASS" : "FAIL");
	}
	/* Keep the pool reserved for the lifetime of this diagnostic boot. */
	return 0;
}

static void
h264_input_test(void)
{
	uint64_t size = 0;
	const uint8_t *data = hakios_image_file("fw", "vdec.h264", &size, NULL);
	(void)h264_decode64(data, size, NULL);
}

/* one core's clock: gate shut, source and divider, gate open, then its final mux on the first copy */
static void
core_clock(uint32_t ctrl, uint32_t mux)
{
	uint32_t v = clk_rd(ctrl) & ~0xffffu;

	clk_wr(ctrl, v | (clk_src << 9));
	IODelay(5);
	clk_wr(ctrl, v | (clk_src << 9) | (1u << 8));
	IODelay(5);
	clk_wr(mux, clk_rd(mux) & ~(1u << 15));
}

static bool
power_on(unsigned pd, const char *name)
{
	uint64_t before = sip(SIP_PWRC_GET, pd, 0, 0), ret, after;

	ret = sip(SIP_PWRC_SET, pd, 1, 0);
	IODelay(50);
	after = sip(SIP_PWRC_GET, pd, 0, 0);
	vdec_log("vdec: %s power domain %u: %s before, set returned %#llx, %s after\n", name, pd,
	    before ? "off" : "on", ret, after ? "OFF" : "on");
	return after == 0;
}

/* the DOS block's own registers (0x3fxx): readable as soon as a domain is on */
static void
show_dos(const char *when)
{
	vdec_log("vdec: %s: SW_RESET0 %#x GCLK_EN0 %#x MEM_PD_VDEC %#x SW_RESET3 %#x GCLK_EN3 %#x MEM_PD_HEVC %#x\n", when,
	    dos_rd(DOS_SW_RESET0), dos_rd(DOS_GCLK_EN0), dos_rd(DOS_MEM_PD_VDEC), dos_rd(DOS_SW_RESET3),
	    dos_rd(DOS_GCLK_EN3), dos_rd(DOS_MEM_PD_HEVC));
}

/* a core's own registers: an external abort unless its memories are on, its reset released and its clocks open;
 * one register per line, so that a fault names the register */
static void
show_core(const char *name, const uint32_t *regs, const char *const *names, unsigned n)
{
	for (unsigned i = 0; i < n; i++) {
		vdec_log("vdec: %s: reading %s (%#x)\n", name, names[i], regs[i]);
		vdec_log("vdec: %s: %s = %#x\n", name, names[i], dos_rd(regs[i]));
	}
}

static bool
scratch_test(uint32_t reg, const char *name)
{
	uint32_t old = dos_rd(reg), got;

	dos_wr(reg, 0x5a5aa5a5u);
	got = dos_rd(reg);
	dos_wr(reg, old);
	vdec_log("vdec: %s scratch: wrote 0x5a5aa5a5, read %#x: %s\n", name, got, got == 0x5a5aa5a5u ? "works" : "FAILED");
	return got == 0x5a5aa5a5u;
}

/* Experimental, explicitly requested by vdec=6. It is not a general video decoder capability. */
static LCK_GRP_DECLARE(vdec_probe_group, "twl_vdec_probe");
static LCK_MTX_DECLARE(vdec_probe_lock, &vdec_probe_group);
static bool probe_published;
static pid_t probe_owner;

/* Match the vendor stop ordering before resetting or reusing a picture context. */
static bool
h264_probe_stop(void)
{
	dos_wr(MPSR, 0);
	dos_wr(CPSR, 0);
	__builtin_arm_dsb(0xf);
	for (unsigned i = 0; i < 10000; i++) {
		if (!((dos_rd(IMEM_DMA_CTRL) | dos_rd(LMEM_DMA_CTRL) | dos_rd(0x0e12)) & 0x8000) &&
		    !(dos_rd(0x0353) & 0xfff)) return true;
		IODelay(10);
	}
	vdec_log("vdec: H.264 DMA did not drain before context stop\n");
	return false;
}

static void
h264_probe_cancel(void)
{
	dos_wr(MPSR, 0);
	dos_wr(CPSR, 0);
	dos_wr(ASSIST_MBOX1_MASK, 0);
	dos_wr(ASSIST_MBOX1_CLR, 1);
	__builtin_arm_dsb(0xf);
	header_pending = stateful_decode = false;
	if (core_codec == CODEC_HEVC || hakios_hevc_active()) hakios_hevc_cancel();
	if (core_codec == CODEC_VP9 || hakios_vp9_active()) hakios_vp9_cancel();
	core_codec = CODEC_H264;
	/* every token of the stream refused from now on; the plane stops showing it before the slots are reused */
	generation_base += 0x10000;
	for (unsigned i = 0; i < TWL_H264_SLOTS; i++) {
		slot_generation[i] = generation_base;
		slot_handed[i] = 0;
	}
	handed_count = 0;
	plane_passed = 0;
	hakios_vplane_stop();
	test_picture_count = test_sequence_info = test_saved_reg_f = 0;
	test_sequence_control = test_bitstream_restriction = 0;
	saved_iqidct_control = saved_vcop_control = saved_vld_control = 0;
}

static int
h264_probe_load(void)
{
	if (!h264_probe_stop()) return ETIMEDOUT;
	/* Between pictures vdec_reset_core's blocks and the two CPUs; the assist block (bit 2) keeps its state. */
	dos_wr(DOS_SW_RESET0, stateful_decode && test_picture_count ? 0x1bb8u : 0xfffffffcu);
	IODelay(20);
	dos_wr(DOS_SW_RESET0, 0);
	IODelay(10);
	int result = hakios_optee_load_video_fw(HAKIOS_VIDEO_DEC_H264_MULTI, HAKIOS_OPTEE_VDEC_LEGACY, false);
	if (result || dump_rpm) vdec_log("vdec: reload picture %u: result %d swap address %#x\n",
	    test_picture_count, result, dos_rd(AV_SCRATCH_0 + 16));
	return result ? EIO : 0;
}

static int
h264_probe_prepare(const struct twl_h264_prepare *request)
{
	if (test_level != 7) return ENOTSUP;
	if (header_pending) return EBUSY;
	if (!request->input_size || request->input_size > TEST_INPUT_SIZE - 1024 ||
	    request->rpm_size != TWL_H264_RPM_WORDS * sizeof(uint16_t)) return EINVAL;
	uint8_t *input = kalloc_data(request->input_size, Z_WAITOK);
	uint16_t *parameters = kalloc_data(request->rpm_size, Z_WAITOK | Z_ZERO);
	int error = ENOMEM;
	if (input && parameters) {
		error = copyin((user_addr_t)request->input, input, request->input_size);
		if (!error) {
			if (!stateful_decode) test_picture_count = 0;
			stateful_decode = true;
			error = h264_probe_load();
			if (!error) error = h264_decode64(input, request->input_size, NULL);
			if (error == EINPROGRESS && header_pending) {
				FlushPoC_DcacheRegion(test_pool + TEST_LMEM_OFF, 4096);
				const uint16_t *rpm = (const uint16_t *)(test_pool + TEST_LMEM_OFF);
				for (unsigned i = 0; i < TWL_H264_RPM_WORDS; i++)
					parameters[i] = rpm[(i & ~3u) + 3 - (i & 3)];
				error = copyout(parameters, (user_addr_t)request->rpm, request->rpm_size);
			} else if (!error) error = EIO;
		}
	}
	if (input) kfree_data(input, request->input_size);
	if (parameters) kfree_data(parameters, request->rpm_size);
	if (error) h264_probe_cancel();
	return error;
}

static int
h264_probe_next(const struct twl_h264_prepare *request)
{
	if (test_level != 7 || !stateful_decode || !test_picture_count || header_pending ||
	    request->input || request->input_size || request->rpm_size != TWL_H264_RPM_WORDS * sizeof(uint16_t))
		return EINVAL;
	if (!dos_rd(VIFF_BIT_CNT)) return EAGAIN;
	uint16_t *parameters = kalloc_data(request->rpm_size, Z_WAITOK | Z_ZERO);
	if (!parameters) return ENOMEM;
	/* In-packet continuation only releases the firmware handshake.
	 * Reinitializing DECODE_INFO here discards flags parsed from the sequence. */
	dos_wr(ASSIST_MBOX1_CLR, 1);
	dos_wr(H264_STATUS, H264_SEARCH_HEAD);
	dos_wr(MPSR, 1);
	__builtin_arm_dsb(0xf);
	/* Resume the diagnostic continuation after the preceding picture was drained. */
	uint32_t status = 0;
	int error = EIO;
	for (unsigned ticks = 0; ticks < 2000; ticks++) {
		status = dos_rd(H264_STATUS);
		if (h264_firmware_debug()) break;
		if (status == H264_SLICE_HEAD_DONE) {
			FlushPoC_DcacheRegion(test_pool + TEST_LMEM_OFF, 4096);
			const uint16_t *rpm = (const uint16_t *)(test_pool + TEST_LMEM_OFF);
			for (unsigned i = 0; i < TWL_H264_RPM_WORDS; i++)
				parameters[i] = rpm[(i & ~3u) + 3 - (i & 3)];
			header_pending = true;
			error = copyout(parameters, (user_addr_t)request->rpm, request->rpm_size);
			break;
		}
		if (status == H264_CONFIG_REQUEST && !h264_test_configure()) break;
		if (status == H264_SEI_READY || status == 0x13) {
			dos_wr(ASSIST_MBOX1_CLR, 1);
			dos_wr(H264_STATUS, status == H264_SEI_READY ? H264_SEI_DONE : 0x14);
		}
		if (status == 0x20 || status == 0x22) { error = EAGAIN; break; }
		if (status == 0x21 || status == 0x23) break;
		IOSleep(1);
	}
	vdec_trace("vdec: next stream header: status %#x, remaining %u bits, error %d\n",
	    status, dos_rd(VIFF_BIT_CNT), error);
	kfree_data(parameters, request->rpm_size);
	if (error) h264_probe_cancel();
	return error;
}

static int
h264_probe_finish(struct twl_h264_config *config)
{
	if (test_level != 7 || !stateful_decode || !header_pending) return EINVAL;
	if (config->current_slot >= slot_count || config->mv_write_slot < -1 ||
	    config->mv_write_slot >= (int)slot_count || config->mv_read_slot < -1 ||
	    config->mv_read_slot >= (int)slot_count || config->direct_8x8 > 1 ||
	    config->mv_read_bottom > 1 || config->first_mb >= stream_mb_total || (config->flags & ~(TWL_H264_SLICE | TWL_H264_NOCOPY)) ||
	    !config->rpm || config->output_size < STREAM_FRAME_BYTES ||
	    ((config->flags & TWL_H264_SLICE) && config->current_slot != picture_slot))
		return EINVAL;
	picture_slot = config->current_slot;
	for (unsigned i = 0; i < TWL_H264_SLOTS; i++) if (config->slots[i].long_term > 3) return EINVAL;
	for (unsigned list = 0; list < 2; list++) {
		if (config->reference_count[list] > 32) return EINVAL;
		for (unsigned i = 0; i < config->reference_count[list]; i++)
			if (config->references[list][i] >= slot_count) return EINVAL;
	}
	unsigned slot = config->current_slot, y = slot * 2, uv = y + 1;
	uint32_t canvas = y | (uv << 8) | (uv << 16);
	dos_wr(POC_RESET, 0);
	dos_wr(POC_DATA, (uint32_t)config->frame_poc);
	dos_wr(POC_DATA, (uint32_t)config->top_poc);
	dos_wr(POC_DATA, (uint32_t)config->bottom_poc);
	dos_wr(CURR_CANVAS, slot << 24);
	dos_wr(REC_CANVAS, canvas);
	dos_wr(DBKR_CANVAS, canvas);
	dos_wr(DBKW_CANVAS, canvas);
	dos_wr(BUFFER_INFO_INDEX, 16);
	for (unsigned i = 0; i < TWL_H264_SLOTS; i++) {
		const struct twl_h264_slot *picture = &config->slots[i];
		uint32_t info = 0xf480 | (picture->long_term << 4) |
		    (picture->bottom_poc < picture->top_poc ? 0x100 : 0) | (i == slot ? 0xf : 0);
		dos_wr(BUFFER_INFO_DATA, info);
		dos_wr(BUFFER_INFO_DATA, (uint32_t)picture->top_poc);
		dos_wr(BUFFER_INFO_DATA, (uint32_t)picture->bottom_poc);
	}
	for (unsigned list = 0; list < 2; list++) {
		dos_wr(BUFFER_INFO_INDEX, list * 8);
		unsigned count = config->reference_count[list];
		for (unsigned word = 0; word < 8; word++) {
			uint32_t packed = 0;
			for (unsigned byte = 0; byte < 4; byte++) {
				unsigned index = word * 4 + byte;
				unsigned reference = count ? config->references[list][index < count ? index : count - 1] : 0;
				packed = (packed << 8) | (count ? 0x60 | reference : 0);
			}
			dos_wr(BUFFER_INFO_DATA, packed);
		}
	}
	dos_wr(IQIDCT_CONTROL, dos_rd(IQIDCT_CONTROL) | (1u << 16));
	dos_wr(DCAC_BYTE64, (dos_rd(DCAC_BYTE64) & ~0xfu) | 0xa);
	unsigned shift = config->direct_8x8 ? 2 : 0;
	uint32_t mv_base = (uint32_t)test_pool_pa + TEST_MV_OFF;
	/* a picture without a co-located buffer of its own writes into the spare one after the slots' */
	unsigned write_slot = config->mv_write_slot < 0 ? slot_count : (unsigned)config->mv_write_slot;
	uint32_t mb_offset = (config->first_mb * 96) >> shift;
	dos_wr(CO_MB_WR_ADDR, mv_base + ((write_slot * MV_SLOT) >> shift) + mb_offset);
	dos_wr(CO_MB_RD_ADDR, config->mv_read_slot < 0 ? 0 :
	    ((mv_base + (((unsigned)config->mv_read_slot * MV_SLOT) >> shift) + mb_offset) >> 3) |
	    (2u << 30) | (config->mv_read_bottom << 29));
	header_pending = false;
	if (dos_rd(AV_SCRATCH_0 + 22) == 1) dos_wr(AV_SCRATCH_0 + 22, 0);
	dos_wr(ASSIST_MBOX1_CLR, 1);
	dos_wr(H264_STATUS, config->flags & TWL_H264_SLICE ? 0xf1 : 0xf3); /* DECODE_SLICE / DECODE_NEWPIC */
	__builtin_arm_dsb(0xf);
	uint32_t status = 0;
	for (unsigned ticks = 0; ticks < 2000; ticks++) {
		status = dos_rd(H264_STATUS);
		if (h264_firmware_debug()) break;
		if (status == H264_SLICE_HEAD_DONE) {
			/* the picture's next slice: its header goes to user space, the firmware waits */
			saved_iqidct_control = dos_rd(IQIDCT_CONTROL);
			saved_vcop_control = dos_rd(VCOP_CONTROL);
			saved_vld_control = dos_rd(VLD_CONTROL);
			FlushPoC_DcacheRegion(test_pool + TEST_LMEM_OFF, 4096);
			uint16_t *parameters = kalloc_data(TWL_H264_RPM_WORDS * sizeof(uint16_t), Z_WAITOK);
			if (!parameters) break;
			const uint16_t *rpm = (const uint16_t *)(test_pool + TEST_LMEM_OFF);
			for (unsigned i = 0; i < TWL_H264_RPM_WORDS; i++)
				parameters[i] = rpm[(i & ~3u) + 3 - (i & 3)];
			int error = copyout(parameters, (user_addr_t)config->rpm, TWL_H264_RPM_WORDS * sizeof(uint16_t));
			kfree_data(parameters, TWL_H264_RPM_WORDS * sizeof(uint16_t));
			if (error) break;
			header_pending = true;
			return EINPROGRESS;
		}
		if (status == 2 || status == 0x20 || status == 0x21 || status == 0x22 ||
		    status == 0x23 || status == H264_CONFIG_REQUEST) break;
		if (status == H264_SEI_READY || status == 0x13) {
			dos_wr(ASSIST_MBOX1_CLR, 1);
			dos_wr(H264_STATUS, status == H264_SEI_READY ? H264_SEI_DONE : 0x14);
		}
		IOSleep(1);
	}
	__builtin_arm_dsb(0xf);
	int error = EIO;
	bool drained = h264_probe_stop();
	if (status == 2 && drained) {
		test_saved_reg_f = dos_rd(AV_SCRATCH_0 + 15);
		test_picture_count++;
		vm_offset_t picture = test_pool + TEST_PICTURE_OFF + slot * PICTURE_SLOT;
		/* the decoder wrote it behind the caches: for the CPU's copy, and for no stale line to be written over it */
		FlushPoC_DcacheRegion(picture, STREAM_FRAME_BYTES);
		error = config->flags & TWL_H264_NOCOPY ? 0 :
		    copyout((const void *)picture, (user_addr_t)config->output, STREAM_FRAME_BYTES);
		config->generation = ++slot_generation[slot];
		slot_handed[slot] = ++handed_count;
	}
	if (!error && !dump_rpm) return 0;
	vdec_log("vdec: reference-picture frame: slot %u, status %#x, error %d\n", slot, status, error);
	vdec_log("vdec: frame execution: MPSR %#x PC %#x CPSR %#x CPC %#x, bits %u, control %#x, buf %#x/%#x count %u\n",
	    dos_rd(MPSR), dos_rd(MPC_E), dos_rd(CPSR), dos_rd(0x0328), dos_rd(VIFF_BIT_CNT),
	    dos_rd(H264_DECODE_INFO), dos_rd(AV_SCRATCH_0), dos_rd(AV_SCRATCH_0 + 7), dos_rd(AV_SCRATCH_0 + 18));
	vdec_log("vdec: frame diagnostic: debug %#x/%#x MB %#x DMA %#x co-MB %#x\n",
	    dos_rd(AV_SCRATCH_0 + 22), dos_rd(AV_SCRATCH_0 + 23), dos_rd(0x0c07), dos_rd(0x0e12), dos_rd(0x0c3d));
	if (error) h264_probe_cancel();
	return error;
}

/* The video plane: the picture a token names, if it is still in its slot. */
int
hakios_vdec_plane_picture(uint32_t slot, uint32_t generation, struct hakios_vdec_picture *out)
{
	int error = 0;

	plane_wanted = mach_absolute_time();

	lck_mtx_lock(&vdec_probe_lock);
	if (test_level != 7 || !test_pool || !stateful_decode || slot >= (core_codec == CODEC_VP9 ? TWL_VP9_SLOTS : core_codec == CODEC_HEVC ? TWL_HEVC_SLOTS : slot_count) ||
	    !slot_handed[slot] || slot_generation[slot] != generation) {
		error = ESTALE;
	} else if (core_codec == CODEC_HEVC) {
		if (!hakios_hevc_picture(slot, out)) error = ESTALE;
	} else if (core_codec == CODEC_VP9) {
		if (!hakios_vp9_picture(slot, out)) error = ESTALE;
	} else {
		out->luma = test_pool_pa + TEST_PICTURE_OFF + (uint64_t)slot * PICTURE_SLOT;
		out->chroma = out->luma + (uint64_t)stream_stride * stream_rows;
		out->stride = stream_stride;
		out->rows = stream_rows;
		out->width = stream_mb_width * 16;
		out->compressed = 0;
		out->depth = 8;
		out->coded_width = out->width;
		out->coded_height = out->rows;
	}
	if (!error && slot < 32) {
		/*
		 * On the plane from here on: it marks the slot pending only after this returns, outside this lock, and a decode
		 * in between that ran short of pages took this picture's (not yet "showing") and decoded into them while the VPU
		 * read them -- colourful lines over Kodi's picture for a frame. hakios_vdec_plane_showing() lets the claim go.
		 */
		os_atomic_or(&plane_claimed, 1u << slot, release);
		if (!out->compressed && (core_codec == CODEC_HEVC || core_codec == CODEC_VP9)) lat_copies++;
	}
	lck_mtx_unlock(&vdec_probe_lock);
	return error;
}

/* Bring-up only (kern.hakios_vpu's test picture): the last SIZE bytes of the pool, which a stream of 1920x1088 or
 * smaller leaves unused. Nothing guards them against a 4K stream. */
int
hakios_vdec_scratch(uint32_t size, vm_offset_t *va, uint64_t *pa)
{
	if (!test_pool || size > test_pool_size / 4) return ENOMEM;
	*va = test_pool + test_pool_size - size;
	*pa = test_pool_pa + test_pool_size - size;
	return 0;
}

/*
 * Diagnosis (the plane gets a fraction of the pictures decoded): from a picture's hand-out to the player to the plane
 * taking its slot, and the pictures whose slot is handed out again before the plane ever took them.
 */
static uint64_t out_at[LAT_SLOTS];
static uint32_t lat_out, lat_shown, lat_never, lat_b[6], lat_steps[4], lat_last_handed;
static uint64_t lat_max;

static void
lat_take(int slot, uint64_t now)
{
	if (slot < 0 || slot >= LAT_SLOTS || !out_unshown[slot]) return;
	out_unshown[slot] = false;
	{
		/* the step in hand-out order from the picture the plane took before: 1 the next one, more skipped, 0 or less back */
		int32_t step = (int32_t)(slot_handed[slot] - lat_last_handed);
		lat_steps[step <= 0 ? 0 : step == 1 ? 1 : step == 2 ? 2 : 3]++;
		lat_last_handed = slot_handed[slot];
	}
	if (slot_handed[slot] > plane_passed) plane_passed = slot_handed[slot];
	uint64_t ns;
	absolutetime_to_nanoseconds(now - out_at[slot], &ns);
	uint64_t ms = ns / 1000000;
	lat_shown++;
	lat_b[ms < 20 ? 0 : ms < 40 ? 1 : ms < 60 ? 2 : ms < 100 ? 3 : ms < 200 ? 4 : 5]++;
	if (ms > lat_max) lat_max = ms;
}

static void
note_out(unsigned slot)
{
	if (slot >= LAT_SLOTS) return;
	if (out_unshown[slot]) lat_never++;
	out_at[slot] = mach_absolute_time();
	out_unshown[slot] = true;
	if (++lat_out >= 300) {
		printf("vdec: to the plane: of %u pictures handed out %u taken (after <20 %u, <40 %u, <60 %u, <100 %u, <200 %u, "
		    "more %u ms; the longest %llu), %u handed out again before\n", lat_out, lat_shown, lat_b[0], lat_b[1], lat_b[2],
		    lat_b[3], lat_b[4], lat_b[5], lat_max, lat_never);
		printf("vdec: to the plane, in order: %u the next picture, %u one skipped, %u more skipped, %u the same or back; "
		    "%u as their copy\n", lat_steps[1], lat_steps[2], lat_steps[3], lat_steps[0], lat_copies);
		lat_copies = 0;
		memset(lat_steps, 0, sizeof(lat_steps));
		lat_out = lat_shown = lat_never = 0;
		lat_max = 0;
		memset(lat_b, 0, sizeof(lat_b));
	}
}

/* The video plane: what it shows and what waits for the vsync (-1: nothing). Lock-free (the plane's lock is held). */
void
hakios_vdec_plane_showing(int slot, int pending)
{
	uint64_t now = mach_absolute_time();
	lat_take(pending, now);
	lat_take(slot, now);
	if (slot != plane_slot) {
		plane_prev = plane_slot;
		plane_prev_vsyncs = 0;
	}
	plane_slot = slot;
	plane_pending = pending;
	os_atomic_store(&plane_claimed, 0, release);
	if (slot >= 0 || pending >= 0) {
		plane_seen = mach_absolute_time();
	}
}

static int
vdec_probe_open(__unused dev_t dev, __unused int flags, __unused int type, struct proc *p)
{
	if (!hakios_optee_video_fw_ready() || !vdec_powered) return ENXIO;
	lck_mtx_lock(&vdec_probe_lock);
	int error = probe_owner ? EBUSY : 0;
	if (!error) { h264_probe_cancel(); probe_owner = proc_pid(p); }
	lck_mtx_unlock(&vdec_probe_lock);
	return error;
}

static int
vdec_probe_close(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	lck_mtx_lock(&vdec_probe_lock);
	h264_probe_cancel();
	probe_owner = 0;
	lck_mtx_unlock(&vdec_probe_lock);
	return 0;
}

/* the slots whose compressed pictures the plane reads now or from the next vsync: their pages are never taken back */
uint32_t
hakios_vdec_showing(unsigned slots)
{
	int shown = plane_slot, pending = plane_pending, prev = plane_prev;
	uint32_t showing = os_atomic_load(&plane_claimed, acquire);

	if (prev >= 0 && (unsigned)prev < slots && prev < 32) showing |= 1u << prev;

	if (shown >= 0 && (unsigned)shown < slots && shown < 32) showing |= 1u << shown;
	if (pending >= 0 && (unsigned)pending < slots && pending < 32) showing |= 1u << pending;
	return showing & (slots >= 32 ? ~0u : (1u << slots) - 1);
}

/*
 * Before a new picture is decoded into SLOT: while the plane shows it (or is about to, or let it go less than a frame
 * ago), wait. User space chose the slot from what the plane showed when it last asked, and the plane may have taken it
 * since; the decode first gives the slot's pages back and takes them again, its first page (the old picture's top rows)
 * first, so the picture on the screen got the new one's top rows: a line of coloured dots near the top for a frame.
 */
void
hakios_vdec_slot_wait(unsigned slot, unsigned slots)
{
	static uint32_t waits, waited_ms;
	unsigned ms = 0;

	while (ms < 120 && (hakios_vdec_showing(slots) & (1u << slot))) {
		IOSleep(2);
		ms += 2;
	}
	if (ms) {
		waits++;
		waited_ms += ms;
		if (waits <= 5 || waits % 100 == 0) {
			printf("vdec: a picture to be decoded into slot %u the plane still showed: waited %u ms (%u times, %u ms in all)\n",
			    slot, ms, waits, waited_ms);
		}
	}
}

/* each vsync of the plane (its worker): the picture before is let go two vsyncs after it was replaced */
void
hakios_vdec_plane_vsync(void)
{
	if (plane_prev >= 0 && ++plane_prev_vsyncs >= 2) plane_prev = -1;
}

/* the slots a player may still ask for or the plane shows (decoded into last; their pictures kept) */
uint32_t
hakios_vdec_held(unsigned slots)
{
	uint32_t held = 0;
	uint64_t second, now = mach_absolute_time();

	nanoseconds_to_absolutetime(NSEC_PER_SEC, &second);
	if (!plane_wanted || now - plane_wanted >= second) return 0;
	for (unsigned i = 0; i < slots && i < 32; i++) {
		if ((slot_handed[i] && handed_count - slot_handed[i] < HOLD_PICTURES && out_unshown[i]) ||
		    (int)i == plane_slot || (int)i == plane_pending) {
			held |= 1u << i;
		}
	}
	return held;
}

/*
 * The HEVC core's mailbox 0 interrupt (dt vdec "mailbox_0": SPI 91, the GIC's 123; boot-arg vdec_irq=N another,
 * 0 none), raised by the H.265 and VP9 microcode at each stop (header parsed, picture done, a probability request).
 * Their waits sleep until it comes (or 250 us; three stops in four raise none) instead of polling every 25 us: at 4K60 that polling kept a core
 * busy for the ~10 ms of every picture.
 */
extern kern_return_t gic_v3_spi_attach(uint32_t intid, bool edge, void (*handler)(void *context, uint32_t intid), void *context);
extern void gic_v3_spi_mask(uint32_t intid);
extern void gic_v3_spi_unmask(uint32_t intid);
extern wait_result_t assert_wait_deadline(event_t event, wait_interrupt_t interruptible, uint64_t deadline);
extern wait_result_t thread_block(thread_continue_t continuation);
extern kern_return_t thread_wakeup_prim(event_t event, boolean_t one_thread, wait_result_t result);
extern kern_return_t clear_wait(thread_t thread, wait_result_t result);
#define HEVC_MBOX0_CLR  0x3071
static uint32_t vdec_irq = 123;
static int vdec_irq_state;              /* 0 not tried, 1 attached, -1 none */
static volatile uint32_t vdec_irq_seq;
static uint32_t nap_woken, nap_timed, nap_quick;

/*
 * In interrupt context nothing of the decoder is touched (it may be between a stop and a reset, its registers then
 * hang the bus; the box froze twice with the handler acknowledging the mailbox itself): the line is masked and the
 * waiting thread woken; the thread, in a decode, acknowledges the mailbox and unmasks the line before it sleeps again.
 */
static volatile bool vdec_irq_masked;

static void
vdec_irq_handler(__unused void *context, uint32_t intid)
{
	gic_v3_spi_mask(intid);
	vdec_irq_masked = true;
	os_atomic_inc(&vdec_irq_seq, relaxed);
	thread_wakeup_prim((event_t)&vdec_irq_seq, FALSE, THREAD_AWAKENED);
}

/* before looking at the core's status: what a nap after it compares with */
uint32_t
hakios_vdec_irq_seq(void)
{
	return os_atomic_load(&vdec_irq_seq, acquire);
}

/* sleep until the core's interrupt after seq, or us; false (at once) without the interrupt */
bool
hakios_vdec_irq_nap(uint32_t seq, unsigned us)
{
	if (vdec_irq_state == 0) {
		static int trying;
		if (os_atomic_cmpxchg(&trying, 0, 1, acq_rel)) {
			kern_return_t kr = vdec_irq ? gic_v3_spi_attach(vdec_irq, true, vdec_irq_handler, NULL) : KERN_FAILURE;
			printf("vdec: the HEVC core's interrupt %u: %s (%d)\n", vdec_irq, kr == KERN_SUCCESS ? "attached" : "not used", kr);
			os_atomic_store(&vdec_irq_state, kr == KERN_SUCCESS ? 1 : -1, release);
		}
	}
	if (os_atomic_load(&vdec_irq_state, acquire) != 1) return false;
	if (vdec_irq_masked) {
		/* (from a decode: the core runs) the mailbox acknowledged, then the line open again */
		hakios_dos_wr(HEVC_MBOX0_CLR, 1);
		vdec_irq_masked = false;
		gic_v3_spi_unmask(vdec_irq);
	}
	uint64_t deadline;
	clock_interval_to_deadline(us, NSEC_PER_USEC, &deadline);
	assert_wait_deadline((event_t)&vdec_irq_seq, THREAD_UNINT, deadline);
	if (os_atomic_load(&vdec_irq_seq, acquire) != seq) {
		clear_wait(current_thread(), THREAD_AWAKENED);
		nap_quick++;
	} else if (thread_block(THREAD_CONTINUE_NULL) == THREAD_TIMED_OUT) {
		nap_timed++;
	} else {
		nap_woken++;
	}
	if (nap_woken + nap_timed + nap_quick >= 2000) {
		printf("vdec: interrupt %u: of 2000 waits %u woken by it, %u already had it, %u ran out of time\n", vdec_irq,
		    nap_woken, nap_quick, nap_timed);
		nap_woken = nap_timed = nap_quick = 0;
	}
	return true;
}

/* for hakios.hevc.c */
uint32_t
hakios_dos_rd(uint32_t reg)
{
	return dos_rd(reg);
}

void
hakios_dos_wr(uint32_t reg, uint32_t value)
{
	dos_wr(reg, value);
}

int
hakios_vdec_copy_memory(vm_offset_t *va, uint64_t *pa, uint32_t *size)
{
	if (!test_pool || !copy_size) return ENOMEM;
	*va = test_pool + test_pool_size;
	*pa = test_pool_pa + test_pool_size;
	*size = copy_size;
	return 0;
}

bool
hakios_vdec_pool(vm_offset_t *va, uint64_t *pa, uint32_t *size)
{
	if (!test_pool) return false;
	*va = test_pool;
	*pa = test_pool_pa;
	*size = test_pool_size;
	return true;
}

/* an HEVC unit or slice, a VP9 frame: a stream of another codec is ended first; a finished picture gets its
 * generation */
static int
hevc_ioctl(u_long command, caddr_t data)
{
	int codec = command == TWL_VP9_PREPARE || command == TWL_VP9_FRAME ? CODEC_VP9 : CODEC_HEVC;
	if (test_level != 7 || !hevc_powered) return ENOTSUP;
	if (core_codec != codec) {
		h264_probe_cancel();
		core_codec = codec;
	}
	stateful_decode = true;
	int error;
	if (command == TWL_HEVC_PREPARE) {
		error = hakios_hevc_prepare((const struct twl_h264_prepare *)data);
	} else if (command == TWL_VP9_PREPARE) {
		error = hakios_vp9_prepare((const struct twl_h264_prepare *)data);
	} else if (command == TWL_VP9_FRAME) {
		struct twl_vp9_frame *frame = (struct twl_vp9_frame *)data;
		error = hakios_vp9_frame(frame);
		if (!error && !(frame->flags & TWL_VP9_SHOW)) {
			frame->generation = ++slot_generation[frame->slot];
			/* (a hidden frame, an alternate reference, never goes to the player: not one to hold for it) */
			slot_handed[frame->slot] = hakios_vp9_shown() ? ++handed_count : 0;
			if (hakios_vp9_shown()) note_out(frame->slot);
		} else if (!error) {
			frame->generation = slot_generation[frame->slot];       /* (shown again: its picture has not changed) */
			slot_handed[frame->slot] = ++handed_count;
			note_out(frame->slot);
		}
	} else {
		struct twl_hevc_slice *slice = (struct twl_hevc_slice *)data;
		error = hakios_hevc_slice(slice);
		if (!error) {
			slice->generation = ++slot_generation[slice->slot];
			slot_handed[slice->slot] = ++handed_count;
			note_out(slice->slot);
		}
	}
	if (error && error != EINPROGRESS && error != ENODATA) {
		h264_probe_cancel();
	}
	return error;
}

/* whether the pool holds SLOTS_MIN slots of a picture this size */
static bool
pool_fits(unsigned width, unsigned height)
{
	uint32_t picture = ((((width + 63) & ~63u) * height * 3 / 2) + 0xffffu) & ~0xffffu;
	uint32_t mv = ((width / 16) * (height / 16) * 96 + 0xffffu) & ~0xffffu;
	return test_pool_size >= TEST_PICTURE_OFF + mv + SLOTS_MIN * (picture + mv);
}

/*
 * The HEVC core's reset between pictures, as vdec.c's hevc_reset_core does it on S7D: its AXI arbiter shut and
 * drained first (a reset with a transfer in flight hangs the bus), the core's blocks reset, then its bridge to the
 * DMC, and the arbiter opened again.
 */
#define HEVC_ASSIST_AFIFO_CTRL  0x3001
#define HEVC_ASSIST_AXI_CTRL    0x307c
#define HEVC_ASSIST_AXI_STATUS  0x307d
#define HEVC_STREAM_CONTROL     0x3101
#define HEVC_SAO_MMU_RESET_CTRL 0x3641
#define HEVC_WRRSP_LMEM         0x3353

static bool
dos_wait(uint32_t reg, uint32_t mask, bool set)
{
	for (unsigned i = 0; i < 10000; i++) {
		if (((dos_rd(reg) & mask) != 0) == set) return true;
		IODelay(10);
	}
	vdec_log("vdec: register %#x mask %#x did not become %s (%#x)\n", reg, mask, set ? "set" : "clear", dos_rd(reg));
	return false;
}

void
hakios_hevc_reset_core(void)
{
	dos_wr(DOS_GCLK_EN3, dos_rd(DOS_GCLK_EN3) | (1u << 2));
	dos_wr(HEVC_STREAM_CONTROL, 0);
	dos_wr(HEVC_ASSIST_AXI_CTRL, dos_rd(HEVC_ASSIST_AXI_CTRL) | (1u << 6));
	(void)dos_wait(HEVC_ASSIST_AXI_STATUS, (1u << 15) | (1u << 11), true);
	(void)dos_wait(HEVC_ASSIST_AFIFO_CTRL, 1u << 28, false);
	IODelay(1);
	dos_wr(HEVC_SAO_MMU_RESET_CTRL, dos_rd(HEVC_SAO_MMU_RESET_CTRL) | 1);
	dos_wr(DOS_SW_RESET3, (1u << 3) | (1u << 4) | (1u << 8) | (1u << 10) | (1u << 11) | (1u << 12) | (1u << 13) |
	    (1u << 14) | (1u << 15) | (1u << 17) | (1u << 18) | (1u << 19) | (1u << 24) | (1u << 26));
	dos_wr(DOS_SW_RESET3, 0);
	(void)dos_wait(HEVC_WRRSP_LMEM, 0xfff, false);
	dos_wr(HEVC_SAO_MMU_RESET_CTRL, dos_rd(HEVC_SAO_MMU_RESET_CTRL) & ~1u);
	volatile uint32_t *r5 = (volatile uint32_t *)(reset + RESET5_LEVEL);
	*r5 &= ~RESET_BRG_HEVCF;
	__builtin_arm_dsb(0xf);
	*r5 |= RESET_BRG_HEVCF;
	__builtin_arm_dsb(0xf);
	dos_wr(HEVC_ASSIST_AXI_CTRL, dos_rd(HEVC_ASSIST_AXI_CTRL) & ~(1u << 6));
}

/* vdec_debug=1: see struct twl_vdec_dbg_reg */
static int
vdec_debug_ioctl(u_long command, caddr_t data)
{
	if (!debug_access || !test_pool) return ENOTTY;
	if (command == TWL_VDEC_DBG_INFO) {
		struct twl_vdec_dbg_info info = { test_pool_pa, test_pool_size, hevc_powered };
		memcpy(data, &info, sizeof(info));
		return 0;
	}
	if (command == TWL_VDEC_DBG_REG) {
		struct twl_vdec_dbg_reg *r = (struct twl_vdec_dbg_reg *)data;
		if (r->reg >= DOS_SIZE / 4) return EINVAL;
		if (r->write) {
			dos_wr(r->reg, r->value);
			__builtin_arm_dsb(0xf);
		} else {
			r->value = dos_rd(r->reg);
		}
		return 0;
	}
	if (command == TWL_VDEC_DBG_MEM) {
		const struct twl_vdec_dbg_mem *m = (const struct twl_vdec_dbg_mem *)data;
		if (!m->size || m->offset >= test_pool_size || m->size > test_pool_size - m->offset) return EINVAL;
		vm_offset_t at = test_pool + m->offset;
		int error;
		if (m->write) {
			error = copyin((user_addr_t)m->buffer, (void *)at, m->size);
			FlushPoC_DcacheRegion(at, m->size);
		} else {
			FlushPoC_DcacheRegion(at, m->size);
			error = copyout((const void *)at, (user_addr_t)m->buffer, m->size);
		}
		return error;
	}
	if (command == TWL_VDEC_DBG_HEVC_RESET) {
		if (!hevc_powered) return ENXIO;
		hakios_hevc_reset_core();
		return 0;
	}
	if (command == TWL_VDEC_DBG_FW) {
		struct twl_vdec_dbg_fw *f = (struct twl_vdec_dbg_fw *)data;
		f->result = hakios_optee_load_video_fw(f->id, f->core, f->swap != 0);
		return 0;
	}
	return ENOTTY;
}

static int
vdec_probe_ioctl_locked(u_long command, caddr_t data)
{
	if (IOCGROUP(command) == 'V' && (command & 0xff) >= 16 && (command & 0xff) <= 20) {
		return vdec_debug_ioctl(command, data);
	}
	if (command == TWL_VDEC_GET_CAPS) {
		struct twl_vdec_caps caps = { TWL_VDEC_ABI, TWL_VDEC_PROBE_ONLY, 0x61766331, 64, 64, 0x34323076 };
		if (test_level == 7) {
			/* width and height: the largest picture a slot holds, either way round */
			caps.flags |= TWL_VDEC_STATEFUL_PROBE | (hevc_powered ? TWL_VDEC_HEVC | TWL_VDEC_VP9 : 0);
			caps.width = pool_fits(4096, 2304) ? 4096 : pool_fits(3840, 2176) ? 3840 : 1920;
			caps.height = caps.width == 4096 ? 2304 : caps.width == 3840 ? 2176 : 1088;
		}
		memcpy(data, &caps, sizeof(caps));
		return 0;
	}
	if (command == TWL_HEVC_PREPARE || command == TWL_HEVC_SLICE || command == TWL_VP9_PREPARE ||
	    command == TWL_VP9_FRAME) return hevc_ioctl(command, data);
	if (command == TWL_VDEC_PREPARE) {
		if (core_codec != CODEC_H264) h264_probe_cancel();
		return h264_probe_prepare((const void *)data);
	}
	if (command == TWL_VDEC_FINISH) return h264_probe_finish((void *)data);
	if (command == TWL_VDEC_NEXT) return h264_probe_next((const void *)data);
	if (command == TWL_VDEC_CANCEL) { h264_probe_cancel(); return 0; }
	if (command == TWL_VDEC_STREAM) {
		if (test_level != 7 || !stateful_decode) return EINVAL;
		struct twl_vdec_stream stream = { slot_count, stream_mb_width * 16, stream_rows, stream_stride, stream_rows,
			0, 0, 0, 0 };
		unsigned slots = slot_count;
		if (core_codec == CODEC_HEVC &&
		    !hakios_hevc_stream(&stream.slots, &stream.width, &stream.height, &stream.stride, &stream.rows))
			return EINVAL;
		if (core_codec == CODEC_VP9 &&
		    !hakios_vp9_stream(&stream.slots, &stream.width, &stream.height, &stream.stride, &stream.rows))
			return EINVAL;
		if (core_codec != CODEC_H264) slots = stream.slots;
		if (core_codec == CODEC_HEVC) stream.transfer = hakios_hevc_transfer();
		uint64_t second;
		nanoseconds_to_absolutetime(NSEC_PER_SEC, &second);
		uint64_t now = mach_absolute_time();
		/* holds as soon as a player asks for pictures (else they are decoded over before it does: no picture would
		 * ever be shown); no copies only once the plane has shown some, and only with kern.hakios_vdec_plane_only:
		 * the window server composites the video itself whenever the interface over it changes (a spinner, the
		 * player's controls), and a pixel buffer left empty was a black frame there */
		stream.plane = plane_only && plane_seen && now - plane_seen < second;
		stream.held = hakios_vdec_held(slots);
		/* everything the plane may read: the picture on it, the one waiting, those handed to it and the one before (still
		 * scanned for a frame); without that last one the next picture went into it nearly every time */
		stream.showing = hakios_vdec_showing(slots);
		memcpy(data, &stream, sizeof(stream));
		return 0;
	}
	if (command != TWL_VDEC_DECODE) return ENOTTY;
	if (stateful_decode || header_pending) return EBUSY;
	const struct twl_vdec_packet *packet = (const struct twl_vdec_packet *)data;
	if (!packet->input_size || packet->input_size > TEST_INPUT_SIZE - 1024 || packet->output_size < 6144)
		return EINVAL;
	uint8_t *input = kalloc_data(packet->input_size, Z_WAITOK);
	uint8_t *output = kalloc_data(6144, Z_WAITOK | Z_ZERO);
	int error = ENOMEM;
	if (input && output) {
		error = copyin((user_addr_t)packet->input, input, packet->input_size);
		if (!error) {
			uint32_t hash = 2166136261u;
			for (unsigned i = 0; i < packet->input_size; i++) hash = (hash ^ input[i]) * 16777619u;
			vdec_log("vdec: probe input %u bytes hash %#x\n", packet->input_size, hash);
			error = h264_probe_load();
			if (!error) error = h264_decode64(input, packet->input_size, output);
			else error = EIO;
			if (!error) error = copyout(output, (user_addr_t)packet->output, 6144);
		}
	}
	if (input) kfree_data(input, packet->input_size);
	if (output) kfree_data(output, 6144);
	return error;
}

static int
vdec_probe_ioctl(__unused dev_t dev, u_long command, caddr_t data, __unused int flags, struct proc *p)
{
	lck_mtx_lock(&vdec_probe_lock);
	int error = probe_owner == proc_pid(p) ? vdec_probe_ioctl_locked(command, data) : EBADF;
	lck_mtx_unlock(&vdec_probe_lock);
	return error;
}

static const struct cdevsw vdec_probe_cdevsw = {
	.d_open = vdec_probe_open, .d_close = vdec_probe_close,
	.d_read = eno_rdwrt, .d_write = eno_rdwrt, .d_ioctl = vdec_probe_ioctl,
	.d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

static void
vdec_probe_publish(void)
{
	if (probe_published || !test_pool) return;
	int major = cdevsw_add(-1, &vdec_probe_cdevsw);
	if (major >= 0 && devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666,
	    "twl-vdec-probe") != NULL) {
		probe_published = true;
		if (test_level == 7) hakios_vplane_publish();
		vdec_log("vdec: /dev/twl-vdec-probe: H.264 %s transport ready\n",
		    test_level == 7 ? "reference-picture probe" : "I-picture probe");
	} else {
		if (major >= 0) cdevsw_remove(major, &vdec_probe_cdevsw);
		vdec_log("vdec: could not publish the decoder probe\n");
	}
}

static void
vdec_bringup(void)
{
	uint32_t gate;
	bool vdec_on, hevc_on;

	clk = ml_io_map(CLK_BASE, 0x1000);
	dos = ml_io_map(DOS_BASE, DOS_SIZE);
	reset = ml_io_map(RESET_BASE, 0x1000);
	if (clk == 0 || dos == 0 || reset == 0) {
		vdec_log("vdec: cannot map the registers\n");
		return;
	}
	gate = clk_rd(SYS_CLK_EN0_REG0);
	vdec_log("vdec: DOS bus clock %s (SYS_CLK_EN0_REG0 %#x); clocks before: VDEC %#x VDEC2 %#x VDEC3 %#x VDEC4 %#x\n",
	    (gate & SYS_CLK_DOS) ? "open" : "shut", gate, clk_rd(VDEC_CLK_CTRL), clk_rd(VDEC2_CLK_CTRL),
	    clk_rd(VDEC3_CLK_CTRL), clk_rd(VDEC4_CLK_CTRL));
	if (!(gate & SYS_CLK_DOS)) {
		clk_wr(SYS_CLK_EN0_REG0, gate | SYS_CLK_DOS);
	}

	core_clock(VDEC_CLK_CTRL, VDEC3_CLK_CTRL);
	core_clock(VDEC2_CLK_CTRL, VDEC4_CLK_CTRL);
	vdec_log("vdec: clocks after: VDEC %#x VDEC2 %#x VDEC3 %#x VDEC4 %#x\n", clk_rd(VDEC_CLK_CTRL),
	    clk_rd(VDEC2_CLK_CTRL), clk_rd(VDEC3_CLK_CTRL), clk_rd(VDEC4_CLK_CTRL));

	vdec_on = power_on(PD_VDEC, "VDEC");
	vdec_powered = vdec_on;
	hevc_on = power_on(PD_HEVC, "HEVC");
	hevc_powered = hevc_on;
	if (!vdec_on && !hevc_on) {
		vdec_log("vdec: neither core powered up; the DOS registers are left alone\n");
		return;
	}
	/* Power on does not release the normal-world APB and DMC bridge resets. */
	volatile uint32_t *r1 = (volatile uint32_t *)(reset + RESET1_LEVEL);
	volatile uint32_t *r5 = (volatile uint32_t *)(reset + RESET5_LEVEL);
	vdec_log("vdec: reset levels before DOS %#x bridge %#x\n", *r1, *r5);
	*r1 |= RESET_DOS_CAPB3 | RESET_DOS;
	*r5 |= (vdec_on ? RESET_BRG_VDEC : 0) | (hevc_on ? RESET_BRG_HEVCF : 0);
	__builtin_arm_dsb(0xf);
	IODelay(20);
	vdec_log("vdec: reset levels after DOS %#x bridge %#x\n", *r1, *r5);
	show_dos("powered");

	/* dos_local_config(): memories on, each core held in reset for 20 us; then its clock gates opened */
	if (vdec_on) {
		dos_wr(DOS_MEM_PD_VDEC, 0);
		dos_wr(DOS_SW_RESET0, 0xfffffffcu);
		IODelay(20);
		dos_wr(DOS_SW_RESET0, 0);
		IODelay(10);
		dos_wr(DOS_MEM_PD_VDEC, 0);
		dos_wr(DOS_GCLK_EN0, 0xffffffffu);
	}
	if (hevc_on) {
		dos_wr(DOS_MEM_PD_HEVC, 0);
		dos_wr(DOS_SW_RESET3, 0xffffffffu);
		IODelay(20);
		dos_wr(DOS_SW_RESET3, 0);
		IODelay(10);
		dos_wr(DOS_GCLK_EN3, 0xffffffffu);
	}
	IODelay(10);
	show_dos("reset, clocks open");
	if (vdec_on) {
		static const uint32_t r[] = { MPSR, CPSR, IMEM_DMA_CTRL };
		static const char *const n[] = { "MPSR", "CPSR", "IMEM_DMA_CTRL" };

		show_core("VDEC", r, n, 3);
		scratch_test(AV_SCRATCH_0, "VDEC");
	}
	if (hevc_on) {
		static const uint32_t r[] = { HEVC_PARSER_VERSION, HEVC_MPSR, HEVC_CPSR };
		static const char *const n[] = { "HEVC_PARSER_VERSION", "HEVC_MPSR", "HEVC_CPSR" };

		show_core("HEVC", r, n, 3);
		scratch_test(HEVC_ASSIST_SCRATCH_0, "HEVC");
	}
	vdec_log("vdec: bring-up done\n");
	if (test_level >= 2) {
		bool h264_loaded = false;
		/* Firmware loading is separate from starting a core: no DMA runs without a decoder context. */
		for (unsigned waited = 0; !hakios_optee_video_fw_ready() && waited < 100; waited++) {
			IOSleep(100);
		}
		if (!hakios_optee_video_fw_ready()) {
			vdec_log("vdec: firmware package is not ready; core loads skipped\n");
			return;
		}
		if (vdec_on) {
			int result = hakios_optee_load_video_fw(HAKIOS_VIDEO_DEC_H264_MULTI, HAKIOS_OPTEE_VDEC_LEGACY, false);
			vdec_log("vdec: H.264 firmware load: %d, MPSR %#x DMA %#x\n", result, dos_rd(MPSR), dos_rd(IMEM_DMA_CTRL));
			h264_loaded = result == 0;
		}
		if (hevc_on) {
			int result = hakios_optee_load_video_fw(HAKIOS_VIDEO_DEC_HEVC, HAKIOS_OPTEE_VDEC_HEVC, true);
			vdec_log("vdec: HEVC firmware load: %d, MPSR %#x\n", result, dos_rd(HEVC_MPSR));
			result = hakios_optee_load_video_fw(HAKIOS_VIDEO_DEC_VP9_MMU, HAKIOS_OPTEE_VDEC_HEVC, false);
			vdec_log("vdec: VP9 MMU firmware load: %d, MPSR %#x\n", result, dos_rd(HEVC_MPSR));
		}
		if (h264_loaded && test_level >= 3 && test_level <= 5) h264_input_test();
		if (h264_loaded && (test_level == 6 || test_level == 7)) vdec_probe_publish();
	}
}

/* Called after the other CPUs have stopped, before the loader can reuse any DMA memory. */
void
hakios_vdec_quiesce(void)
{
	hakios_wdt_stop();      /* the loader's fastboot does not feed it */
	if (!dos) return;
	if (vdec_powered) {
		dos_wr(MPSR, 0);
		dos_wr(CPSR, 0);
		dos_wr(ASSIST_MBOX1_MASK, 0);
		for (unsigned i = 0; i < 10000 &&
		    ((dos_rd(IMEM_DMA_CTRL) | dos_rd(LMEM_DMA_CTRL)) & 0x8000); i++) IODelay(10);
		dos_wr(DOS_SW_RESET0, 0xfffffffcu);
	}
	if (hevc_powered) {
		dos_wr(HEVC_MPSR, 0);
		dos_wr(HEVC_CPSR, 0);
		dos_wr(DOS_SW_RESET3, 0xffffffffu);
	}
	__builtin_arm_dsb(0xf);
	IODelay(100);
	if (vdec_powered) (void)sip(SIP_PWRC_SET, PD_VDEC, 0, 0);
	if (hevc_powered) (void)sip(SIP_PWRC_SET, PD_HEVC, 0, 0);
	if (vdec_powered || hevc_powered) {
		char line[128];

		snprintf(line, sizeof(line), "vdec: quiesced: power states VDEC %#llx HEVC %#llx (1 = off)\n",
		    sip(SIP_PWRC_GET, PD_VDEC, 0, 0), sip(SIP_PWRC_GET, PD_HEVC, 0, 0));
		loader_log_line(line); /* The shutdown path must not acquire a console lock. */
	}
	vdec_powered = hevc_powered = false;
}

static void
vdec_later(void *arg, wait_result_t wr)
{
	(void)wr;
	IOSleep((unsigned int)(uintptr_t)arg * 1000);
	vdec_bringup();
	if (test_level == 5) {
		for (unsigned i = 1; i < 3; i++) {
			hakios_vdec_quiesce();
			IOSleep(20);
			vdec_log("vdec: decoder restart %u/2\n", i);
			vdec_bringup();
		}
	}
}

void
hakios_vdec_init(void)
{
	unsigned int on = 0, delay = 0;
	thread_t thread;

	hakios_wdt_init();      /* not the decoder's, but this is the box's late init and its way back (quiesce) */
	hakios_amlcpu_init();

	if (!PE_parse_boot_argn("vdec", &on, sizeof(on)) || on == 0) {
		return;
	}
	test_level = on;
	(void)PE_parse_boot_argn("vdec_rpm", &dump_rpm, sizeof(dump_rpm));
	uint32_t slow = 0;
	if (PE_parse_boot_argn("vdec_667", &slow, sizeof(slow)) && slow) clk_src = CLK_SRC_FCLK_DIV3;
	(void)PE_parse_boot_argn("vdec_irq", &vdec_irq, sizeof(vdec_irq));
	(void)PE_parse_boot_argn("vdec_udebug", &ucode_debug, sizeof(ucode_debug));
	(void)PE_parse_boot_argn("vdec_debug", &debug_access, sizeof(debug_access));
	/* Physically 1 MiB aligned (pnum_mask): with a pool that is not 64 KiB aligned (0x...54000) the
	 * firmware loses its context from the third picture of a stream on. */
	unsigned pool_mb = POOL_MB_DEFAULT;
	(void)PE_parse_boot_argn("vdec_pool_mb", &pool_mb, sizeof(pool_mb));
	if (pool_mb < 32) pool_mb = 32;
	if (pool_mb > 512) pool_mb = 512;
	test_pool_size = pool_mb << 20;
	/* past the pool: the video plane's copies of a window server's video layers (hakios_vdec_copy_memory) */
	unsigned copy_mb = COPY_MB_DEFAULT;
	(void)PE_parse_boot_argn("vplane_copy_mb", &copy_mb, sizeof(copy_mb));
	copy_size = copy_mb > 64 ? 64u << 20 : copy_mb << 20;
	if (on >= 3 && kmem_alloc_contig(kernel_map, &test_pool, TEST_POOL_SIZE + copy_size, PAGE_MASK,
	    (ppnum_t)(0xffffffffull >> PAGE_SHIFT), (ppnum_t)((1u << 20) / PAGE_SIZE - 1), KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) == KERN_SUCCESS) {
		test_pool_pa = kvtophys(test_pool);
		vdec_log("vdec: decoder pool %u MiB at %#llx, then %u MiB for the video plane's copies\n", pool_mb, test_pool_pa,
		    copy_size >> 20);
	}
	(void)PE_parse_boot_argn("vdec_delay", &delay, sizeof(delay));
	if (delay == 0 && on == 1) {
		vdec_bringup();
		return;
	}
	if (kernel_thread_start(vdec_later, (void *)(uintptr_t)delay, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		vdec_log("vdec: no thread for the delayed bring-up\n");
	}
}

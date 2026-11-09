/* An H.264 transport for testing the tvOS VideoToolbox bridge. */
#ifndef _SYS_HAKIOS_VDEC_H_
#define _SYS_HAKIOS_VDEC_H_
#include <stdint.h>
#include <sys/ioccom.h>
#define TWL_VDEC_ABI 3u
#define TWL_VDEC_PROBE_ONLY 1u
#define TWL_VDEC_STATEFUL_PROBE 2u
#define TWL_H264_SLOTS 24u
#define TWL_H264_RPM_WORDS 1024u
struct twl_vdec_caps {
    uint32_t abi, flags, codec, width, height, pixel_format;
};
struct twl_vdec_packet {
    uint64_t input, output;
    uint32_t input_size, output_size;
};
#define TWL_VDEC_GET_CAPS _IOR('V', 0, struct twl_vdec_caps)
#define TWL_VDEC_DECODE _IOW('V', 1, struct twl_vdec_packet)
/* vdec=7 only: the firmware stops at the slice header while user space manages its DPB.
 * Buffer references are logical slots; user space cannot supply DMA addresses/registers.
 */
struct twl_h264_prepare {
    uint64_t input, rpm;
    uint32_t input_size, rpm_size;
};
struct twl_h264_slot {
    int32_t top_poc, bottom_poc;
    uint32_t long_term;
};
struct twl_h264_config {
    uint32_t current_slot;
    int32_t mv_write_slot, mv_read_slot;
    uint32_t direct_8x8, first_mb, mv_read_bottom;
    int32_t frame_poc, top_poc, bottom_poc;
    uint32_t reference_count[2], references[2][32];
    struct twl_h264_slot slots[TWL_H264_SLOTS];
    uint64_t output;
    uint32_t output_size, flags;
    /* FINISH fails with EINPROGRESS when the picture has another slice: its parameters are copied here,
     * and the next FINISH (TWL_H264_SLICE set) configures that slice. */
    uint64_t rpm;
    /* out: the picture's generation in its slot, for the video plane (struct twl_vplane_token) */
    uint32_t generation, reserved;
};
#define TWL_H264_SLICE 1u   /* flags: a further slice of the picture being decoded */
#define TWL_H264_NOCOPY 2u  /* flags: do not copy the picture out (the video plane shows it from its slot) */
#define TWL_VDEC_PREPARE _IOW('V', 2, struct twl_h264_prepare)
#define TWL_VDEC_FINISH _IOWR('V', 3, struct twl_h264_config)
#define TWL_VDEC_CANCEL _IO('V', 4)
/* Diagnostic: consume the next picture already present in the current input FIFO. */
#define TWL_VDEC_NEXT _IOW('V', 5, struct twl_h264_prepare)
/* After the first PREPARE of a stream: how many slots its pictures get (the DPB must use no others), the coded
 * picture's size and the kernel frame's layout (luma stride x rows, then interleaved chroma). */
struct twl_vdec_stream {
    uint32_t slots, width, height, stride, rows;
    uint32_t held;      /* slots the video plane shows or may still be asked to: decode into them last */
    uint32_t plane;     /* nonzero: the video plane showed one of this decoder's pictures in the last second */
    uint32_t showing;   /* slots on the video plane now or at the next vsync: never decode into them */
    uint32_t transfer;  /* the bitstream's transfer characteristics (H.273: 16 PQ, 18 HLG; HEVC VUI), 0: not said */
};
#define TWL_VDEC_STREAM _IOR('V', 6, struct twl_vdec_stream)

/*
 * HEVC (caps flag TWL_VDEC_HEVC, vdec=7): the same split as H.264. PREPARE feeds one access unit (Annex B; the
 * VPS/SPS/PPS once, before the first) and stops at its first slice header: the firmware's parse of it (RPM, 128
 * words, struct union in tvos/video/hevc) is copied out. User space keeps the DPB (POC, reference sets, which slot
 * each picture goes into) and answers with SLICE: logical slots and POCs only; the kernel derives every address
 * and every other register from its own copy of the header. SLICE fails with EINPROGRESS and the next slice's
 * header when the picture has another slice, returns 0 when the picture is done, and ENODATA from PREPARE means
 * the unit had no slice (parameter sets or SEI only). TWL_VDEC_STREAM, CANCEL and the video plane's tokens work
 * as for H.264; the frame is 8-bit NV12 (10-bit streams rounded), luma stride x rows then interleaved chroma.
 */
#define TWL_VDEC_HEVC 4u
#define TWL_HEVC_RPM_WORDS 128u
#define TWL_HEVC_SLOTS 24u
struct twl_hevc_slice {
    uint32_t slot;              /* the picture's slot (all its slices) */
    int32_t poc;
    uint32_t flags;
    int32_t col_slot, col_poc;  /* the co-located picture (TMVP); col_slot -1: none */
    uint32_t ref_num[2];
    int32_t ref_poc[2][16];
    uint32_t ref_slot[2][16];
    uint32_t long_term;         /* bit i: L0 entry i is long-term; bit 16 + i: L1 entry i */
    uint64_t output, rpm;
    uint32_t output_size;
    uint32_t generation;        /* out: the picture's generation in its slot (struct twl_vplane_token) */
    uint32_t referenced;  /* bit 31 and the slots a reference still names: the others' compressed bodies can go */
};
#define TWL_HEVC_SKIP 1u        /* flags: not decoded (a RASL picture without its references) */
#define TWL_HEVC_NOCOPY 2u      /* flags: as TWL_H264_NOCOPY */
#define TWL_HEVC_PREPARE _IOW('V', 8, struct twl_h264_prepare)
#define TWL_HEVC_SLICE _IOWR('V', 9, struct twl_hevc_slice)

/*
 * VP9 (caps flag TWL_VDEC_VP9, vdec=7), Profile 0 and 2 (8 and 10 bits, 4:2:0) on the HEVC core. PREPARE feeds ONE
 * frame (a superframe is split by the caller, its index left out) and stops at its header: the firmware's parse
 * (RPM, 128 words, struct in tvos/video/vp9) is copied out. User space keeps the reference map (which slot each of
 * the eight references is, which slot a frame goes into, refresh_frame_flags) and answers with FRAME: the frame's
 * slot and the slots of its LAST, GOLDEN and ALTREF references. The kernel keeps the rest: sizes taken from a
 * reference, motion vectors of the frame before, the probability contexts and their adaptation, segmentation.
 * FRAME returns 0 with the picture done (copied out unless NOCOPY; the frame's size in width/height). SKIP: the
 * frame is not decoded (before the first keyframe); SKIP | SHOW: show_existing_frame, the picture in `slot` is
 * copied out as it is. PREPARE's ENODATA: the data held no frame. A larger frame than the stream's first: ESTALE
 * (start again: CANCEL, then a keyframe).
 */
#define TWL_VDEC_VP9 8u
#define TWL_VP9_RPM_WORDS 128u
#define TWL_VP9_SLOTS 24u
struct twl_vp9_frame {
    uint32_t slot;              /* the frame's slot; with SHOW, the picture shown */
    uint32_t ref_slot[3];       /* LAST, GOLDEN, ALTREF (inter frames) */
    uint32_t flags;
    uint32_t width, height;     /* out: the frame's size */
    uint32_t output_size;
    uint64_t output;
    uint32_t generation;        /* out: as twl_hevc_slice's */
    uint32_t referenced;  /* bit 31 and the slots a reference still names: the others' compressed bodies can go */
};
#define TWL_VP9_SKIP 1u
#define TWL_VP9_NOCOPY 2u
#define TWL_VP9_SHOW 4u
#define TWL_VP9_QUARTER 8u      /* above 1440p the copy a quarter of the size each way (a player that names no size) */
#define TWL_VP9_PREPARE _IOW('V', 10, struct twl_h264_prepare)
#define TWL_VP9_FRAME _IOWR('V', 11, struct twl_vp9_frame)

/*
 * Bring-up only (boot-arg vdec_debug=1, never on a normal image): the DOS registers, the decoder pool and firmware
 * loads from user space, on the probe node, for developing a core's sequence without a kernel per experiment. A
 * caller can make the decoder DMA anywhere: it is a development hole by design.
 */
struct twl_vdec_dbg_reg {
    uint32_t reg, value, write, reserved;   /* reg: DOS word index (0..0x3fff) */
};
struct twl_vdec_dbg_mem {
    uint64_t buffer;
    uint32_t offset, size, write, reserved; /* offset into the pool; cache maintenance included */
};
struct twl_vdec_dbg_info {
    uint64_t pool_pa;
    uint32_t pool_size, hevc_powered;
};
struct twl_vdec_dbg_fw {
    uint32_t id, core, swap;
    int32_t result;
};
#define TWL_VDEC_DBG_REG _IOWR('V', 16, struct twl_vdec_dbg_reg)
#define TWL_VDEC_DBG_MEM _IOW('V', 17, struct twl_vdec_dbg_mem)
#define TWL_VDEC_DBG_INFO _IOR('V', 18, struct twl_vdec_dbg_info)
#define TWL_VDEC_DBG_FW _IOWR('V', 19, struct twl_vdec_dbg_fw)
#define TWL_VDEC_DBG_HEVC_RESET _IO('V', 20)   /* the HEVC core's reset between pictures (vdec.c hevc_reset_core) */

/*
 * The video plane (/dev/twl-vplane): a decoded picture shown by the VPU under the OSD, at the output's resolution,
 * where the OSD has the key colour (TWL_VPLANE_KEY_RGB, made transparent by the OSD's colour key). The decoder
 * tags each picture's IOSurface with its token (IOSurface value TWL_VPLANE_TOKEN_KEY, the struct as CFData); a
 * player that draws the key colour where the picture goes asks for the picture with SHOW.
 */
#define TWL_VPLANE_MAGIC 0x504c5754u    /* "TWLP" */
#define TWL_VPLANE_TOKEN_KEY "TwlVideoPlanePicture"
#define TWL_VPLANE_KEY_RGB 0xff00ffu   /* what Y 60, Cb 255, Cr 255 becomes in any BT.601/709 conversion, clamped */
struct twl_vplane_token {
    uint32_t magic, slot, generation, width, height, flags;
};
#define TWL_VPLANE_PQ 1u        /* flags: the picture is PQ (HDR10): an HDR10 output shows it as it is */
#define TWL_VPLANE_SHOW _IOW('P', 1, struct twl_vplane_token)
#define TWL_VPLANE_HIDE _IO('P', 2)
#define TWL_VPLANE_WAIT _IO('P', 3)    /* until the plane's next refresh (at once with it off) */
#endif

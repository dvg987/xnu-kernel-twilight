/*
 * A minimal OP-TEE client for the Amlogic box (boot-arg optee=N, off by default): the secure world's
 * identity and capabilities, and the video decoders' firmware handed to the video TA and loaded into
 * the cores. See drivers/gpu/hakios.optee.c.
 */
#ifndef _SYS_HAKIOS_OPTEE_H_
#define _SYS_HAKIOS_OPTEE_H_

#include <stdbool.h>
#include <stdint.h>

void hakios_optee_init(void);

/* the firmware package went to the video TA (optee=2): the cores can be loaded */
bool hakios_optee_video_fw_ready(void);

/*
 * One core's microcode, from the package the TA holds into the core: SMC 0xB200000F (index, core, swap),
 * Amlogic's TEE_SMC_LOAD_VIDEO_FW (media_modules amvdec.c optee_load_fw). The core's power domain and
 * clocks must be on first: the secure world writes its registers. Returns the SMC's x0 (negative: failed),
 * or -1 without the call when optee=0.
 */
int hakios_optee_load_video_fw(uint32_t index, uint32_t core, bool swap);

/* index: media_modules firmware_type.h */
#define HAKIOS_FW_TAG(a, b, c, d)       (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (uint32_t)(d))
#define HAKIOS_VIDEO_DEC_H264_MULTI     15u     /* "mh264": core VDEC_LEGACY, no swap */
#define HAKIOS_VIDEO_DEC_HEVC           16u
#define HAKIOS_VIDEO_DEC_HEVC_MMU       17u
#define HAKIOS_VIDEO_DEC_VP9_MMU        19u
#define HAKIOS_VIDEO_DEC_H264_MULTI_MMU 23u
#define HAKIOS_VIDEO_DEC_HEVC_MMU_SWAP  HAKIOS_FW_TAG('2', '6', '5', 'S')      /* "hevc_mmu_swap": core HEVC, swap */
#define HAKIOS_VIDEO_DEC_AV1_MMU        HAKIOS_FW_TAG('A', 'V', '1', 'M')

/* core: OPTEE_VDEC_* */
#define HAKIOS_OPTEE_VDEC_LEGACY        0u      /* the VDEC core (Amlogic's OPTEE_VDEC_LEGENCY) */
#define HAKIOS_OPTEE_VDEC               1u
#define HAKIOS_OPTEE_VDEC_HEVC          2u
#define HAKIOS_OPTEE_VDEC_HCODEC        3u
#define HAKIOS_OPTEE_VDEC_HEVCB         4u

#endif

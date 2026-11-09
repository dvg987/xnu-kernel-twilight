/* Between the H.264 decoder (hakios.vdec.c) and the video plane (hakios.vplane.c), inside the kernel. */
#ifndef _SYS_HAKIOS_VPLANE_H_
#define _SYS_HAKIOS_VPLANE_H_
#include <stdint.h>
#include <mach/vm_types.h>

struct hakios_vdec_picture {
	uint64_t luma, chroma;          /* physical: NV12, luma stride x rows, then interleaved chroma */
	uint32_t stride, rows, width;   /* the coded picture (the token says how much of it is shown) */
	uint64_t compressed;            /* physical: the decoder's compressed picture (its header; the body in pages
	                                 * the header names), 0: none (H.264) */
	uint32_t depth;                 /* the compressed picture's bits per sample, 8 or 10 */
	uint32_t coded_width, coded_height;     /* the compressed picture's size (the NV12 copy can be smaller) */
};
int hakios_vdec_plane_picture(uint32_t slot, uint32_t generation, struct hakios_vdec_picture *out);
void hakios_vdec_plane_showing(int slot, int pending);
int hakios_vdec_scratch(uint32_t size, vm_offset_t *va, uint64_t *pa);
/* the decoder's stream is gone: stop showing it, at once */
void hakios_vplane_stop(void);
/* /dev/twl-vplane, published with the decoder's transport */
void hakios_vplane_publish(void);
/* MiniIOMobileFramebuffer: a frame about to be scanned (32-bit, the kernel's view of its pixels) */
void hakios_vplane_osd_frame(uint8_t *pixels, uint32_t row_bytes, uint32_t width, uint32_t height);
/*
 * MiniIOMobileFramebuffer: a swap's video layer (an NV12 surface the window server hands to the display rather than
 * drawing it, AVSampleBufferDisplayLayer's frames), shown by the plane under the window server's frame, which is
 * transparent where the picture is to be seen. Either a picture of the decoder's (its token, no copy) or the planes'
 * pixels, which are copied. NULL: the swaps have no video layer any more.
 */
struct hakios_vplane_surface {
	const uint8_t *luma, *chroma;   /* the kernel's view, at the top left of what is shown */
	uint32_t luma_stride, chroma_stride;
	uint32_t width, height;         /* what is shown of the surface */
	const void *token;              /* struct twl_vplane_token (the surface's TWL_VPLANE_TOKEN_KEY), or NULL */
	uint32_t x, y, w, h;            /* where, in the window server's frame (OSD1's 1920 x 1080) */
};
void hakios_vplane_surface(const struct hakios_vplane_surface *surface);
/* the decoder's pool's companion: contiguous memory for the copies of those surfaces */
int hakios_vdec_copy_memory(vm_offset_t *va, uint64_t *pa, uint32_t *size);
#endif

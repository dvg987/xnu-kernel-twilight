/* Between the decoder node (hakios.vdec.c) and the VP9 decoder (hakios.vp9.c, hakios.vp9_prob.c), inside the kernel. */
#ifndef _SYS_HAKIOS_VP9_H_
#define _SYS_HAKIOS_VP9_H_
#include <stdint.h>
#include <stdbool.h>
#include <sys/hakios.hevc.h>

/* hakios.vp9.c; all called with the decoder node's lock held */
int hakios_vp9_prepare(const struct twl_h264_prepare *request);
int hakios_vp9_frame(struct twl_vp9_frame *frame);     /* 0: the picture in frame->slot is done */
void hakios_vp9_cancel(void);
bool hakios_vp9_active(void);
bool hakios_vp9_shown(void);
uint32_t hakios_vdec_showing(unsigned slots);           /* hakios.vdec.c: slots the plane reads */
uint32_t hakios_vdec_irq_seq(void);                      /* hakios.vdec.c: the core's interrupt, before a look */
bool hakios_vdec_irq_nap(uint32_t seq, unsigned us);    /* sleep until the next one (or us); false without it */
uint32_t hakios_vdec_held(unsigned slots);              /* hakios.vdec.c: slots a player or the plane may still show */                            /* the frame just decoded is one to show */
bool hakios_vp9_stream(uint32_t *slots, uint32_t *width, uint32_t *height, uint32_t *stride, uint32_t *rows);
bool hakios_vp9_picture(uint32_t slot, struct hakios_vdec_picture *out);

/* hakios.vp9_prob.c: prev_prob (the context the frame started from), cur_prob (the frame's), count (its symbols) */
#define HAKIOS_VP9_PROB_SIZE    (496 * 2 * 4)
void hakios_vp9_adapt_probs(int pic_count, int prev_kf, int cur_kf, int pre_fc, unsigned int *prev_prob,
    unsigned int *cur_prob, unsigned int *count);
#endif

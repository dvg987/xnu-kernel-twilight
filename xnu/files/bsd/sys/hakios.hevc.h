/* Between the decoder node (hakios.vdec.c: the pool, the DOS block, the HEVC core's reset) and the HEVC decoder
 * (hakios.hevc.c), inside the kernel. */
#ifndef _SYS_HAKIOS_HEVC_H_
#define _SYS_HAKIOS_HEVC_H_
#include <stdint.h>
#include <stdbool.h>
#include <sys/hakios.vdec.h>
#include <sys/hakios.vplane.h>

/* hakios.vdec.c */
uint32_t hakios_dos_rd(uint32_t reg);
void hakios_dos_wr(uint32_t reg, uint32_t value);
void hakios_hevc_reset_core(void);
bool hakios_vdec_pool(vm_offset_t *va, uint64_t *pa, uint32_t *size);
uint32_t hakios_vdec_showing(unsigned slots);           /* hakios.vdec.c: slots the plane reads */
uint32_t hakios_hevc_transfer(void);                     /* hakios.hevc.c: the VUI's transfer characteristics, 0: none */
void hakios_vdec_slot_wait(unsigned slot, unsigned slots);    /* hakios.vdec.c: until the plane lets the slot go */
uint32_t hakios_vdec_irq_seq(void);                      /* hakios.vdec.c: the core's interrupt, before a look */
bool hakios_vdec_irq_nap(uint32_t seq, unsigned us);    /* sleep until the next one (or us); false without it */
uint32_t hakios_vdec_held(unsigned slots);              /* slots a player or the plane may still show */

/* hakios.hevc.c; all called with the decoder node's lock held */
int hakios_hevc_prepare(const struct twl_h264_prepare *request);
int hakios_hevc_slice(struct twl_hevc_slice *slice);   /* 0: the picture in slice->slot is done */
void hakios_hevc_cancel(void);
bool hakios_hevc_active(void);
bool hakios_hevc_stream(uint32_t *slots, uint32_t *width, uint32_t *height, uint32_t *stride, uint32_t *rows);
bool hakios_hevc_picture(uint32_t slot, struct hakios_vdec_picture *out);
#endif

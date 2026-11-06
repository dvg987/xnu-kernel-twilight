/*
 * The Amlogic box's HDMI transmitter (drivers/gpu/hakios.hdmitx.c, boot-arg hdmitx=1).
 */
#ifndef _SYS_HAKIOS_HDMITX_H_
#define _SYS_HAKIOS_HDMITX_H_

#include <stdint.h>
#include <stdbool.h>

#define HAKIOS_HDMITX_EDID_MAX  1024            /* 8 blocks */
#define HAKIOS_HDMITX_TIMINGS_MAX 24
#define HAKIOS_HDMITX_COLORS    8

/* A timing the box can drive and the TV takes, as IOMobileFramebuffer's TimingElements want it */
struct hakios_hdmitx_timing {
	uint16_t id;                            /* 1.. */
	uint8_t vic;
	bool fractional;                        /* the 1/1.001 twin of the code's rate */
	uint16_t width, height, htotal, vtotal;
	uint32_t pixel_khz;
	uint32_t rate;                          /* refresh, 16.16 */
	uint32_t colors;                        /* bit n: colour element n + 1 */
};

/* A colour element (ColorElements), in IOAVFamily's numbers */
struct hakios_hdmitx_color {
	uint8_t id, encoding, depth, colorimetry, eotf, range;
};

#ifdef KERNEL
void hakios_hdmitx_init(void);                  /* what U-Boot left on the wire, and the TV's EDID */
const uint8_t *hakios_hdmitx_edid(unsigned *length);   /* NULL if none was read */
uint32_t hakios_hdmitx_physical_address(void);  /* the CEC physical address from the EDID, 0xffff if not known */
const struct hakios_hdmitx_timing *hakios_hdmitx_timings(unsigned *count);     /* NULL if the EDID gave none */
extern const struct hakios_hdmitx_color hakios_hdmitx_colors[HAKIOS_HDMITX_COLORS];
const char *hakios_hdmitx_tv(bool *pq, bool *hlg, bool *bt2020);           /* the TV's name and HDR */
#endif

#endif /* _SYS_HAKIOS_HDMITX_H_ */

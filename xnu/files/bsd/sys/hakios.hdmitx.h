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
#define HAKIOS_HDMITX_SADS      16              /* short audio descriptors kept */

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

/* A short audio descriptor of the sink (CTA-861 audio data block) */
struct hakios_hdmitx_sad {
	uint8_t format;                         /* 1 LPCM, 2 AC-3, 7 DTS, 10 E-AC-3, 11 DTS-HD, 12 MAT, 15 extension */
	uint8_t channels;                       /* the most */
	uint8_t rates;                          /* bit 0..6: 32, 44.1, 48, 88.2, 96, 176.4, 192 kHz */
	uint8_t extra;                          /* byte 3: LPCM bit depths; AC-3..ATRAC the bit rate / 8 kbit/s; E-AC-3 and MAT flags (bit 0: JOC / Atmos) */
};

/* What to send (hakios_hdmitx_audio_start): the audio block's I2S arrives at the transmitter as this */
struct hakios_hdmitx_audio {
	uint32_t rate;                          /* Hz, of the sample packets: 192000 for E-AC-3 in IEC 61937 and for HBR */
	uint8_t channels;                       /* 2, 4, 6 or 8 */
	uint8_t hbr;                            /* high-bit-rate packets (MAT, TrueHD, DTS-HD MA): four I2S lanes */
	uint8_t nonaudio;                       /* the data is an IEC 61937 stream: channel status says "not audio" */
	uint8_t ca;                             /* infoframe channel allocation, 0xff: the usual one for `channels' */
	uint8_t spdif;                          /* the transmitter's S/PDIF input (AUD_MODE 2) instead of I2S: stereo, as the bootloader leaves it */
};

#ifdef KERNEL
void hakios_hdmitx_init(void);                  /* what U-Boot left on the wire, and the TV's EDID */
const uint8_t *hakios_hdmitx_edid(unsigned *length);   /* NULL if none was read */
uint32_t hakios_hdmitx_physical_address(void);  /* the CEC physical address from the EDID, 0xffff if not known */
const struct hakios_hdmitx_timing *hakios_hdmitx_timings(unsigned *count);     /* NULL if the EDID gave none */
extern const struct hakios_hdmitx_color hakios_hdmitx_colors[HAKIOS_HDMITX_COLORS];
/* audio (the sink's formats from the EDID, and the transmitter's audio side: used by hakios.hdmiaudio.c) */
const struct hakios_hdmitx_sad *hakios_hdmitx_sads(unsigned *count, uint8_t *speakers);
bool hakios_hdmitx_sink_takes(unsigned format, unsigned channels, unsigned rate_bit);
bool hakios_hdmitx_audio_bus_clock(bool open);     /* CLKCTRL sys_audio: the audio registers need it open */
bool hakios_hdmitx_audio_clock(void);              /* the transmitter's own audio clock on */
bool hakios_hdmitx_audio_start(const struct hakios_hdmitx_audio *a);      /* N/CTS, I2S input, channel status, infoframe; unmuted */
void hakios_hdmitx_audio_stop(void);               /* muted */
void hakios_hdmitx_audio_show(void);               /* its audio registers, in the log */
void hakios_hdmiaudio_init(void);                  /* hakios.hdmiaudio.c, boot-arg hdmiaudio=N: at the end of hakios_hdmitx_init() */
void hakios_vdec_init(void);                       /* hakios.vdec.c, boot-arg vdec=N: after hakios_hdmiaudio_init() */
const char *hakios_hdmitx_tv(bool *pq, bool *hlg, bool *bt2020);           /* the TV's name and HDR */
/* Settings (MiniIOMobileFramebuffer): the current timing/colour IDs, a choice (async mode set), a callback on hot plug and mode set */
void hakios_hdmitx_current(unsigned *timing_id, unsigned *color_id);
bool hakios_hdmitx_set_output(unsigned timing_id, unsigned color_id);
void hakios_hdmitx_set_listener(void (*fn)(void));
void hakios_hdmitx_osd_source(unsigned w, unsigned h, unsigned stride);  /* what the OSD scans (in the blanking) */
unsigned hakios_hdmitx_output_height(void);        /* the picture's height on the wire, 0: none */
uint64_t hakios_hdmitx_frame_ns(void);             /* a frame on the wire in ns, 0: none */
unsigned hakios_hdmitx_output_width(void);         /* and its width */
unsigned hakios_hdmitx_color_count(void);          /* colour elements Settings gets (SDR 4, or with HDR10 boot-arg hdmitx_hdrmodes=1) */
#endif

#endif /* _SYS_HAKIOS_HDMITX_H_ */

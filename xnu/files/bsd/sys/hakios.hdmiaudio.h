/*
 * /dev/twl-audio: the sound of the Amlogic box (S905X5M), out of its HDMI transmitter (drivers/audio/hakios.hdmiaudio.c,
 * boot-arg hdmiaudio=N). The audio server's HAL plug-in (tvos/src/twlaudio.c) writes what it has mixed here.
 *
 *   write()   whole frames of the format set last: `channels' samples each, `bits' (16 or 32) wide, little endian, signed.
 *             Never waits: what does not fit is dropped (the writer is a real-time thread).
 *   ioctl     TWL_AUDIO_SET_FORMAT   starts the path (clocks, FIFO, I2S, HDMI packets) for a format
 *             TWL_AUDIO_GET_POSITION frames the hardware has taken, and when (the device's clock for the HAL)
 *             TWL_AUDIO_GET_CAPS     what the sink (TV or AV receiver) says it plays (EDID)
 *             TWL_AUDIO_SET_VOLUME   a software gain on PCM, in 1/65536 (65536 = 0 dB); 0 mutes
 *             TWL_AUDIO_STOP         the path off
 */
#ifndef _SYS_HAKIOS_HDMIAUDIO_H_
#define _SYS_HAKIOS_HDMIAUDIO_H_

#include <stdint.h>
#include <sys/ioccom.h>

#define TWL_AUDIO_DEVICE        "/dev/twl-audio"

enum {
	TWL_AUDIO_PCM = 0,              /* linear PCM, 2 to 8 channels */
	TWL_AUDIO_IEC61937 = 1,         /* an IEC 61937 stream in a 2-channel carrier: AC-3 at 48 kHz, E-AC-3 at 192 kHz (4x) */
	TWL_AUDIO_HBR = 2,              /* IEC 61937 in high-bit-rate packets, 8 channels at 192 kHz: MAT (Atmos), TrueHD, DTS-HD MA */
};

struct twl_audio_format {
	uint32_t kind;                  /* TWL_AUDIO_* */
	uint32_t rate;                  /* Hz of the frames written: 32000..192000 */
	uint32_t channels;              /* 2, 4, 6 or 8 */
	uint32_t bits;                  /* 16 or 32 */
};

struct twl_audio_position {
	uint64_t frames;                /* played since the format was set */
	uint64_t mach_time;             /* mach_absolute_time() of that reading */
	uint32_t fill_frames;           /* written and not yet played */
	uint32_t underruns;             /* times the hardware ran out of data */
	uint32_t dropped_frames;        /* written while the ring was full */
	uint32_t running;
};

#define TWL_AUDIO_SADS 16
struct twl_audio_caps {
	uint32_t count;                 /* short audio descriptors */
	uint32_t speakers;              /* speaker allocation block, byte 1 */
	struct { uint8_t format, channels, rates, extra; } sad[TWL_AUDIO_SADS];     /* as in CTA-861 */
};

#define TWL_AUDIO_SET_FORMAT    _IOW('t', 1, struct twl_audio_format)
#define TWL_AUDIO_GET_POSITION  _IOR('t', 2, struct twl_audio_position)
#define TWL_AUDIO_GET_CAPS      _IOR('t', 3, struct twl_audio_caps)
#define TWL_AUDIO_SET_VOLUME    _IOW('t', 4, uint32_t)
#define TWL_AUDIO_STOP          _IO('t', 5)

#endif /* _SYS_HAKIOS_HDMIAUDIO_H_ */

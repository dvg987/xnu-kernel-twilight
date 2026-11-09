/*
 * HDMI-CEC on the Amlogic box: the TV's remote into /dev/hakios-touch (boot-arg cec=1).
 * See drivers/input/hakios.cec.c.
 */
#ifndef _SYS_HAKIOS_CEC_H_
#define _SYS_HAKIOS_CEC_H_

#include <stdint.h>

void hakios_cec_init(void);
/* from hakios.hdmitx.c: a TV was plugged in (again) and its EDID gave this physical address (0xffff: none) */
void hakios_cec_hotplug(uint32_t physical_address);
/* something for the CEC thread to send (sysctl kern.hakios_hdmitx_cmd 5, 6: for tests) */
#define HAKIOS_CEC_STANDBY              1       /* <Standby> to all */
#define HAKIOS_CEC_ONE_TOUCH_PLAY       2       /* <Image View On> to the TV, <Active Source> to all */
void hakios_cec_request(int what);

#endif

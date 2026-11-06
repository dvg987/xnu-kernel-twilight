/*
 * HDMI-CEC on the Amlogic box: the TV's remote into /dev/hakios-touch (boot-arg cec=1).
 * See drivers/input/hakios.cec.c.
 */
#ifndef _SYS_HAKIOS_CEC_H_
#define _SYS_HAKIOS_CEC_H_

void hakios_cec_init(void);

#endif

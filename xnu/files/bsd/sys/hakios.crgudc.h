/*
 * The Amlogic box's USB port as a device: keys from the host into /dev/hakios-touch (boot-arg usbkeys=1).
 * See drivers/input/hakios.crgudc.c.
 */
#ifndef _SYS_HAKIOS_CRGUDC_H_
#define _SYS_HAKIOS_CRGUDC_H_

void hakios_crgudc_init(void);

#endif

/*
 * A USB disk on the Amlogic box's outside port as block devices for its partitions (/dev/usb_darwin,
 * /dev/usb_darwin_data). Only with boot-arg usbdisk=1. See drivers/storage/hakios.usbdisk.c.
 */
#ifndef _SYS_HAKIOS_USBDISK_H_
#define _SYS_HAKIOS_USBDISK_H_

#include <stdbool.h>
#include <sys/types.h>

void hakios_usbdisk_init(void);
bool hakios_usbdisk_dev(dev_t dev);

#endif /* _SYS_HAKIOS_USBDISK_H_ */

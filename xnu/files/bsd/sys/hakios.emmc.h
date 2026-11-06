/*
 * The eMMC of the Amlogic box as block devices for its GPT partitions (/dev/emmc_darwin).
 * Only with boot-arg emmc=1 (read only) or emmc=2. See drivers/storage/hakios.emmc.c.
 */
#ifndef _SYS_HAKIOS_EMMC_H_
#define _SYS_HAKIOS_EMMC_H_

#include <stdbool.h>
#include <sys/types.h>

void hakios_emmc_init(void);
bool hakios_emmc_dev(dev_t dev);

#endif /* _SYS_HAKIOS_EMMC_H_ */

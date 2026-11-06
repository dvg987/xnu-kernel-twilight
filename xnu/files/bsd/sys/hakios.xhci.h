/*
 * The Amlogic box's internal USB host and the W2 Wi-Fi/Bluetooth chip on it (boot-arg w2=N).
 * See drivers/wifi/hakios.xhci.c (the bus, the chip's bring-up) and drivers/wifi/hakios.w2.c (its Wi-Fi).
 */
#ifndef _SYS_HAKIOS_XHCI_H_
#define _SYS_HAKIOS_XHCI_H_

#include <stdbool.h>
#include <stdint.h>

void hakios_xhci_init(void);

/*
 * The chip's side of the bus, once its firmware is up: each call is one whole exchange, made under
 * the bus's lock, from any thread but not from interrupt context. Addresses are the chip's.
 */
bool hakios_w2_read(uint32_t addr, uint32_t *value);
bool hakios_w2_write(uint32_t addr, uint32_t value);
bool hakios_w2_sram_write(uint32_t addr, const void *data, uint32_t length);
bool hakios_w2_sram_read(uint32_t addr, void *data, uint32_t length);
bool hakios_w2_irq(uint32_t words[4]);                  /* the "interrupt": RX head, status, BT read, BT write */
bool hakios_w2_packets(const void *pages, const uint16_t *sizes, unsigned count);       /* frames to send */
bool hakios_w2_confirmations(void *data, uint32_t length);                              /* the TX confirmations */

/* the same over the chip's Bluetooth endpoints (2 OUT, 2 IN) */
bool hakios_w2bt_read(uint32_t addr, uint32_t *value);
bool hakios_w2bt_write(uint32_t addr, uint32_t value);
bool hakios_w2bt_sram_write(uint32_t addr, const void *data, uint32_t length);
bool hakios_w2bt_sram_read(uint32_t addr, void *data, uint32_t length);

/* hakios.w2bt.c: the Bluetooth host, started once the Wi-Fi radio is up (boot-arg w2bt=1) */
void hakios_w2bt_start(void);

/* hakios.w2.c: the Wi-Fi driver, run on the bus's thread once the firmware is up; does not return */
void hakios_w2_run(unsigned step);

#endif

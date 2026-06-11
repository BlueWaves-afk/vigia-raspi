/**
 * Pin assignments — matches .claude/design/03_pico2_firmware_contracts.md
 */
#ifndef VIGIA_PINS_H
#define VIGIA_PINS_H

#include "hardware/uart.h"

#define VIGIA_GPS_UART       uart1
#define VIGIA_GPS_UART_BAUD  9600u
#define VIGIA_GPS_TX_PIN     8u
#define VIGIA_GPS_RX_PIN     9u

#endif

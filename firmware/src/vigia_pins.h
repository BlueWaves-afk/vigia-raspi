/**
 * Pin assignments — matches .claude/design/03_pico2_firmware_contracts.md
 */
#ifndef VIGIA_PINS_H
#define VIGIA_PINS_H

#include "hardware/spi.h"
#include "hardware/uart.h"

/* UART1 — NEO-M8N GPS */
#define VIGIA_GPS_UART       uart1
#define VIGIA_GPS_UART_BAUD  9600u
#define VIGIA_GPS_TX_PIN     8u
#define VIGIA_GPS_RX_PIN     9u

/* SPI0 — BNO085 IMU @ 3 MHz, CPOL=1 CPHA=1 (SPI mode 3) */
#define VIGIA_BNO085_SPI         spi0
#define VIGIA_BNO085_SPI_BAUD    3000000u
#define VIGIA_BNO085_SCK_PIN     18u
#define VIGIA_BNO085_MOSI_PIN    19u  /* sensor AD0 */
#define VIGIA_BNO085_MISO_PIN    16u  /* sensor SDA */
#define VIGIA_BNO085_CS_PIN      17u
#define VIGIA_BNO085_INT_PIN     20u
#define VIGIA_BNO085_RST_PIN     21u
#define VIGIA_BNO085_WAKE_PIN    22u  /* sensor PS0 — required for SPI */

#endif

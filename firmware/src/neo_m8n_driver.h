/**
 * NEO-M8N GPS driver — UBX byte stream, NAV-PVT (0x01 0x07) parser.
 *
 * UART1 on GP8 (TX) / GP9 (RX) @ 9600 baud (module default).
 */
#ifndef NEO_M8N_DRIVER_H
#define NEO_M8N_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/uart.h"

typedef enum {
    NEO_M8N_SRC_NONE = 0,
    NEO_M8N_SRC_UBX,
    NEO_M8N_SRC_NMEA,
} neo_m8n_source_t;

typedef struct {
    double latitude;
    double longitude;
    float altitude_m;
    float speed_ms;
    float course_deg;
    float hdop;
    uint8_t fix_type;
    uint8_t satellites;
    bool valid;
} neo_m8n_report_t;

void neo_m8n_init(uart_inst_t *uart);
void neo_m8n_poll(void);
bool neo_m8n_get_report(neo_m8n_report_t *out);
uint32_t neo_m8n_uart_rx_bytes(void);
uint32_t neo_m8n_baud_rate(void);
neo_m8n_source_t neo_m8n_source(void);

#endif

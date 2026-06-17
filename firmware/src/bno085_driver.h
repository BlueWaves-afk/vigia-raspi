/**
 * BNO085 IMU driver — SHTP over SPI0.
 */
#ifndef BNO085_DRIVER_H
#define BNO085_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/spi.h"

typedef struct {
    float qw;
    float qx;
    float qy;
    float qz;
    float ax;
    float ay;
    float az;
    uint8_t cal_status;
    bool valid;
} bno085_report_t;

typedef struct {
    uint8_t probe_status;
    uint8_t int_level;
    uint8_t rst_level;
    uint8_t wake_level;
    uint8_t cs_level;
    uint8_t header[4];
    uint16_t shtp_len;
    uint8_t shtp_channel;
    uint8_t shtp_seq;
} bno085_probe_t;

#define BNO085_STAGE_NONE           0u
#define BNO085_STAGE_BOOT_TIMEOUT   1u
#define BNO085_STAGE_ENABLE_ROT     2u
#define BNO085_STAGE_ENABLE_ACCEL   3u
#define BNO085_STAGE_READY          4u
#define BNO085_STAGE_NO_DATA        5u

#define BNO085_PROBE_NOT_RUN        0u
#define BNO085_PROBE_NO_INT         1u
#define BNO085_PROBE_SPI_READ_FAIL  2u
#define BNO085_PROBE_BAD_HEADER     3u
#define BNO085_PROBE_SPI_READ_OK    4u

void bno085_init(spi_inst_t *spi);
void bno085_poll(void);
bool bno085_get_report(bno085_report_t *out);
void bno085_get_probe(bno085_probe_t *out);
bool bno085_ready(void);
uint32_t bno085_sample_count(void);
uint8_t bno085_init_stage(void);
uint32_t bno085_shtp_rx_count(void);
uint8_t bno085_last_report_id(void);
uint32_t bno085_ch3_rx_count(void);
uint8_t bno085_last_channel(void);
uint8_t bno085_last_payload0(void);

#endif

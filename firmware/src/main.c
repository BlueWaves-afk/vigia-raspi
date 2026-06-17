/**
 * vigia_pico_hello — Pi ↔ Pico 2 USB CDC link + NEO-M8N GPS + BNO085 IMU.
 *
 * USB serial (/dev/ttyACM0 on the Pi):
 *   VIGIA_GPS  — parsed fix @ 1 Hz (+ timestamp_us)
 *   VIGIA_IMU  — quaternion + linear accel @ ~100 Hz (each new sample)
 *   VIGIA_PING — link heartbeat @ 1 Hz (when no GPS frames yet)
 *
 * GPS wiring (UART1):
 *   Pico GP8 (TX) → M8N RX
 *   Pico GP9 (RX) ← M8N TX
 *
 * BNO085 wiring (SPI0): see firmware/README.md
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bno085_driver.h"
#include "hardware/gpio.h"
#include "neo_m8n_driver.h"
#include "pico/stdlib.h"
#include "vigia_pins.h"

/* Phase 2 — COBS signing pipeline (VIGIA_PHASE2=1 to enable) */
#if defined(VIGIA_PHASE2) && (VIGIA_PHASE2 == 1)
#  include "atecc608a_driver.h"
#  include "cobs_tx_driver.h"
#endif

static const uint LED_PIN = PICO_DEFAULT_LED_PIN;

#ifndef VIGIA_IMU_DEBUG_USB
#define VIGIA_IMU_DEBUG_USB 1
#endif

static const char *source_name(neo_m8n_source_t src) {
    switch (src) {
    case NEO_M8N_SRC_UBX:
        return "ubx";
    case NEO_M8N_SRC_NMEA:
        return "nmea";
    default:
        return "none";
    }
}

static void print_gps_line(uint32_t seq, const neo_m8n_report_t *r) {
    const uint64_t timestamp_us = time_us_64();
    printf("VIGIA_GPS seq=%lu timestamp_us=%llu lat=%.7f lon=%.7f speed_ms=%.2f "
           "fix_type=%u satellites=%u hdop=%.2f valid=%u src=%s\n",
           (unsigned long)seq,
           (unsigned long long)timestamp_us,
           r->latitude,
           r->longitude,
           (double)r->speed_ms,
           (unsigned)r->fix_type,
           (unsigned)r->satellites,
           (double)r->hdop,
           r->valid ? 1u : 0u,
           source_name(neo_m8n_source()));
}

static void print_ping_line(uint32_t seq, uint64_t uptime_ms, uint32_t boot_ms) {
    bno085_probe_t probe;
    bno085_get_probe(&probe);

    printf("VIGIA_PING seq=%lu uptime_ms=%llu boot_ms=%lu fw=gps+imu "
           "uart_rx=%lu baud=%lu imu_ready=%u imu_stage=%u imu_samples=%lu "
           "imu_pkts=%lu imu_ch3=%lu imu_ch=%u imu_b0=0x%02x imu_rid=0x%02x "
           "imu_probe=%u imu_int=%u imu_rst=%u imu_wake=%u imu_cs=%u "
           "imu_h0=0x%02x imu_h1=0x%02x imu_h2=0x%02x imu_h3=0x%02x "
           "imu_len=%u imu_probe_ch=%u imu_probe_seq=%u\n",
           (unsigned long)seq,
           (unsigned long long)uptime_ms,
           (unsigned long)boot_ms,
           (unsigned long)neo_m8n_uart_rx_bytes(),
           (unsigned long)neo_m8n_baud_rate(),
           bno085_ready() ? 1u : 0u,
           (unsigned)bno085_init_stage(),
           (unsigned long)bno085_sample_count(),
           (unsigned long)bno085_shtp_rx_count(),
           (unsigned long)bno085_ch3_rx_count(),
           (unsigned)bno085_last_channel(),
           (unsigned)bno085_last_payload0(),
           (unsigned)bno085_last_report_id(),
           (unsigned)probe.probe_status,
           (unsigned)probe.int_level,
           (unsigned)probe.rst_level,
           (unsigned)probe.wake_level,
           (unsigned)probe.cs_level,
           (unsigned)probe.header[0],
           (unsigned)probe.header[1],
           (unsigned)probe.header[2],
           (unsigned)probe.header[3],
           (unsigned)probe.shtp_len,
           (unsigned)probe.shtp_channel,
           (unsigned)probe.shtp_seq);
}

#if VIGIA_IMU_DEBUG_USB
static void print_imu_line(uint32_t seq, const bno085_report_t *r) {
    const uint64_t timestamp_us = time_us_64();
    const float norm = sqrtf((r->qw * r->qw) + (r->qx * r->qx) + (r->qy * r->qy) +
                             (r->qz * r->qz));
    printf("VIGIA_IMU seq=%lu timestamp_us=%llu qw=%.4f qx=%.4f qy=%.4f qz=%.4f "
           "ax=%.3f ay=%.3f az=%.3f cal=%u valid=%u qnorm=%.4f\n",
           (unsigned long)seq,
           (unsigned long long)timestamp_us,
           (double)r->qw,
           (double)r->qx,
           (double)r->qy,
           (double)r->qz,
           (double)r->ax,
           (double)r->ay,
           (double)r->az,
           (unsigned)r->cal_status,
           r->valid ? 1u : 0u,
           (double)norm);
}
#endif

int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    neo_m8n_init(VIGIA_GPS_UART);
    bno085_init(VIGIA_BNO085_SPI);

#if defined(VIGIA_PHASE2) && (VIGIA_PHASE2 == 1)
    vigia_atca_init();

    /* Static buffers — no heap allowed */
    static EtHashInput     s_et_input;
    static SignedEtPacket  s_et_pkt;
    static uint8_t         s_cobs_frame[COBS_FRAME_MAX];
    static uint32_t        s_et_seq = 0;

    /* Phase 2: device_id provisioned in ATECC608A data zone.
     * Hardcoded placeholder until provisioning script runs. */
    static const uint8_t k_device_id[16] = {
        0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B
    };
#endif

    const uint32_t boot_ms = to_ms_since_boot(get_absolute_time());
    uint32_t gps_seq = 0;
#if VIGIA_IMU_DEBUG_USB
    uint32_t imu_seq = 0;
#endif
    bool gps_seen = false;

    sleep_ms(1500);

    absolute_time_t next_gps_print = make_timeout_time_ms(1000);

    while (true) {
        neo_m8n_poll();
        bno085_poll();

#if VIGIA_IMU_DEBUG_USB
        if (bno085_ready()) {
            bno085_report_t imu;
            while (bno085_get_report(&imu) && imu.valid) {
                print_imu_line(imu_seq++, &imu);
            }
            fflush(stdout);
        }
#endif

        if (absolute_time_diff_us(get_absolute_time(), next_gps_print) <= 0) {
            neo_m8n_report_t report;
            if (neo_m8n_get_report(&report)) {
                gps_seen = true;
                if (report.valid) {
                    gpio_put(LED_PIN, 1);
                } else {
                    gpio_xor_mask(1u << LED_PIN);
                }

#if defined(VIGIA_PHASE2) && (VIGIA_PHASE2 == 1)
                /* ── Phase 2: assemble, hash, sign, COBS-encode ───────── */
                bno085_report_t last_imu;
                const bool have_imu = bno085_get_report(&last_imu);

                memcpy(s_et_input.device_id, k_device_id, 16);
                s_et_input.timestamp_us = time_us_64();
                s_et_input.sequence     = s_et_seq;
                if (have_imu) {
                    s_et_input.qw = last_imu.qw; s_et_input.qx = last_imu.qx;
                    s_et_input.qy = last_imu.qy; s_et_input.qz = last_imu.qz;
                    s_et_input.ax = last_imu.ax; s_et_input.ay = last_imu.ay;
                    s_et_input.az = last_imu.az;
                    s_et_input.cal_status = last_imu.cal_status;
                }
                s_et_input.latitude   = report.latitude;
                s_et_input.longitude  = report.longitude;
                s_et_input.speed_ms   = report.speed_ms;
                s_et_input.fix_type   = report.fix_type;
                s_et_input.satellites = report.satellites;
                memset(s_et_input._pad0, 0, sizeof(s_et_input._pad0));
                memset(s_et_input._pad1, 0, sizeof(s_et_input._pad1));

                /* Hash (~40 ms) then sign (~57 ms) — runs in GPS branch (1 Hz budget) */
                vigia_atca_sha((const uint8_t *)&s_et_input, ET_HASH_INPUT_SIZE,
                               s_et_pkt.et_hash);
                vigia_atca_sign(s_et_pkt.et_hash, s_et_pkt.ecdsa_sig);

                /* Populate packet header + IMU + GPS */
                s_et_pkt.magic      = 0xE7;
                s_et_pkt.version    = 0x02;
                s_et_pkt.timestamp_us = s_et_input.timestamp_us;
                s_et_pkt.sequence     = s_et_seq++;
                s_et_pkt.qw = s_et_input.qw; s_et_pkt.qx = s_et_input.qx;
                s_et_pkt.qy = s_et_input.qy; s_et_pkt.qz = s_et_input.qz;
                s_et_pkt.ax = s_et_input.ax; s_et_pkt.ay = s_et_input.ay;
                s_et_pkt.az = s_et_input.az;
                s_et_pkt.cal_status = s_et_input.cal_status;
                memset(s_et_pkt._imu_pad, 0, sizeof(s_et_pkt._imu_pad));
                s_et_pkt.latitude   = report.latitude;
                s_et_pkt.longitude  = report.longitude;
                s_et_pkt.speed_ms   = report.speed_ms;
                s_et_pkt.fix_type   = report.fix_type;
                s_et_pkt.satellites = report.satellites;
                memset(s_et_pkt._gps_pad, 0, sizeof(s_et_pkt._gps_pad));

                /* COBS encode and transmit */
                size_t frame_len = cobs_encode(
                    (const uint8_t *)&s_et_pkt, SIGNED_ET_PACKET_SIZE,
                    s_cobs_frame, sizeof(s_cobs_frame));
                if (frame_len > 0) {
                    fwrite(s_cobs_frame, 1, frame_len, stdout);
                    fflush(stdout);
                }
#else
                /* Phase 1: plain text output */
                print_gps_line(gps_seq++, &report);
#endif /* VIGIA_PHASE2 */
            } /* end neo_m8n_get_report */
            else if (!gps_seen) {
                gpio_xor_mask(1u << LED_PIN);
                const uint64_t uptime_ms = to_ms_since_boot(get_absolute_time());
                print_ping_line(gps_seq++, uptime_ms, boot_ms);
            }

            fflush(stdout);
            next_gps_print = make_timeout_time_ms(1000);
        }

        sleep_ms(1);
    }

    return 0;
}

/**
 * vigia_pico_hello — Pi ↔ Pico 2 USB CDC link + NEO-M8N GPS (UBX NAV-PVT).
 *
 * USB serial (/dev/ttyACM0 on the Pi):
 *   VIGIA_GPS  — parsed fix @ 1 Hz
 *   VIGIA_PING — link heartbeat @ 1 Hz (when no GPS frames yet)
 *
 * GPS wiring (UART1):
 *   Pico GP8 (TX) → M8N RX
 *   Pico GP9 (RX) ← M8N TX
 *   GND common, M8N VCC 3.3 V
 */

#include <stdio.h>

#include "hardware/gpio.h"
#include "neo_m8n_driver.h"
#include "pico/stdlib.h"
#include "vigia_pins.h"

static const uint LED_PIN = PICO_DEFAULT_LED_PIN;

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
    printf("VIGIA_GPS seq=%lu lat=%.7f lon=%.7f speed_ms=%.2f fix_type=%u "
           "satellites=%u hdop=%.2f valid=%u src=%s\n",
           (unsigned long)seq,
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
    printf("VIGIA_PING seq=%lu uptime_ms=%llu boot_ms=%lu fw=gps "
           "uart_rx=%lu baud=%lu\n",
           (unsigned long)seq,
           (unsigned long long)uptime_ms,
           (unsigned long)boot_ms,
           (unsigned long)neo_m8n_uart_rx_bytes(),
           (unsigned long)neo_m8n_baud_rate());
}

int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    neo_m8n_init(VIGIA_GPS_UART);

    const uint32_t boot_ms = to_ms_since_boot(get_absolute_time());
    uint32_t seq = 0;
    bool gps_seen = false;

    sleep_ms(1500);

    absolute_time_t next_print = make_timeout_time_ms(1000);

    while (true) {
        neo_m8n_poll();

        if (absolute_time_diff_us(get_absolute_time(), next_print) <= 0) {
            neo_m8n_report_t report;
            if (neo_m8n_get_report(&report)) {
                gps_seen = true;
                if (report.valid) {
                    gpio_put(LED_PIN, 1);
                } else {
                    gpio_xor_mask(1u << LED_PIN);
                }
                print_gps_line(seq++, &report);
            } else if (!gps_seen) {
                gpio_xor_mask(1u << LED_PIN);
                const uint64_t uptime_ms = to_ms_since_boot(get_absolute_time());
                print_ping_line(seq++, uptime_ms, boot_ms);
            }

            fflush(stdout);
            next_print = make_timeout_time_ms(1000);
        }

        sleep_ms(10);
    }

    return 0;
}

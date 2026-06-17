/**
 * NEO-M8N parser — UBX NAV-PVT preferred, NMEA GGA/RMC fallback.
 */
#include "neo_m8n_driver.h"

#include <stdlib.h>
#include <string.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "vigia_pins.h"

#define UBX_SYNC1        0xB5u
#define UBX_SYNC2        0x62u
#define UBX_CLASS_NAV    0x01u
#define UBX_ID_NAV_PVT   0x07u
#define UBX_NAV_PVT_LEN  92u
#define UBX_FRAME_MAX    128u
#define NMEA_LINE_MAX    96u

#define UBX_CLASS_CFG    0x06u
#define UBX_ID_CFG_MSG   0x01u
#define UBX_ID_CFG_RATE  0x08u

enum ubx_rx_state {
    UBX_RX_SYNC1 = 0,
    UBX_RX_SYNC2,
    UBX_RX_CLASS,
    UBX_RX_ID,
    UBX_RX_LEN_L,
    UBX_RX_LEN_H,
    UBX_RX_PAYLOAD,
    UBX_RX_CK_A,
    UBX_RX_CK_B,
};

static uart_inst_t *gps_uart_;
static neo_m8n_report_t latest_report_;
static bool report_updated_;
static neo_m8n_source_t report_source_;
static uint32_t uart_rx_bytes_;
static uint32_t active_baud_;

static uint8_t frame_buf_[UBX_FRAME_MAX];
static uint16_t payload_len_;
static uint16_t payload_pos_;
static uint8_t ck_a_;
static uint8_t ck_b_;
static enum ubx_rx_state rx_state_;

static char nmea_line_[NMEA_LINE_MAX];
static uint16_t nmea_len_;
static bool nmea_active_;

static void neo_m8n_poll_uart(void);

static void ubx_checksum_byte(uint8_t byte) {
    ck_a_ = (uint8_t)(ck_a_ + byte);
    ck_b_ = (uint8_t)(ck_b_ + ck_a_);
}

static void ubx_send(uart_inst_t *uart, uint8_t cls, uint8_t id,
                     const uint8_t *payload, uint16_t len) {
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    uart_putc_raw(uart, UBX_SYNC1);
    uart_putc_raw(uart, UBX_SYNC2);
    uart_putc_raw(uart, cls);
    uart_putc_raw(uart, id);
    uart_putc_raw(uart, (uint8_t)(len & 0xFFu));
    uart_putc_raw(uart, (uint8_t)(len >> 8));

    const uint8_t header[] = {cls, id, (uint8_t)(len & 0xFFu), (uint8_t)(len >> 8)};
    for (size_t i = 0; i < sizeof(header); i++) {
        ck_a = (uint8_t)(ck_a + header[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }

    for (uint16_t i = 0; i < len; i++) {
        const uint8_t byte = payload[i];
        uart_putc_raw(uart, byte);
        ck_a = (uint8_t)(ck_a + byte);
        ck_b = (uint8_t)(ck_b + ck_a);
    }

    uart_putc_raw(uart, ck_a);
    uart_putc_raw(uart, ck_b);
}

static int32_t read_i32_le(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
                     ((uint32_t)p[3] << 24));
}

static uint32_t read_u32_le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void mark_report(neo_m8n_source_t src) {
    report_source_ = src;
    report_updated_ = true;
}

static void parse_nav_pvt(const uint8_t *payload, uint16_t len) {
    if (len < UBX_NAV_PVT_LEN) {
        return;
    }

    const uint8_t fix_type = payload[20];
    const uint8_t satellites = payload[23];
    const int32_t lon_raw = read_i32_le(&payload[24]);
    const int32_t lat_raw = read_i32_le(&payload[28]);
    const int32_t h_msl_mm = read_i32_le(&payload[36]);
    const uint32_t g_speed_mm_s = read_u32_le(&payload[60]);
    const int32_t head_mot_raw = read_i32_le(&payload[64]);
    const uint16_t p_dop_raw = read_u16_le(&payload[76]);

    latest_report_.fix_type = fix_type;
    latest_report_.satellites = satellites;
    latest_report_.longitude = (double)lon_raw * 1e-7;
    latest_report_.latitude = (double)lat_raw * 1e-7;
    latest_report_.altitude_m = (float)h_msl_mm * 0.001f;
    latest_report_.speed_ms = (float)g_speed_mm_s * 0.001f;
    latest_report_.course_deg = (float)head_mot_raw * 1e-5f;
    latest_report_.hdop = (float)p_dop_raw * 0.01f;
    latest_report_.valid = (fix_type >= 2u);
    mark_report(NEO_M8N_SRC_UBX);
}

static double nmea_to_degrees(const char *ddmm, char hemi) {
    const double val = strtod(ddmm, NULL);
    const int deg = (int)(val / 100.0);
    const double minutes = val - ((double)deg * 100.0);
    double out = (double)deg + (minutes / 60.0);
    if (hemi == 'S' || hemi == 'W') {
        out = -out;
    }
    return out;
}

static const char *nmea_field(const char *line, int index) {
    const char *p = line;
    while (*p != '\0' && index > 0) {
        if (*p == ',') {
            index--;
        }
        p++;
    }
    if (index != 0) {
        return NULL;
    }
    return p;
}

static bool field_is_empty(const char *field) {
    return field == NULL || field[0] == '\0';
}

static void parse_nmea_gga(const char *line) {
    if (strstr(line, "GGA") == NULL) {
        return;
    }

    const char *lat_field = nmea_field(line, 2);
    const char *lat_hemi = nmea_field(line, 3);
    const char *lon_field = nmea_field(line, 4);
    const char *lon_hemi = nmea_field(line, 5);
    const char *fix_field = nmea_field(line, 6);
    const char *sats_field = nmea_field(line, 7);
    const char *hdop_field = nmea_field(line, 8);
    const char *alt_field = nmea_field(line, 9);

    if (field_is_empty(fix_field)) {
        return;
    }

    const int fix_quality = (int)strtol(fix_field, NULL, 10);
    latest_report_.fix_type = (fix_quality >= 1) ? 3u : 0u;
    latest_report_.valid = (fix_quality >= 1);
    latest_report_.satellites =
        field_is_empty(sats_field) ? 0u : (uint8_t)strtoul(sats_field, NULL, 10);
    latest_report_.hdop = field_is_empty(hdop_field) ? 99.9f : (float)strtod(hdop_field, NULL);
    latest_report_.altitude_m =
        field_is_empty(alt_field) ? 0.0f : (float)strtod(alt_field, NULL);

    if (!field_is_empty(lat_field) && !field_is_empty(lat_hemi) &&
        !field_is_empty(lon_field) && !field_is_empty(lon_hemi)) {
        latest_report_.latitude = nmea_to_degrees(lat_field, lat_hemi[0]);
        latest_report_.longitude = nmea_to_degrees(lon_field, lon_hemi[0]);
    }

    mark_report(NEO_M8N_SRC_NMEA);
}

static void parse_nmea_rmc(const char *line) {
    if (strstr(line, "RMC") == NULL) {
        return;
    }

    const char *status = nmea_field(line, 2);
    const char *speed_knots = nmea_field(line, 7);
    const char *course = nmea_field(line, 8);

    if (field_is_empty(status) || status[0] != 'A') {
        return;
    }

    if (!field_is_empty(speed_knots)) {
        latest_report_.speed_ms = (float)strtod(speed_knots, NULL) * 0.514444f;
    }
    if (!field_is_empty(course)) {
        latest_report_.course_deg = (float)strtod(course, NULL);
    }

    if (report_source_ != NEO_M8N_SRC_UBX) {
        mark_report(NEO_M8N_SRC_NMEA);
    }
}

static void parse_nmea_line(const char *line) {
    if (line[0] != '$') {
        return;
    }
    parse_nmea_gga(line);
    parse_nmea_rmc(line);
}

static void reset_rx_state(void) {
    rx_state_ = UBX_RX_SYNC1;
    payload_len_ = 0;
    payload_pos_ = 0;
    ck_a_ = 0;
    ck_b_ = 0;
}

static void handle_ubx_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
    if (cls == UBX_CLASS_NAV && id == UBX_ID_NAV_PVT) {
        parse_nav_pvt(payload, len);
    }
}

static void ubx_feed_byte(uint8_t byte) {
    switch (rx_state_) {
    case UBX_RX_SYNC1:
        if (byte == UBX_SYNC1) {
            rx_state_ = UBX_RX_SYNC2;
        }
        break;

    case UBX_RX_SYNC2:
        if (byte == UBX_SYNC2) {
            rx_state_ = UBX_RX_CLASS;
            ck_a_ = 0;
            ck_b_ = 0;
        } else if (byte == UBX_SYNC1) {
            rx_state_ = UBX_RX_SYNC2;
        } else {
            rx_state_ = UBX_RX_SYNC1;
        }
        break;

    case UBX_RX_CLASS:
        frame_buf_[0] = byte;
        ubx_checksum_byte(byte);
        rx_state_ = UBX_RX_ID;
        break;

    case UBX_RX_ID:
        frame_buf_[1] = byte;
        ubx_checksum_byte(byte);
        rx_state_ = UBX_RX_LEN_L;
        break;

    case UBX_RX_LEN_L:
        frame_buf_[2] = byte;
        ubx_checksum_byte(byte);
        rx_state_ = UBX_RX_LEN_H;
        break;

    case UBX_RX_LEN_H:
        frame_buf_[3] = byte;
        ubx_checksum_byte(byte);
        payload_len_ = (uint16_t)frame_buf_[2] | ((uint16_t)byte << 8);
        if (payload_len_ > (UBX_FRAME_MAX - 6u)) {
            reset_rx_state();
            break;
        }
        payload_pos_ = 0;
        rx_state_ = (payload_len_ == 0u) ? UBX_RX_CK_A : UBX_RX_PAYLOAD;
        break;

    case UBX_RX_PAYLOAD:
        frame_buf_[4u + payload_pos_] = byte;
        ubx_checksum_byte(byte);
        payload_pos_++;
        if (payload_pos_ >= payload_len_) {
            rx_state_ = UBX_RX_CK_A;
        }
        break;

    case UBX_RX_CK_A:
        if (byte == ck_a_) {
            rx_state_ = UBX_RX_CK_B;
        } else {
            reset_rx_state();
        }
        break;

    case UBX_RX_CK_B:
        if (byte == ck_b_) {
            handle_ubx_frame(frame_buf_[0], frame_buf_[1], &frame_buf_[4], payload_len_);
        }
        reset_rx_state();
        break;
    }
}

static void nmea_feed_byte(uint8_t byte) {
    if (byte == '$') {
        nmea_len_ = 0;
        nmea_active_ = true;
        nmea_line_[nmea_len_++] = (char)byte;
        return;
    }

    if (!nmea_active_) {
        return;
    }

    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        if (nmea_len_ < NMEA_LINE_MAX) {
            nmea_line_[nmea_len_] = '\0';
            parse_nmea_line(nmea_line_);
        }
        nmea_active_ = false;
        nmea_len_ = 0;
        return;
    }

    if (nmea_len_ + 1u < NMEA_LINE_MAX) {
        nmea_line_[nmea_len_++] = (char)byte;
    } else {
        nmea_active_ = false;
        nmea_len_ = 0;
    }
}

static void feed_byte(uint8_t byte) {
    nmea_feed_byte(byte);
    ubx_feed_byte(byte);
}

static bool uart_saw_traffic(uint32_t min_bytes) {
    const uint32_t saved = uart_rx_bytes_;
    uart_rx_bytes_ = 0;
    reset_rx_state();
    nmea_active_ = false;
    nmea_len_ = 0;

    absolute_time_t deadline = make_timeout_time_ms(400);
    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        neo_m8n_poll_uart();
        if (uart_rx_bytes_ >= min_bytes) {
            uart_rx_bytes_ = saved;
            return true;
        }
        sleep_ms(5);
    }

    const bool ok = uart_rx_bytes_ >= min_bytes;
    uart_rx_bytes_ = saved;
    return ok;
}

static uint32_t autodetect_baud(uart_inst_t *uart) {
    static const uint32_t candidates[] = {9600u, 38400u, 115200u};

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        uart_rx_bytes_ = 0;
        reset_rx_state();
        nmea_active_ = false;
        nmea_len_ = 0;
        uart_init(uart, candidates[i]);
        sleep_ms(50);
        if (uart_saw_traffic(8u)) {
            return candidates[i];
        }
    }

    return VIGIA_GPS_UART_BAUD;
}

static void configure_gps(uart_inst_t *uart) {
    const uint8_t cfg_msg[] = {
        UBX_CLASS_NAV, UBX_ID_NAV_PVT,
        0x00u, 0x01u, 0x00u, 0x00u, 0x00u, 0x00u,
    };
    ubx_send(uart, UBX_CLASS_CFG, UBX_ID_CFG_MSG, cfg_msg, (uint16_t)sizeof(cfg_msg));
    sleep_ms(50);

    const uint8_t cfg_rate[] = {
        0xE8u, 0x03u, 0x01u, 0x00u, 0x00u, 0x00u,
    };
    ubx_send(uart, UBX_CLASS_CFG, UBX_ID_CFG_RATE, cfg_rate, (uint16_t)sizeof(cfg_rate));
    sleep_ms(50);
}

void neo_m8n_init(uart_inst_t *uart) {
    gps_uart_ = uart;
    memset(&latest_report_, 0, sizeof(latest_report_));
    report_updated_ = false;
    report_source_ = NEO_M8N_SRC_NONE;
    uart_rx_bytes_ = 0;
    active_baud_ = VIGIA_GPS_UART_BAUD;
    reset_rx_state();
    nmea_active_ = false;
    nmea_len_ = 0;

    gpio_set_function(VIGIA_GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(VIGIA_GPS_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(VIGIA_GPS_RX_PIN);

    active_baud_ = autodetect_baud(uart);
    configure_gps(uart);
}

static void neo_m8n_poll_uart(void) {
    if (gps_uart_ == NULL) {
        return;
    }

    while (uart_is_readable(gps_uart_)) {
        feed_byte(uart_getc(gps_uart_));
        uart_rx_bytes_++;
    }
}

void neo_m8n_poll(void) {
    neo_m8n_poll_uart();
}

uint32_t neo_m8n_uart_rx_bytes(void) {
    return uart_rx_bytes_;
}

uint32_t neo_m8n_baud_rate(void) {
    return active_baud_;
}

neo_m8n_source_t neo_m8n_source(void) {
    return report_source_;
}

bool neo_m8n_get_report(neo_m8n_report_t *out) {
    if (out == NULL) {
        return false;
    }

    *out = latest_report_;
    return report_updated_;
}

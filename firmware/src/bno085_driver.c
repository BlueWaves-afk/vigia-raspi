/**
 * BNO085 SHTP driver — SPI mode 3 + PS0/WAKE.
 *
 * Outputs: Game Rotation Vector (qw/qx/qy/qz, no magnetometer) +
 *          Linear Acceleration (gravity-compensated) at REPORT_INTERVAL_US.
 *
 * WAKE/INT protocol notes
 * ───────────────────────
 * After reset the BNO085 will not drive INT# until WAKE/PS0 is asserted by
 * the host at least once.  wake_for_transaction() handles this for every SPI
 * read; when INT is already low (sensor has data) it returns immediately
 * without toggling WAKE.  The poll loop therefore only calls
 * shtp_read_packet_ms when gpio_get(INT)==0, guaranteeing that
 * wake_for_transaction never toggles WAKE during normal polling.
 */

#include "bno085_driver.h"

#include <string.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "vigia_pins.h"

/* ── SHTP / SH-2 constants ──────────────────────────────────────────────── */

#define SHTP_HDR_LEN             4u
#define SHTP_RX_BUF_LEN          512u
#define SHTP_TX_BUF_LEN          32u

#define SHTP_CH_EXECUTABLE       1u
#define SHTP_CH_SENSOR_HUB       2u
#define SHTP_CH_INPUT            3u

#define SHTP_CMD_SET_FEATURE     0xFDu
#define EXECUTABLE_RESET_DONE    1u

#define REPORT_ROTATION_VECTOR   0x05u
#define REPORT_LINEAR_ACCEL      0x04u
#define REPORT_BASE_TIMESTAMP    0xFBu
#define REPORT_TIMESTAMP_REBASE  0xFAu

#define REPORT_INTERVAL_US       10000u   /* 100 Hz */

#define QUAT_SCALE               (1.0f / 16384.0f)
#define ACCEL_SCALE              (1.0f / 256.0f)

/* ── Driver state ───────────────────────────────────────────────────────── */

static spi_inst_t     *spi_;
static bno085_report_t latest_;
static bool            report_updated_;
static bool            ready_;
static bool            enable_pending_;
static uint32_t        sample_count_;
static uint32_t        shtp_rx_count_;
static uint32_t        ch3_rx_count_;
static uint8_t         init_stage_;
static uint8_t         last_report_id_;
static uint8_t         last_channel_;
static uint8_t         last_payload0_;
static uint8_t         out_seq_[8];
static bno085_probe_t  probe_;

static uint8_t rx_buf_[SHTP_RX_BUF_LEN];
static uint8_t tx_buf_[SHTP_TX_BUF_LEN];

/* ── Low-level GPIO / SPI helpers ───────────────────────────────────────── */

static void cs_assert(void)    { gpio_put(VIGIA_BNO085_CS_PIN,   0); }
static void cs_deassert(void)  { gpio_put(VIGIA_BNO085_CS_PIN,   1); }
static void wake_release(void) { gpio_put(VIGIA_BNO085_WAKE_PIN, 1); }

static bool wait_int_low(uint32_t timeout_ms) {
    absolute_time_t dl = make_timeout_time_ms(timeout_ms);
    while (gpio_get(VIGIA_BNO085_INT_PIN) != 0u) {
        if (absolute_time_diff_us(get_absolute_time(), dl) <= 0) {
            return false;
        }
        tight_loop_contents();
    }
    return true;
}

/* Assert WAKE/PS0 when INT is not already low; wait for sensor to ACK via INT.
 * If INT is already low the function returns immediately without touching WAKE.
 * Called before every SPI read — the BNO085 needs at least one WAKE assertion
 * to begin driving INT after reset. */
static bool wake_for_transaction(uint32_t timeout_ms) {
    if (gpio_get(VIGIA_BNO085_INT_PIN) == 0u) {
        return true;
    }
    gpio_put(VIGIA_BNO085_WAKE_PIN, 0);
    const bool ok = wait_int_low(timeout_ms);
    wake_release();
    sleep_us(200);
    return ok;
}

static bool spi_write_buf(const uint8_t *buf, size_t len) {
    cs_assert();
    sleep_us(2);
    const int rc = spi_write_blocking(spi_, buf, len);
    cs_deassert();
    return rc == (int)len;
}

static int16_t read_i16_le(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void capture_probe_gpio(void) {
    probe_.int_level  = (uint8_t)gpio_get(VIGIA_BNO085_INT_PIN);
    probe_.rst_level  = (uint8_t)gpio_get(VIGIA_BNO085_RST_PIN);
    probe_.wake_level = (uint8_t)gpio_get(VIGIA_BNO085_WAKE_PIN);
    probe_.cs_level   = (uint8_t)gpio_get(VIGIA_BNO085_CS_PIN);
}

static bool header_is_plausible(uint16_t len, uint8_t channel) {
    return len >= SHTP_HDR_LEN && len <= SHTP_RX_BUF_LEN && channel <= SHTP_CH_INPUT;
}

static void probe_spi_header_after_reset(void) {
    memset(&probe_, 0, sizeof(probe_));
    probe_.probe_status = BNO085_PROBE_NOT_RUN;
    capture_probe_gpio();

    if (!wake_for_transaction(1000u)) {
        probe_.probe_status = BNO085_PROBE_NO_INT;
        capture_probe_gpio();
        return;
    }

    for (size_t i = 0; i < sizeof(probe_.header); i++) {
        probe_.header[i] = 0xFFu;
    }

    cs_assert();
    sleep_us(2);
    const int rc = spi_write_read_blocking(spi_, probe_.header, probe_.header,
                                           sizeof(probe_.header));
    cs_deassert();
    capture_probe_gpio();

    if (rc != (int)sizeof(probe_.header)) {
        probe_.probe_status = BNO085_PROBE_SPI_READ_FAIL;
        return;
    }

    probe_.shtp_len = ((uint16_t)probe_.header[0] |
                       ((uint16_t)probe_.header[1] << 8)) & 0x7FFFu;
    probe_.shtp_channel = probe_.header[2];
    probe_.shtp_seq = probe_.header[3];
    probe_.probe_status = header_is_plausible(probe_.shtp_len, probe_.shtp_channel)
                              ? BNO085_PROBE_SPI_READ_OK
                              : BNO085_PROBE_BAD_HEADER;
}

/* ── SHTP packet read / write ───────────────────────────────────────────── */

static bool spi_read_buf(uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        buf[i] = 0x00u;
    }
    cs_assert();
    sleep_us(2);
    const int rc = spi_write_read_blocking(spi_, buf, buf, len);
    cs_deassert();
    return rc == (int)len;
}

/* Read one complete SHTP frame using the partial-read flow described by the
 * SHTP spec and used by reference BNO08x SPI drivers: read 4 header bytes,
 * wait for INT to assert again, then read the full packet length. */
static bool shtp_read_packet_ms(uint32_t timeout_ms) {
    if (!wake_for_transaction(timeout_ms)) {
        return false;
    }

    if (!spi_read_buf(rx_buf_, SHTP_HDR_LEN)) {
        return false;
    }

    uint16_t pkt_len = ((uint16_t)rx_buf_[0] | ((uint16_t)rx_buf_[1] << 8)) & 0x7FFFu;

    if (pkt_len < SHTP_HDR_LEN || pkt_len > SHTP_RX_BUF_LEN) {
        return false;
    }

    if (!wait_int_low(timeout_ms)) {
        return false;
    }

    if (!spi_read_buf(rx_buf_, pkt_len)) {
        return false;
    }

    shtp_rx_count_++;
    return true;
}

static bool shtp_send(uint8_t channel, const uint8_t *payload, uint16_t payload_len) {
    if (payload_len + SHTP_HDR_LEN > SHTP_TX_BUF_LEN) {
        return false;
    }
    const uint16_t total = (uint16_t)(payload_len + SHTP_HDR_LEN);
    tx_buf_[0] = (uint8_t)(total & 0xFFu);
    tx_buf_[1] = (uint8_t)((total >> 8) & 0x7Fu);
    tx_buf_[2] = channel;
    tx_buf_[3] = out_seq_[channel]++;
    memcpy(&tx_buf_[SHTP_HDR_LEN], payload, payload_len);
    if (!wake_for_transaction(500u)) {
        return false;
    }
    return spi_write_buf(tx_buf_, total);
}

/* ── Sensor report parsing ──────────────────────────────────────────────── */

static void parse_rotation_vector(const uint8_t *r) {
    latest_.qx = (float)read_i16_le(&r[4])  * QUAT_SCALE;
    latest_.qy = (float)read_i16_le(&r[6])  * QUAT_SCALE;
    latest_.qz = (float)read_i16_le(&r[8])  * QUAT_SCALE;
    latest_.qw = (float)read_i16_le(&r[10]) * QUAT_SCALE;
    latest_.cal_status = (uint8_t)(r[2] & 0x03u);
    latest_.valid    = true;
    report_updated_  = true;
    sample_count_++;
    last_report_id_  = REPORT_ROTATION_VECTOR;
}

static void parse_linear_accel(const uint8_t *r) {
    latest_.ax      = (float)read_i16_le(&r[4]) * ACCEL_SCALE;
    latest_.ay      = (float)read_i16_le(&r[6]) * ACCEL_SCALE;
    latest_.az      = (float)read_i16_le(&r[8]) * ACCEL_SCALE;
    last_report_id_ = REPORT_LINEAR_ACCEL;
}

static uint8_t report_byte_len(uint8_t id) {
    switch (id) {
    case REPORT_ROTATION_VECTOR:  return 14u;
    case REPORT_LINEAR_ACCEL:     return 10u;
    case REPORT_BASE_TIMESTAMP:
    case REPORT_TIMESTAMP_REBASE: return 5u;
    default:                      return 0u;
    }
}

static void handle_input_payload(const uint8_t *payload, uint16_t len) {
    for (uint16_t c = 0; c < len; ) {
        const uint8_t id   = payload[c];
        const uint8_t rlen = report_byte_len(id);
        if (rlen == 0u || (uint16_t)(c + rlen) > len) {
            c++;
            continue;
        }
        if (id == REPORT_ROTATION_VECTOR) {
            parse_rotation_vector(&payload[c]);
        } else if (id == REPORT_LINEAR_ACCEL) {
            parse_linear_accel(&payload[c]);
        }
        c = (uint16_t)(c + rlen);
    }
}

/* ── SHTP packet processor ──────────────────────────────────────────────── */

static void process_packet(void) {
    const uint16_t pkt_len  = ((uint16_t)rx_buf_[0] | ((uint16_t)rx_buf_[1] << 8)) & 0x7FFFu;
    const uint16_t pay_len  = pkt_len - SHTP_HDR_LEN;
    const uint8_t  channel  = rx_buf_[2];
    const uint8_t *payload  = &rx_buf_[SHTP_HDR_LEN];

    if (pay_len == 0u) {
        return;
    }

    last_channel_  = channel;
    last_payload0_ = payload[0];

    if (channel == SHTP_CH_EXECUTABLE &&
        pay_len  >= 1u &&
        payload[0] == EXECUTABLE_RESET_DONE) {
        /* Re-enable reports after a reset notification. During bring-up, the
         * BNO085 can leave an executable reset packet in the queue after we
         * have already parsed input reports, so do not wipe the last sample. */
        if (init_stage_ != BNO085_STAGE_READY) {
            ready_ = false;
        }
        enable_pending_ = true;
        return;
    }

    if (channel == SHTP_CH_INPUT) {
        ch3_rx_count_++;
        handle_input_payload(payload, pay_len);
    }
}

/* ── Init helpers ───────────────────────────────────────────────────────── */

static void service_until(uint32_t n, uint32_t timeout_ms) {
    for (uint32_t i = 0; i < n; i++) {
        if (!shtp_read_packet_ms(timeout_ms)) {
            break;
        }
        process_packet();
    }
}

static bool enable_report(uint8_t report_id, uint32_t interval_us) {
    uint8_t cmd[17];
    memset(cmd, 0, sizeof(cmd));
    cmd[0] = SHTP_CMD_SET_FEATURE;
    cmd[1] = report_id;
    cmd[5] = (uint8_t)(interval_us         & 0xFFu);
    cmd[6] = (uint8_t)((interval_us >>  8) & 0xFFu);
    cmd[7] = (uint8_t)((interval_us >> 16) & 0xFFu);
    cmd[8] = (uint8_t)((interval_us >> 24) & 0xFFu);
    for (int attempt = 0; attempt < 5; attempt++) {
        if (shtp_send(SHTP_CH_SENSOR_HUB, cmd, sizeof(cmd))) {
            return true;
        }
        sleep_ms(10);
    }
    return false;
}

static void hardware_reset(void) {
    wake_release();
    gpio_put(VIGIA_BNO085_RST_PIN, 0);
    sleep_ms(20);
    gpio_put(VIGIA_BNO085_RST_PIN, 1);
    sleep_ms(400);   /* BNO085 spec: boot up to ~400 ms */
}

static bool wait_for_reset_complete(uint32_t timeout_ms) {
    absolute_time_t dl = make_timeout_time_ms(timeout_ms);
    while (absolute_time_diff_us(get_absolute_time(), dl) > 0) {
        if (!shtp_read_packet_ms(100u)) {
            sleep_ms(2);
            continue;
        }
        const uint16_t pkt_len = ((uint16_t)rx_buf_[0] | ((uint16_t)rx_buf_[1] << 8)) & 0x7FFFu;
        const uint8_t *payload = &rx_buf_[SHTP_HDR_LEN];
        const uint16_t pay_len = pkt_len - SHTP_HDR_LEN;
        if (rx_buf_[2] == SHTP_CH_EXECUTABLE &&
            pay_len     >= 1u                &&
            payload[0]  == EXECUTABLE_RESET_DONE) {
            return true;
        }
        process_packet();
        sleep_ms(2);
    }
    return false;
}

static bool wait_for_first_sample(uint32_t timeout_ms) {
    absolute_time_t dl = make_timeout_time_ms(timeout_ms);
    while (sample_count_ == 0u && absolute_time_diff_us(get_absolute_time(), dl) > 0) {
        if (gpio_get(VIGIA_BNO085_INT_PIN) == 0u) {
            service_until(1u, 20u);
        } else {
            sleep_ms(5);
        }
    }
    return sample_count_ > 0u;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void bno085_init(spi_inst_t *spi) {
    spi_ = spi;
    memset(&latest_,  0, sizeof(latest_));
    memset(out_seq_,  0, sizeof(out_seq_));
    report_updated_ = false;
    ready_          = false;
    enable_pending_ = false;
    sample_count_   = 0;
    shtp_rx_count_  = 0;
    ch3_rx_count_   = 0;
    init_stage_     = BNO085_STAGE_NONE;
    last_report_id_ = 0;
    last_channel_   = 0;
    last_payload0_  = 0;
    memset(&probe_, 0, sizeof(probe_));

    gpio_init(VIGIA_BNO085_CS_PIN);
    gpio_set_dir(VIGIA_BNO085_CS_PIN, GPIO_OUT);
    gpio_put(VIGIA_BNO085_CS_PIN, 1);

    gpio_init(VIGIA_BNO085_RST_PIN);
    gpio_set_dir(VIGIA_BNO085_RST_PIN, GPIO_OUT);
    gpio_put(VIGIA_BNO085_RST_PIN, 1);

    gpio_init(VIGIA_BNO085_WAKE_PIN);
    gpio_set_dir(VIGIA_BNO085_WAKE_PIN, GPIO_OUT);
    wake_release();

    gpio_init(VIGIA_BNO085_INT_PIN);
    gpio_set_dir(VIGIA_BNO085_INT_PIN, GPIO_IN);
    gpio_pull_up(VIGIA_BNO085_INT_PIN);

    gpio_set_function(VIGIA_BNO085_SCK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(VIGIA_BNO085_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(VIGIA_BNO085_MISO_PIN, GPIO_FUNC_SPI);
    gpio_pull_up(VIGIA_BNO085_MISO_PIN);

    spi_init(spi_, VIGIA_BNO085_SPI_BAUD);
    spi_set_format(spi_, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    hardware_reset();
    probe_spi_header_after_reset();

    /* The probe intentionally reads only the header. Reset again so the normal
     * initialization path receives a fresh, complete boot packet. */
    hardware_reset();

    if (!wait_for_reset_complete(2000u)) {
        init_stage_ = BNO085_STAGE_BOOT_TIMEOUT;
        return;
    }

    if (!enable_report(REPORT_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
        init_stage_ = BNO085_STAGE_ENABLE_ROT;
        return;
    }
    service_until(8u, 100u);

    if (!enable_report(REPORT_LINEAR_ACCEL, REPORT_INTERVAL_US)) {
        init_stage_ = BNO085_STAGE_ENABLE_ACCEL;
        return;
    }
    service_until(8u, 100u);

    if (!wait_for_first_sample(3000u)) {
        init_stage_     = BNO085_STAGE_NO_DATA;
        enable_pending_ = true;   /* poll loop will retry the enables */
        return;
    }

    ready_      = true;
    init_stage_ = BNO085_STAGE_READY;
}

void bno085_poll(void) {
    if (spi_ == NULL) {
        return;
    }

    if (enable_pending_) {
        enable_pending_ = false;
        enable_report(REPORT_ROTATION_VECTOR, REPORT_INTERVAL_US);
        enable_report(REPORT_LINEAR_ACCEL,    REPORT_INTERVAL_US);
    }

    /* Only read when INT is currently low.  This guarantees that
     * wake_for_transaction() inside shtp_read_packet_ms() will find INT
     * already low and return immediately — WAKE is never toggled during normal
     * polling, preventing spurious write-initiation transactions. */
    for (uint32_t i = 0; i < 8u; i++) {
        if (gpio_get(VIGIA_BNO085_INT_PIN) != 0u) {
            break;
        }
        if (!shtp_read_packet_ms(50u)) {
            break;
        }
        process_packet();
        if (!ready_ && sample_count_ > 0u) {
            ready_      = true;
            init_stage_ = BNO085_STAGE_READY;
        }
    }
}

bool bno085_get_report(bno085_report_t *out) {
    if (out == NULL) {
        return false;
    }
    *out = latest_;
    const bool updated = report_updated_;
    report_updated_ = false;
    return updated;
}

void bno085_get_probe(bno085_probe_t *out) {
    if (out != NULL) {
        *out = probe_;
    }
}

bool     bno085_ready(void)          { return ready_; }
uint32_t bno085_sample_count(void)   { return sample_count_; }
uint8_t  bno085_init_stage(void)     { return init_stage_; }
uint32_t bno085_shtp_rx_count(void)  { return shtp_rx_count_; }
uint8_t  bno085_last_report_id(void) { return last_report_id_; }
uint32_t bno085_ch3_rx_count(void)   { return ch3_rx_count_; }
uint8_t  bno085_last_channel(void)   { return last_channel_; }
uint8_t  bno085_last_payload0(void)  { return last_payload0_; }

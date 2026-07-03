/**
 * atecc608a_driver.c — ATECC608A Secure Element wrappers for Vigia Phase 2
 *
 * Two build modes:
 *   VIGIA_PHASE2_STUB=1  — no I2C; sha/sign zero-fill outputs (validate pipeline)
 *   VIGIA_PHASE2_STUB=0  — real cryptoauthlib calls over I2C1 GP2/GP3
 *
 * cryptoauthlib must be compiled with ATCA_NO_HEAP to prevent heap use.
 */

#include "atecc608a_driver.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <string.h>

/* Compile-time size assertions */
_Static_assert(sizeof(EtHashInput)    == ET_HASH_INPUT_SIZE,    "EtHashInput must be 96 bytes");
_Static_assert(sizeof(SignedEtPacket) == SIGNED_ET_PACKET_SIZE, "SignedEtPacket must be 173 bytes");

/* ── I2C1 pin mapping ─────────────────────────────────────────────────────── */
#define VIGIA_ATCA_I2C      i2c1
#define VIGIA_ATCA_SDA_PIN  2
#define VIGIA_ATCA_SCL_PIN  3
#define VIGIA_ATCA_I2C_HZ   100000   /* 100 kHz — reliable on jumper/breadboard; use 400k on PCB */
#define VIGIA_ATCA_ADDR     0xC0     /* 8-bit I2C addr (7-bit 0x60); ADDR pin → GND */

/* ── LED blink error codes ────────────────────────────────────────────────── */
static void blink_panic(uint32_t code) {
    const uint led_pin = PICO_DEFAULT_LED_PIN;
    while (true) {
        for (uint32_t i = 0; i < code; ++i) {
            gpio_put(led_pin, 1); sleep_ms(200);
            gpio_put(led_pin, 0); sleep_ms(200);
        }
        sleep_ms(800);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
   STUB MODE — SE not yet wired; zero-fills hash and signature
   ══════════════════════════════════════════════════════════════════════════ */
#if defined(VIGIA_PHASE2_STUB) && (VIGIA_PHASE2_STUB == 1)

int vigia_atca_init(void) {
    /* No I2C init needed in stub mode */
    return 0;
}

int vigia_atca_read_device_id(uint8_t device_id[16]) {
    if (!device_id) {
        return -1;
    }
    memset(device_id, 0, 16);
    return 0;
}

int vigia_atca_sha(const uint8_t *input, size_t len, uint8_t *hash_out) {
    (void)input; (void)len;
    memset(hash_out, 0x00, 32);
    return 0;
}

int vigia_atca_sign(const uint8_t *hash, uint8_t *sig_out) {
    (void)hash;
    memset(sig_out, 0x00, 64);
    return 0;
}

int vigia_atca_provision_slot0(uint8_t pubkey[64], bool *generated_new) {
    (void)pubkey;
    if (generated_new) {
        *generated_new = false;
    }
    return -1;
}

/* ══════════════════════════════════════════════════════════════════════════
   LIVE MODE — real ATECC608A over I2C1
   ══════════════════════════════════════════════════════════════════════════ */
#else

#include <stdbool.h>

#include "atecc608a_provision.h"
#include "cryptoauthlib.h"

/* cryptoauthlib HAL config for Pico I2C */
static ATCAIfaceCfg g_atca_cfg = {
    .iface_type  = ATCA_I2C_IFACE,
    .devtype     = ATECC608,
    .atcai2c     = {
        .slave_address = VIGIA_ATCA_ADDR,
        .bus           = 1,
        .baud          = VIGIA_ATCA_I2C_HZ,
    },
    .wake_delay  = 1500,
    .rx_retries  = 3,
};

int vigia_atca_init(void) {
    i2c_init(VIGIA_ATCA_I2C, VIGIA_ATCA_I2C_HZ);
    gpio_set_function(VIGIA_ATCA_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(VIGIA_ATCA_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(VIGIA_ATCA_SDA_PIN);
    gpio_pull_up(VIGIA_ATCA_SCL_PIN);

    ATCA_STATUS status = atcab_init(&g_atca_cfg);
    if (status != ATCA_SUCCESS) {
#if defined(VIGIA_ATCA_NO_PANIC) && (VIGIA_ATCA_NO_PANIC == 1)
        return (int)status;
#else
        blink_panic(3);  /* 3 blinks = SE init failure */
#endif
    }
    return 0;
}

int vigia_atca_read_device_id(uint8_t device_id[16]) {
    if (!device_id) {
        return -1;
    }
    memset(device_id, 0, 16);

    uint8_t atca_serial[ATCA_SERIAL_NUM_SIZE];
    const ATCA_STATUS status = atcab_read_serial_number(atca_serial);
    if (status != ATCA_SUCCESS) {
#if defined(VIGIA_ATCA_NO_PANIC) && (VIGIA_ATCA_NO_PANIC == 1)
        return (int)status;
#else
        blink_panic(2); /* serial read failed — SE may need provisioning */
#endif
    }
    memcpy(device_id, atca_serial, ATCA_SERIAL_NUM_SIZE);
    return 0;
}

int vigia_atca_sha(const uint8_t *input, size_t len, uint8_t *hash_out) {
    ATCA_STATUS s = atcab_sha((uint16_t)len, input, hash_out);
    return (s == ATCA_SUCCESS) ? 0 : (int)s;
}

int vigia_atca_sign(const uint8_t *hash, uint8_t *sig_out) {
    /* Private key in slot 0; signature is R||S, 64 bytes */
    ATCA_STATUS s = atcab_sign(0, hash, sig_out);
    return (s == ATCA_SUCCESS) ? 0 : (int)s;
}

int vigia_atca_provision_slot0(uint8_t pubkey[64], bool *generated_new) {
    return vigia_atca_provision_slot0_verbose(
        pubkey, generated_new, NULL, NULL, NULL);
}

#endif /* VIGIA_PHASE2_STUB */

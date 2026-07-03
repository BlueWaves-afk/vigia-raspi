/**
 * vigia_pico_provision — ATECC608 slot-0 provisioning over USB CDC.
 *
 * Repeats its result every 5 s so you can open the serial port at any time.
 * Run: python3 tools/pico_provision_read.py --port /dev/cu.usbmodem*
 */

#include "atecc608a_driver.h"
#include "atecc608a_provision.h"
#include "cryptoauthlib.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const uint LED_PIN = PICO_DEFAULT_LED_PIN;

static void provision_log(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fflush(stdout);
}

static void print_hex(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        printf("%02x", data[i]);
    }
    fflush(stdout);
}

static void print_chip_status(void) {
    bool config_locked = false;
    bool data_locked   = false;
    bool slot_locked   = false;
    bool slot_private  = false;

    if (atcab_is_config_locked(&config_locked) == ATCA_SUCCESS) {
        provision_log("VIGIA_PROVISION config_locked=%u\n", config_locked ? 1u : 0u);
    }
    if (atcab_is_data_locked(&data_locked) == ATCA_SUCCESS) {
        provision_log("VIGIA_PROVISION data_locked=%u\n", data_locked ? 1u : 0u);
    }
    if (atcab_is_slot_locked(0, &slot_locked) == ATCA_SUCCESS) {
        provision_log("VIGIA_PROVISION slot0_locked=%u\n", slot_locked ? 1u : 0u);
    }
    if (atcab_is_private(0, &slot_private) == ATCA_SUCCESS) {
        provision_log("VIGIA_PROVISION slot0_private=%u\n", slot_private ? 1u : 0u);
    }
}

static bool sign_self_test(void) {
    static const uint8_t k_test_hash[32] = {
        0x5a, 0x8d, 0x73, 0x9f, 0x2e, 0x4c, 0x1b, 0x90,
        0x6f, 0x33, 0x8a, 0x44, 0xbb, 0x19, 0x7c, 0x02,
        0x88, 0x51, 0x63, 0x4d, 0x0a, 0x9f, 0x5e, 0x11,
        0x42, 0x77, 0x93, 0x20, 0x61, 0x8e, 0x4f, 0x7b,
    };
    uint8_t sig[64];
    memset(sig, 0, sizeof(sig));
    if (vigia_atca_sign(k_test_hash, sig) != 0) {
        return false;
    }
    for (size_t i = 0; i < sizeof(sig); ++i) {
        if (sig[i] != 0) return true;
    }
    return false;
}

/* ------------------------------------------------------------------ */

typedef enum {
    STATE_INIT_FAIL,
    STATE_SERIAL_FAIL,
    STATE_GENKEY_FAIL,
    STATE_SIGN_FAIL,
    STATE_OK,
} ProvState;

typedef struct {
    ProvState state;
    int       rc;
    int       get_pubkey_rc;
    int       genkey_rc;
    int       config_write_rc;
    uint8_t   serial[16]; /* vigia_atca_read_device_id writes 16 bytes */
    uint8_t   pubkey[64];
    bool      generated;
    bool      config_locked;
    bool      data_locked;
    bool      slot0_locked;
} ProvResult;

/* Blink pattern: n short blinks, long pause. */
static void blink_pattern(int count, uint32_t on_ms, uint32_t off_ms, uint32_t pause_ms) {
    for (int i = 0; i < count; ++i) {
        gpio_put(LED_PIN, 1); sleep_ms(on_ms);
        gpio_put(LED_PIN, 0); sleep_ms(off_ms);
    }
    sleep_ms(pause_ms);
}

static void print_result(const ProvResult *r) {
    switch (r->state) {
    case STATE_INIT_FAIL:
        provision_log("VIGIA_PROVISION status=error step=init rc=%d\n", r->rc);
        provision_log("VIGIA_PROVISION hint=I2C init failed -- check GP2/GP3 wiring, addr 0x60\n");
        break;

    case STATE_SERIAL_FAIL:
        provision_log("VIGIA_PROVISION status=error step=serial rc=%d\n", r->rc);
        provision_log("VIGIA_PROVISION hint=chip not responding on I2C\n");
        break;

    case STATE_GENKEY_FAIL:
        provision_log("VIGIA_PROVISION status=error step=genkey rc=%d\n", r->rc);
        provision_log("VIGIA_PROVISION get_pubkey_rc=%d genkey_rc=%d config_lock_rc=%d\n",
                      r->get_pubkey_rc, r->genkey_rc, r->config_write_rc);
        provision_log("VIGIA_PROVISION config_locked=%u data_locked=%u slot0_locked=%u\n",
                      r->config_locked ? 1u : 0u,
                      r->data_locked   ? 1u : 0u,
                      r->slot0_locked  ? 1u : 0u);
        provision_log("VIGIA_PROVISION serial=");
        print_hex(r->serial, 9);
        provision_log("\n");
        if (r->config_locked && r->genkey_rc != 0) {
            provision_log("VIGIA_PROVISION hint=config_locked=1 but genkey still fails -- slot 0 KeyConfig incompatible\n");
        } else if (!r->config_locked && r->config_write_rc != 0) {
            provision_log("VIGIA_PROVISION hint=config zone write or lock failed -- check I2C and chip state\n");
        } else {
            provision_log("VIGIA_PROVISION hint=genkey failed after config lock -- chip may need full reset\n");
        }
        break;

    case STATE_SIGN_FAIL:
        provision_log("VIGIA_PROVISION status=error step=sign rc=0\n");
        provision_log("VIGIA_PROVISION pubkey=");
        print_hex(r->pubkey, 64);
        provision_log("\n");
        provision_log("VIGIA_PROVISION hint=key generated but sign self-test returned all-zero sig\n");
        break;

    case STATE_OK:
        provision_log("VIGIA_PROVISION serial=");
        print_hex(r->serial, 9);
        provision_log("\n");
        provision_log("VIGIA_PROVISION pubkey=");
        print_hex(r->pubkey, 64);
        provision_log("\n");
        provision_log("VIGIA_PROVISION generated=%u\n", r->generated ? 1u : 0u);
        provision_log("VIGIA_PROVISION sign_test=ok\n");
        provision_log("VIGIA_PROVISION status=ok slot=0\n");
        provision_log("VIGIA_PROVISION next=reflash vigia_pico_phase2_live.uf2\n");
        break;
    }
}

/* ------------------------------------------------------------------ */

int main(void) {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    /* Wait for USB CDC host to attach. Fixed delay works with all tools
     * (no dependency on DTR state). */
    sleep_ms(3000);

    provision_log("VIGIA_PROVISION boot fw=provision\n");

    ProvResult result;
    memset(&result, 0, sizeof(result));

    /* --- Init -------------------------------------------------------- */
    result.rc = vigia_atca_init();
    if (result.rc != 0) {
        result.state = STATE_INIT_FAIL;
        goto loop;
    }

    /* --- Chip info --------------------------------------------------- */
    {
        uint8_t revision[4];
        if (atcab_info(revision) == ATCA_SUCCESS) {
            provision_log("VIGIA_PROVISION revision=%02x%02x%02x%02x\n",
                          revision[0], revision[1], revision[2], revision[3]);
        }
    }

    provision_log("VIGIA_PROVISION begin\n");

    /* --- Serial number ----------------------------------------------- */
    result.rc = vigia_atca_read_device_id(result.serial);
    if (result.rc != 0) {
        result.state = STATE_SERIAL_FAIL;
        goto loop;
    }

    /* Read zone-lock state into result so it's always in the loop output. */
    atcab_is_config_locked(&result.config_locked);
    atcab_is_data_locked(&result.data_locked);
    atcab_is_slot_locked(0, &result.slot0_locked);

    print_chip_status();

    /* --- Key provisioning -------------------------------------------- */
    result.rc = vigia_atca_provision_slot0_verbose(
        result.pubkey, &result.generated,
        &result.get_pubkey_rc, &result.genkey_rc, &result.config_write_rc);
    if (result.rc != 0) {
        result.state = STATE_GENKEY_FAIL;
        goto loop;
    }

    /* --- Sign self-test ---------------------------------------------- */
    if (!sign_self_test()) {
        result.state = STATE_SIGN_FAIL;
        goto loop;
    }

    result.state = STATE_OK;

loop:
    /* Print result immediately, then repeat every 5 s so the host can
     * open the port at any time and still read the full output. */
    while (true) {
        print_result(&result);

        /* LED pattern reflects outcome */
        switch (result.state) {
        case STATE_INIT_FAIL:
            /* 3 quick blinks = I2C / init error */
            blink_pattern(3, 150, 150, 700);
            break;
        case STATE_SERIAL_FAIL:
            /* 2 blinks = chip not responding */
            blink_pattern(2, 200, 200, 800);
            break;
        case STATE_GENKEY_FAIL:
            /* Slow 1 Hz = genkey error */
            for (int i = 0; i < 5; ++i) {
                gpio_put(LED_PIN, 1); sleep_ms(500);
                gpio_put(LED_PIN, 0); sleep_ms(500);
            }
            break;
        case STATE_SIGN_FAIL:
            /* 4 rapid blinks = sign test fail */
            blink_pattern(4, 100, 100, 600);
            break;
        case STATE_OK:
            /* Solid ON, brief off every 5 s */
            gpio_put(LED_PIN, 1);
            sleep_ms(4800);
            gpio_put(LED_PIN, 0);
            sleep_ms(200);
            break;
        }
    }

    return 0;
}

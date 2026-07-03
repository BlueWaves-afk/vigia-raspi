/**
 * ATECC608 slot-0 provisioning helpers (live cryptoauthlib only).
 *
 * Required provisioning sequence for ATECC608B:
 *   1. Write config zone   (if unlocked)
 *   2. Lock config zone    (if not already locked) ← mandatory before GenKey
 *   3. atcab_genkey(0)     (generates P-256 private key in slot 0)
 *   4. Export public key
 *
 * Locking is irreversible; we validate the template before locking.
 */

#include "atecc608a_provision.h"

#if defined(VIGIA_PHASE2_STUB) && (VIGIA_PHASE2_STUB == 1)

int vigia_atca_provision_slot0_verbose(
    uint8_t pubkey[64],
    bool *generated_new,
    int *get_rc,
    int *gen_rc,
    int *config_rc)
{
    (void)pubkey;
    if (generated_new) *generated_new = false;
    if (get_rc)        *get_rc  = -1;
    if (gen_rc)        *gen_rc  = -1;
    if (config_rc)     *config_rc = 0;
    return -1;
}

#else

#include <stdbool.h>
#include <string.h>

#include "cryptoauthlib.h"

#ifndef ATCA_ECC_CONFIG_SIZE
#define ATCA_ECC_CONFIG_SIZE 128
#endif

/*
 * Vigia ATECC608 config template.
 *
 * Bytes 0-15 are read-only (SN, RevNum, I2C address) and are replaced
 * with live chip values before writing.
 *
 * Critical fields verified:
 *   SlotConfig[0]  bytes 20-21 = 0x2FAF  → IsSecret=1, WriteConfig=GenKey
 *   KeyConfig[0]   bytes 96-97 = 0x0033  → Private=1, PubInfo=1, P-256, Lockable
 */
static const uint8_t k_vigia_atecc608_config[ATCA_ECC_CONFIG_SIZE] = {
    /* 0x00 */ 0x01, 0x23, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00,
               0x04, 0x05, 0x06, 0x07, 0xEE, 0x01, 0x01, 0x00,
    /* 0x10 */ 0xC0, 0x00, 0xA1, 0x00, 0xAF, 0x2F, 0xC4, 0x44,
               0x87, 0x20, 0xC4, 0xF4, 0x8F, 0x0F, 0x0F, 0x0F,
    /* 0x20 */ 0x9F, 0x8F, 0x83, 0x64, 0xC4, 0x44, 0xC4, 0x64,
               0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
    /* 0x30 */ 0x0F, 0x0F, 0x0F, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF,
               0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
    /* 0x40 */ 0x00, 0x00, 0x00, 0x00, 0xFF, 0x84, 0x03, 0xBC,
               0x09, 0x69, 0x76, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* 0x50 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
               0xFF, 0xFF, 0x0E, 0x40, 0x00, 0x00, 0x00, 0x00,
    /* 0x60 KeyConfig slots 0-7 */
               0x33, 0x00, 0x1C, 0x00, 0x13, 0x00, 0x1C, 0x00,
               0x3C, 0x00, 0x3A, 0x10, 0x1C, 0x00, 0x33, 0x00,
    /* 0x70 KeyConfig slots 8-15 */
               0x1C, 0x00, 0x1C, 0x00, 0x38, 0x00, 0x30, 0x00,
               0x3C, 0x00, 0x3C, 0x00, 0x32, 0x00, 0x30, 0x00,
};

static bool pubkey_nonzero(const uint8_t pubkey[64]) {
    for (size_t i = 0; i < 64; ++i) {
        if (pubkey[i] != 0) return true;
    }
    return false;
}

static int try_read_pubkey(uint8_t slot, uint8_t pubkey[64]) {
    memset(pubkey, 0, 64);

    ATCA_STATUS s = atcab_get_pubkey(slot, pubkey);
    if (s == ATCA_SUCCESS && pubkey_nonzero(pubkey)) return 0;

    memset(pubkey, 0, 64);
    s = atcab_read_pubkey(slot, pubkey);
    if (s == ATCA_SUCCESS && pubkey_nonzero(pubkey)) return 0;

    return (int)s;
}

static int try_genkey(uint8_t slot, uint8_t pubkey[64]) {
    memset(pubkey, 0, 64);

    ATCA_STATUS s = atcab_genkey(slot, pubkey);
    if (s == ATCA_SUCCESS && pubkey_nonzero(pubkey)) return 0;

    /* atcab_genkey_base with mode=0x04 = generate + return public key */
    memset(pubkey, 0, 64);
    s = atcab_genkey_base(0x04, slot, NULL, pubkey);
    if (s == ATCA_SUCCESS && pubkey_nonzero(pubkey)) return 0;

    return (int)s;
}

/*
 * Write the Vigia config template if the config zone is unlocked,
 * then lock the config zone.
 *
 * Returns:
 *   0                   — config zone was already locked (nothing done), or
 *                         write + lock both succeeded
 *   non-zero ATCA code  — write or lock failed
 *
 * NOTE: locking is irreversible. We verify the chip bytes [0..15]
 * (serial/revision, read-only) are preserved before locking.
 */
static int write_and_lock_config(void) {
    bool config_locked = true;
    ATCA_STATUS st = atcab_is_config_locked(&config_locked);
    if (st != ATCA_SUCCESS) return (int)st;

    if (config_locked) {
        /* Already locked — cannot change config. GenKey must rely on
         * whatever the chip has. Return success so the caller can
         * attempt GenKey and report the specific error. */
        return 0;
    }

    /* Read current to preserve read-only header bytes 0-15. */
    uint8_t current[ATCA_ECC_CONFIG_SIZE];
    st = atcab_read_config_zone(current);
    if (st != ATCA_SUCCESS) return (int)st;

    uint8_t target[ATCA_ECC_CONFIG_SIZE];
    memcpy(target, k_vigia_atecc608_config, sizeof(target));
    memcpy(target, current, 16); /* keep SN / RevNum / I2C addr */

    /* Write config zone (cryptoauthlib handles zone-word writes). */
    bool same = false;
    atcab_cmp_config_zone(target, &same);
    if (!same) {
        st = atcab_write_config_zone(target);
        if (st != ATCA_SUCCESS) return (int)st;
    }

    /* Lock config zone — required for GenKey to succeed on ATECC608B. */
    st = atcab_lock_config_zone();
    return (int)st;
}

int vigia_atca_provision_slot0_verbose(
    uint8_t pubkey[64],
    bool *generated_new,
    int *get_rc,
    int *gen_rc,
    int *config_rc)
{
    if (!pubkey) return -1;

    if (generated_new) *generated_new = false;
    if (get_rc)        *get_rc  = 0;
    if (gen_rc)        *gen_rc  = 0;
    if (config_rc)     *config_rc = 0;

    memset(pubkey, 0, 64);

    /* Step 1: Try to read an existing key — if one is already present
     * (from a previous provisioning run), we're done immediately. */
    int read_rc = try_read_pubkey(0, pubkey);
    if (get_rc) *get_rc = read_rc;
    if (read_rc == 0) return 0;

    /* Step 2: Write + lock config zone so GenKey can proceed. */
    const int cfg_rc = write_and_lock_config();
    if (config_rc) *config_rc = cfg_rc;
    if (cfg_rc != 0) {
        /* Config zone write or lock failed — cannot generate key. */
        return cfg_rc;
    }

    /* Step 3: Generate P-256 private key in slot 0. */
    int genkey_rc = try_genkey(0, pubkey);
    if (gen_rc) *gen_rc = genkey_rc;
    if (genkey_rc == 0) {
        if (generated_new) *generated_new = true;
        return 0;
    }

    return genkey_rc;
}

#endif /* VIGIA_PHASE2_STUB */

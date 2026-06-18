#pragma once
/**
 * ATECC608A Secure Element driver — Vigia Phase 2
 *
 * Wraps cryptoauthlib with ATCA_NO_HEAP.
 * All calls are blocking; never invoke from ISR context.
 *
 * I2C1 wiring:
 *   GP2 = SDA, GP3 = SCL
 *   ADDR pin → GND → I2C address 0x60
 *   Pull-ups: 4.7 kΩ to 3.3 V (external — not on Pico board)
 *   Speed: 400 kHz (Fast Mode)
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── EtHashInput — 96-byte packed struct hashed by ATECC608A ─────────────── */

#pragma pack(push, 1)
typedef struct {
    uint8_t  device_id[16];   /* provisioned in ATECC608A data zone at factory */
    uint64_t timestamp_us;    /* BNO085 SHTP 64-bit µs counter */
    uint32_t sequence;        /* global monotonic frame counter */
    float    qw, qx, qy, qz; /* quaternion (Q14 → float, kQuatScale=1/16384) */
    float    ax, ay, az;      /* linear accel (Q8 → float, kAccelScale=1/256) */
    uint8_t  cal_status;
    uint8_t  _pad0[3];        /* explicit pad — align latitude to 8-byte boundary */
    double   latitude;        /* NEO-M8N NAV-PVT lat@28, UBX units → degrees */
    double   longitude;       /* NEO-M8N NAV-PVT lon@24, UBX units → degrees */
    float    speed_ms;        /* NAV-PVT gSpeed@60 mm/s → m/s */
    uint8_t  fix_type;        /* NAV-PVT fixType@20 */
    uint8_t  satellites;      /* NAV-PVT numSV@23 */
    uint8_t  _pad1[2];        /* explicit end-of-struct pad */
    uint8_t  _pad2[12];       /* reserved — keeps EtHashInput at 96 bytes for SHA */
} EtHashInput;
#pragma pack(pop)

/* Verified at compile time in atecc608a_driver.c */
#define ET_HASH_INPUT_SIZE 96

/* ── SignedEtPacket — 173-byte packed struct sent over USB CDC (COBS) ─────── */

#pragma pack(push, 1)
typedef struct {
    /* Header — 2 bytes */
    uint8_t  magic;           /* 0xE7 — Vigia DePIN frame marker */
    uint8_t  version;         /* 0x02 — Phase 2 protocol version */

    /* IMU — 40 bytes */
    uint64_t timestamp_us;
    uint32_t sequence;
    float    qw, qx, qy, qz;
    float    ax, ay, az;
    uint8_t  cal_status;
    uint8_t  _imu_pad[3];

    /* GPS — 27 bytes */
    double   latitude;
    double   longitude;
    float    speed_ms;
    uint8_t  fix_type;
    uint8_t  satellites;
    uint8_t  _gps_pad[1];

    /* Signing — 96 bytes */
    uint8_t  et_hash[32];     /* SHA-256 of EtHashInput via atcab_sha() */
    uint8_t  ecdsa_sig[64];   /* secp256r1 ECDSA-SHA256 via atcab_sign() slot 0 */

    /* Wire padding — keeps total frame at 173 bytes (ABI with Pi decoder) */
    uint8_t  _wire_pad[8];
} SignedEtPacket;
#pragma pack(pop)

#define SIGNED_ET_PACKET_SIZE 173

/* ── API ──────────────────────────────────────────────────────────────────── */

/**
 * Call once at boot after i2c1 is initialised.
 * Returns 0 on success; panics with LED blink code on hard failure.
 * In VIGIA_PHASE2_STUB mode: always returns 0, performs no I2C.
 */
int vigia_atca_init(void);

/**
 * Compute SHA-256 of `input` (len bytes) into `hash_out` (32 bytes).
 * Blocking — ~40 ms on I2C 400 kHz. Do NOT call from ISR.
 * Returns 0 on success, non-zero on ATCA error.
 */
int vigia_atca_sha(const uint8_t *input, size_t len, uint8_t *hash_out);

/**
 * Sign `hash` (32 bytes) with device private key in slot 0.
 * Output in `sig_out` (64 bytes, R||S format).
 * Blocking — ~57 ms. Do NOT call from ISR.
 * Returns 0 on success, non-zero on ATCA error.
 */
int vigia_atca_sign(const uint8_t *hash, uint8_t *sig_out);

#ifdef __cplusplus
}
#endif

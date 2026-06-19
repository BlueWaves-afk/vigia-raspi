---
title: "ECDSA Signing (ATECC608A secp256r1)"
type: security
tags: [security, transport]
source: firmware/src/atecc608a_driver.h
related: ["[[atecc608a]]", "[[atecc608a-driver]]", "[[pico-2]]", "[[et-hash-input]]", "[[sensor-bridge-node]]", "[[mbedtls-verify]]", "[[flow-sensor-bridge]]"]
updated: 2026-06-19
---

# ECDSA Signing — ATECC608A secp256r1

**Algorithm:** ECDSA secp256r1 (P-256)  
**Hash:** SHA-256 (computed by ATECC608A internally via `atcab_sha`)  
**Output:** 64-byte raw signature in IEEE P1363 format: `[R: 32 bytes BE] || [S: 32 bytes BE]`

## Signing Input: [[et-hash-input]]
96-byte packed struct hashed by ATECC608A:
- 16 bytes device_id (UUID from ATECC608A UserExtra zone)
- 8 bytes timestamp_us (Pico 2 hardware timer)
- 4 bytes sequence (monotonic counter)
- 16 bytes quaternion (q_w/x/y/z float32)
- 12 bytes linear accel (ax/ay/az float32)
- 1 byte imu_cal_status + 3 pad
- 16 bytes latitude/longitude (float64×2)
- 12 bytes altitude_m/speed_ms/course_deg (float32×3)
- 1 byte fix_type + 1 byte satellites + 2 pad + 4 bytes hdop

`static_assert(sizeof(EtHashInput) == 96)`

## Signing Flow (Pico 2 super-loop, per GPS frame)
1. Populate `EtHashInput` from BNO085 + NEO-M8N state
2. `vigia_atca_sha(&et_input, 96, hash[32])` → ~40 ms
3. `vigia_atca_sign(hash, sig[64])` → ~57 ms
4. Place hash + sig into `SignedEtPacket` → COBS encode → USB-CDC

## Pi-Side Verification ([[mbedtls-verify]])
[[sensor-bridge-node]] verifies each incoming `SIGNED_ET` packet:
1. Anti-replay: `pkt->sequence > last_et_seq_`
2. `mbedtls_ecdsa_verify()` with device public key from `/etc/vigia/device_pubkey.pem`
3. Set `msg->sig_valid = true` on success; drop packet on failure

## Server-Side Attestation (Phase 6)
Python `cryptography` library: convert raw R||S to DER, verify against device cert. See `.claude/design/05_anti_death_and_depin_contracts.md §8.4`.

## Links
- Hardware: [[atecc608a]], [[pico-2]]
- Driver: [[atecc608a-driver]]
- ABI: [[et-hash-input]]
- Pi verifier: [[mbedtls-verify]], [[sensor-bridge-node]]
- Transport: [[cobs-usb-cdc]]
- Flow: [[flow-sensor-bridge]]

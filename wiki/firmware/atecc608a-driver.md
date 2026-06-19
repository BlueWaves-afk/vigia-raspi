---
title: "Atecc608aDriver"
type: firmware
tags: [firmware, security]
source: firmware/src/atecc608a_driver.c
related: ["[[pico-2]]", "[[atecc608a]]", "[[sensor-bridge-node]]", "[[et-hash-input]]", "[[ecdsa-signing]]", "[[cobs-tx-driver]]", "[[mbedtls-verify]]"]
updated: 2026-06-19
---

# Atecc608aDriver

**Files:** `firmware/src/atecc608a_driver.c` / `firmware/src/atecc608a_driver.h`  
**Status:** Written; hardware NOT YET WIRED to Pico 2 bringup board (ATECC608A pending)

cryptoauthlib wrappers for [[atecc608a]] over I2C1. Performs SHA-256 hashing and ECDSA secp256r1 signing of the [[et-hash-input]] struct.

## Interface
- **Bus:** I2C1, 400 kHz (Fast Mode)
- **Pins:** SDA=GP2, SCL=GP3
- **I2C address:** 0x60 (ADDR pin → GND)
- **Pull-ups:** 4.7 kΩ external to 3.3 V (not on Pico board)

## cryptoauthlib HAL
Uses Microchip cryptoauthlib with RP2350 I2C HAL (`atca_hal_rp2350.c` adapted from RP2040).  
`ATCA_NO_HEAP` defined — no dynamic allocation.

## API Functions
```c
ATCA_STATUS vigia_atca_init(void);      // boot: atcab_init
ATCA_STATUS vigia_atca_sha(input, len, hash_out[32]);   // ~40 ms blocking
ATCA_STATUS vigia_atca_sign(hash[32], sig_out[64]);     // ~57 ms blocking
// Total signing per GPS cycle: ~97 ms at 1 Hz GPS → 10% of 1000 ms budget
```

## Stub Mode (`VIGIA_PHASE2_STUB=1`)
Zero-fills hash_out and sig_out — allows COBS framing and Pi-side parser validation before SE is wired.

## Cryptographic Contract
- Computes `E_t = SHA-256(EtHashInput)` via `atcab_sha()` (96-byte packed struct)
- Signs E_t with device private key in ATECC608A slot 0 via `atcab_sign()` → 64-byte secp256r1 ECDSA sig (raw R||S)
- Device UUID (16 bytes) read from ATECC608A UserExtra zone

## Links
- Hardware: [[atecc608a]] (I2C1 secure element)
- MCU: [[pico-2]] (RP2350)
- ABI: [[et-hash-input]] (96-byte struct both Pico and Pi agree on)
- Signing: [[ecdsa-signing]]
- Pi verification: [[mbedtls-verify]] (SensorBridgeNode)
- Next: [[cobs-tx-driver]] (places sig into SignedEtPacket → COBS frame)

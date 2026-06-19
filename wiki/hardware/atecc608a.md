---
title: "ATECC608A Secure Element"
type: hardware
tags: [hardware, security]
source: firmware/src/atecc608a_driver.h
related: ["[[pico-2]]", "[[atecc608a-driver]]", "[[ecdsa-signing]]", "[[et-hash-input]]", "[[sensor-bridge-node]]", "[[mbedtls-verify]]", "[[flow-sensor-bridge]]"]
updated: 2026-06-19
---

# ATECC608A Secure Element

**Manufacturer:** Microchip Technology  
**Interface:** I2C1, 400 kHz (Fast Mode)  
**I2C Address:** 0x60 (ADDR pin → GND)  
**Pins on Pico 2:** SDA=GP2, SCL=GP3  
**Pull-ups:** 4.7 kΩ external to 3.3 V  
**Status:** Hardware NOT YET WIRED to Pico 2 bringup board

## Role
The sole hardware signing authority in the VIGIA system. The Pi 5 has NO direct access to this secure element. All ECDSA signatures originate here.

## Cryptographic Capabilities Used
| Operation | API | Latency |
|---|---|---|
| SHA-256 hash | `atcab_sha(96, EtHashInput, hash[32])` | ~40 ms |
| ECDSA secp256r1 sign | `atcab_sign(key_slot=0, hash[32], sig[64])` | ~57 ms |
| Device UUID read | `atcab_read_user_extra()` → 16 bytes | ~1 ms |

Total signing latency per GPS cycle: ~97 ms at 1 Hz GPS → 10% of 1000 ms budget.

## Key Slot Assignments
| Slot | Content |
|---|---|
| 0 | Device private key (secp256r1, provisioned at manufacture via Microchip Trust Platform) |
| UserExtra zone | Device UUID (16 bytes) |

## Provisioning (Phase 6)
- Microchip Trust Platform provisions private key + X.509 device certificate
- Pi stores `device_cert.pem` at `/etc/vigia/device_cert.pem`
- Server verifies certificate chain against Microchip Trust Platform root CA (anti-Sybil)

## What It Signs — [[et-hash-input]]
```
SHA-256(EtHashInput 96 bytes) → E_t_hash[32]
ECDSA_secp256r1_sign(E_t_hash) → ecdsa_sig[64] (raw R||S)
```

## Links
- Connected to: [[pico-2]] via I2C1
- Driver: [[atecc608a-driver]]
- Signing: [[ecdsa-signing]]
- ABI: [[et-hash-input]]
- Pi verification: [[mbedtls-verify]] in [[sensor-bridge-node]]
- Flow: [[flow-sensor-bridge]]

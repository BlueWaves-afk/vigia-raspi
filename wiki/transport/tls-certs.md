---
title: "TLS Certificates"
type: transport
tags: [transport, security]
source: .claude/design/05_anti_death_and_depin_contracts.md
related: ["[[anti-death-node]]", "[[aws-iot-mqtt]]", "[[https-rest]]", "[[ecdsa-signing]]"]
updated: 2026-06-19
---

# TLS Certificates — mTLS Device Identity

Files provisioned once per device at `/etc/vigia/`:

| File | Contents | Used by |
|---|---|---|
| `ca_chain.pem` | AWS IoT Core CA chain | Paho MQTT `trust_store`, validates broker |
| `device_cert.pem` | Device X.509 cert (secp256r1, from Microchip Trust Platform) | Paho MQTT `key_store` (client identity) |
| `device_key.pem` | Device private key (ECDSA) | Paho MQTT `private_key` |
| `device_ed25519.key` | 32-byte Ed25519 seed | [[anti-death-node]] libsodium signing |
| `device_id` | UUID string | Read at startup by [[anti-death-node]] |
| `device_pubkey.pem` | Device ECDSA public key | [[sensor-bridge-node]] mbedTLS verify |

## TLS Version
TLS 1.2 mandatory. `MQTT_SSL_VERSION_TLS_1_2` forced in Paho connection options. SIM7600 firmware confirmed TLS 1.2 capable.

## Certificate Chain
Device cert anchored to Microchip Trust Platform root CA. Server verifies chain for anti-Sybil validation (Phase 6).

## Links
- Used by: [[aws-iot-mqtt]], [[https-rest]]
- Signs payloads: [[ecdsa-signing]] (ATECC608A secp256r1), [[ed25519-signing]] (libsodium)

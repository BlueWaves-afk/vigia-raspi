---
title: "Flow: Sensor Bridge (signed E_t)"
type: flow
tags: [flow, sensor-bridge]
source: vigia_ws/src/vigia_edge_node/src/sensor_bridge_node.cpp
related: ["[[bno085]]", "[[neo-m8n]]", "[[bno085-driver]]", "[[neo-m8n-driver]]", "[[atecc608a]]", "[[et-hash-input]]", "[[ecdsa-signing]]", "[[cobs-usb-cdc]]", "[[sensor-bridge-node]]", "[[mbedtls-verify]]", "[[fusion-node]]"]
updated: 2026-06-19
---

# Flow: Sensor Bridge

How cryptographically-attested sensor data crosses from the Pico 2 to the Pi.

1. Pico 2 firmware reads [[bno085]] (SPI, via [[bno085-driver]]) and [[neo-m8n]]
   (UART, via [[neo-m8n-driver]]).
2. It packs an [[et-hash-input]] struct, SHA-256 hashes it, and signs with [[atecc608a]]
   ([[ecdsa-signing]]) → `SignedEt`.
3. The frame is COBS-encoded and sent over [[cobs-usb-cdc]] at 921600 baud.
4. [[sensor-bridge-node]] (Core 3, prio 85) decodes the COBS frame and runs
   [[mbedtls-verify]] against the SE public key; sets `sig_valid`.
5. Verified IMU/GPS/SignedEt is published to [[fusion-node]] for the ISS/RRI computation.

## Links
[[bno085]] / [[neo-m8n]] → [[ecdsa-signing]] → [[cobs-usb-cdc]] → [[sensor-bridge-node]] → [[mbedtls-verify]] → [[fusion-node]].

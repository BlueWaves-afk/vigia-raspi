---
title: "BLE GATT (BlueZ D-Bus)"
type: transport
tags: [transport]
source: vigia_ws/src/vigia_edge_node/src/ble_gatt_node.cpp
related: ["[[ble-gatt-node]]", "[[ble-frame-codec]]", "[[ble-gatt-constants]]", "[[raspberry-pi-5]]"]
updated: 2026-06-19
---

# BLE GATT Transport — Android Phone Link

**Stack:** BlueZ D-Bus (`sdbus-cpp` / GLib main loop)  
**Role:** Pi 5 acts as GATT peripheral; Android companion app is central  
**Compile guard:** `VIGIA_HAVE_SDBUS` — optional build flag  
**Stream rate:** 5 Hz default (configurable `stream_hz_`)

## GATT Profile
See [[ble-gatt-constants]] for UUIDs.

- **TELEMETRY_CHAR** — Notify: [[ble-frame-codec]] frames (RRI + spatial latent 256-D or 512-D, or RRI-only on poor link)
- **HANDSHAKE_CHAR** — ECDH P-256 handshake (Phase 2; Phase 1 is skeleton only)
- **CONTROL_CHAR** — Write: phone selects stream dimensions (256/512), pause/resume
- **ATTEST_CHAR** — Notify: anti-spoof attestation (Phase 2)

## Frame Wire Format
```
[0]    version=0x01
[1..4] RRI float32 clamped [0,1]
[5]    dims code (0x00=256D, 0x01=512D, 0xFF=RRI-only)
[6..]  float32[dims] spatial latent
```
Frame sizes: 1030 B (256-D), 2054 B (512-D), 6 B (RRI-only).

## D-Bus Thread Model
GLib main loop on dedicated `std::thread` (SCHED_OTHER). ROS2 callbacks update `mailbox_mutex_`-guarded mailbox. GLib timer drains mailbox at `stream_hz_`.

## Graceful Degradation
When BLE link is poor (MTU or retry limits), [[ble-frame-codec]] encodes `kRriOnly` (0xFF dims code) — drops vector, keeps streaming RRI to phone.

## Links
- Used by: [[ble-gatt-node]]
- Codec: [[ble-frame-codec]]
- Constants: [[ble-gatt-constants]]
- Radio: [[raspberry-pi-5]] (onboard BT)

---
title: "SensorBridgeNode"
type: ros2-node
tags: [ros2-node, sensor, security]
source: vigia_ws/src/vigia_edge_node/src/sensor_bridge_node.cpp
related: ["[[fusion-node]]", "[[anti-death-node]]", "[[pico-2]]", "[[bno085]]", "[[neo-m8n]]", "[[atecc608a]]", "[[cobs-usb-cdc]]", "[[mbedtls-verify]]", "[[et-hash-input]]", "[[vigia-qos]]", "[[rt-thread]]", "[[params-yaml]]", "[[flow-sensor-bridge]]"]
updated: 2026-06-19
---

# SensorBridgeNode

**File:** `vigia_ws/src/vigia_edge_node/src/sensor_bridge_node.cpp` / `sensor_bridge_node.hpp`

Reads COBS-framed binary packets from the [[pico-2]] over `/dev/ttyACM0`. Decodes, validates, verifies ECDSA signature on `SIGNED_ET` packets. Publishes IMU, GPS, and signed kinematic context. Also supports legacy text protocol for bringup.

## Thread Configuration

| Property | Value |
|---|---|
| SCHED_FIFO Priority | **85** |
| CPU Core Affinity | **Core 3** |
| pthread Name | `vigia_bridge` |

## Published Topics

| Topic | Message Type | Rate |
|---|---|---|
| `/vigia/imu` | `vigia_msgs/msg/ImuSample` | 100 Hz |
| `/vigia/gps` | `vigia_msgs/msg/GpsPvt` | 1-10 Hz |
| `/vigia/signed_et` | `vigia_msgs/msg/SignedEt` | 1-10 Hz |
| `/vigia/sensor_health` | `vigia_msgs/msg/SensorHealth` | 1 Hz diagnostic |

## Dual-Protocol Auto-Detection
First byte on `/dev/ttyACM0`:
- `'V'` (0x56) → text mode (`VIGIA_IMU`/`VIGIA_GPS`/`VIGIA_PING` lines) — bringup
- Other byte → COBS binary mode (Phase 2 `SignedEtPacket` 173 bytes)

## COBS Decoder (Pi-side, `sensor_bridge_node.cpp`)
State-machine decoder, no dynamic allocation. Packet buffer pre-allocated at 512 bytes max.

## ECDSA Verification (Phase 2, [[mbedtls-verify]])
On `SIGNED_ET` packets:
1. Anti-replay: `pkt->sequence > last_et_seq_`
2. `mbedtls_ecdsa_verify()` with `MBEDTLS_PK_ECDSA` + `MBEDTLS_MD_SHA256`
3. On failure: increment `sig_fail_counter_`, `RCLCPP_WARN`, drop packet
4. `msg->sig_valid = true` only after verified

## IMU History Ring Buffer
- 200-sample circular array of `ImuSnapshot` structs
- `imu_at_or_before(timestamp_us)` — reverse scan for GPS-timestamp-aligned IMU lookup
- Exposed as ROS2 service `/vigia/imu_at_timestamp`

## Read Loop Efficiency
- `select()` + 256-byte chunk reads — 256× fewer syscalls vs byte-by-byte
- 1 ms `TimerBase` callback poll at 921600 baud (GPS packet arrives in ~0.37 ms)

## Links
- Reads from: [[pico-2]] via [[cobs-usb-cdc]] (`/dev/ttyACM0`)
- Data sources: [[bno085]] (via Pico), [[neo-m8n]] (via Pico), [[atecc608a]] (signing)
- Publishes to: [[fusion-node]] (IMU/GPS/SignedEt), [[anti-death-node]] (SignedEt)
- Verifies: [[ecdsa-signing]] via [[mbedtls-verify]]
- Struct ABI: [[et-hash-input]]
- Config: [[params-yaml]] `sensor_bridge_node` section
- Flow: [[flow-sensor-bridge]]

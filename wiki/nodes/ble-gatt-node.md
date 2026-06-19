---
title: "BleGattNode"
type: ros2-node
tags: [ros2-node, transport]
source: vigia_ws/src/vigia_edge_node/src/ble_gatt_node.cpp
related: ["[[vision-node]]", "[[fusion-node]]", "[[ble-gatt]]", "[[ble-frame-codec]]", "[[ble-gatt-constants]]", "[[vigia-rri]]", "[[vigia-qos]]", "[[raspberry-pi-5]]"]
updated: 2026-06-19
---

# BleGattNode

**File:** `vigia_ws/src/vigia_edge_node/src/ble_gatt_node.cpp` / `ble_gatt_node.hpp`

BlueZ D-Bus GATT peripheral for the Android companion app phone link. Streams RRI score + spatial latent vector S_t to a bonded central (phone) at configurable rate (default 5 Hz). Guarded by `VIGIA_HAVE_SDBUS` compile flag.

## Thread Model
- ROS2 callbacks run on ROS2 executor thread (updates atomic mailbox)
- D-Bus GLib main loop runs on a **dedicated `std::thread`** (SCHED_OTHER) — `dbus_thread_main()`
- Decoupled via `mailbox_mutex_` — contention is bounded (notify timer fires at stream_hz)

## Subscribed Topics

| Topic | Callback |
|---|---|
| `/vigia/spatial_latent` | Cache latest latent vector into `latest_latent_` |
| `/vigia/detections` | Cache latest RRI into `latest_rri_` (best confidence) |

## GATT Profile (via [[ble-gatt-constants]])
- Service UUID: `5e355f98-eabf-4ae0-8417-919e926d411e`
- `TELEMETRY_CHAR` (`4d23...`): Notify — streams [[ble-frame-codec]] encoded frames
- `HANDSHAKE_CHAR` (`eb4b...`): Read+Write+Notify — ECDH P-256 handshake (Phase 2)
- `CONTROL_CHAR` (`0bb8...`): Write — phone requests 256-D / 512-D stream or pause/resume
- `ATTEST_CHAR` (`580c...`): Notify — anti-spoof attestation (Phase 2)

## Wire Format (via [[ble-frame-codec]])
```
[0]     uint8   version = 0x01
[1..4]  float32 RRI score clamped [0,1]
[5]     uint8   dims code (0x00=256D, 0x01=512D, 0xFF=RRI-only)
[6..]   float32[dims] spatial latent vector
```
Graceful degradation: `kRriOnly` (0xFF) streams when BLE link is poor — RRI without vector.

## Parameters
| Parameter | Default |
|---|---|
| `ble_adapter_` | `hci0` |
| `device_id_` | `vigia-001` |
| `stream_hz_` | `5.0` |
| `default_dims_` | `256` |

## Links
- Subscribes to: [[vision-node]] (spatial latent), [[fusion-node]] (detections/RRI)
- Uses transport: [[ble-gatt]] (BlueZ D-Bus)
- Codec: [[ble-frame-codec]]
- Constants: [[ble-gatt-constants]]
- RRI helper: [[vigia-rri]]
- Physical radio: [[raspberry-pi-5]] (onboard BT/BLE)

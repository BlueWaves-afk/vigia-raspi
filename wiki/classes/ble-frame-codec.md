---
title: "BleFrameCodec"
type: cpp-class
tags: [cpp-class, transport]
source: vigia_ws/src/vigia_edge_node/include/vigia_edge_node/ble_frame_codec.hpp
related: ["[[ble-gatt-node]]", "[[ble-gatt-constants]]", "[[ble-gatt]]"]
updated: 2026-06-19
---

# BleFrameCodec

**File:** `vigia_ws/src/vigia_edge_node/include/vigia_edge_node/ble_frame_codec.hpp`

Header-only encoder/decoder for BLE telemetry wire format. Must stay byte-identical with Android app's `BleDataStreamerImpl.kt`.

## Wire Format (little-endian)
```
[0]     uint8   version = 0x01
[1..4]  float32 RRI score, clamped [0,1]
[5]     uint8   dims code
[6..]   float32[dims] spatial latent vector
```

## DimsCode enum
| Code | Dims | Frame size |
|---|---|---|
| `0x00` | 256 | 6 + 256×4 = 1030 bytes |
| `0x01` | 512 | 6 + 512×4 = 2054 bytes |
| `0xFF` | 0 (RRI-only) | 6 bytes |

## Key Functions
- `encode_frame(rri, code, latent, latent_len, out)` — writes into `out` vector, no extra allocation
- `decode_frame(data, len)` → `DecodedFrame{valid, rri, dims_code, latent}` — mirrors Kotlin decoder; silently rejects malformed frames

## Graceful Degradation
`kRriOnly` (0xFF) sentinel — streams RRI without vector when BLE link is poor. Android decoder must accept 0xFF.

## Links
- Used by: [[ble-gatt-node]]
- Constants: [[ble-gatt-constants]]
- Transport: [[ble-gatt]]

---
title: "FrameMetadataRing"
type: cpp-class
tags: [cpp-class, realtime, memory]
source: vigia_ws/src/vigia_edge_node/include/vigia_edge_node/shm_ring_buffer.hpp
related: ["[[fusion-node]]", "[[camera-node]]", "[[anti-death-node]]", "[[shm-ring-buffer]]", "[[adr-seqlock-ring]]"]
updated: 2026-06-19
---

# FrameMetadataRing

**File:** `vigia_ws/src/vigia_edge_node/include/vigia_edge_node/shm_ring_buffer.hpp` (addition)

Parallel compact ring buffer alongside [[shm-ring-buffer]]. Stores per-frame sensor context (124 bytes/frame × 300 frames = 37.2 KB). This is what gets serialized into the MQTT/HTTPS payload during the [[anti-death-node]] emergency sequence — NOT the 829 MB raw pixel buffer.

## `FrameMetadata` struct (124 bytes, `__attribute__((packed))`)
| Field | Size | Notes |
|---|---|---|
| `timestamp_us` | uint64 | CameraNode capture time |
| `frame_id` | uint32 | Monotonic counter |
| `rri_score` | float32 | -1.0 if not yet fused |
| `iss_score` | float32 | -1.0 if not yet computed |
| `q_w/x/y/z` | float32 × 4 | BNO085 quaternion |
| `lin_accel_x/y/z` | float32 × 3 | Body-frame accel |
| `imu_cal_status` | uint8 | + 3 pad bytes |
| `latitude/longitude` | float64 × 2 | NEO-M8N WGS-84 |
| `altitude_m/speed_ms/course_deg` | float32 × 3 | GPS fields |
| `fix_type/satellites/hdop` | uint8+uint8+float32 | + 2 pad bytes |
| `detection_count` | uint8 | YOLO detections this frame |
| `best_yolo_conf` | float32 | -1.0 if no detections |
| `depth_hash_trunc` | uint8[8] | Truncated SHA-256 of depth map |

## `FrameMetadataRing` struct
- `alignas(64) std::atomic<uint32_t> seq{0}` — seqlock counter
- `uint32_t write_idx` — next write position
- `FrameMetadata frames[300]`
- Allocated at `/dev/shm/vigia_meta_ring.buf` (mmap'd by CameraNode + FusionNode + AntiDeathNode)

## Writers
- **FusionNode**: calls `write_metadata()` after each RRI computation (fields: rri_score, iss_score, detection_count, best_yolo_conf, depth_hash_trunc)
- **CameraNode**: writes timestamp_us, frame_id, imu_cal_status on frame capture

## Snapshot Reader
- **AntiDeathNode**: `snapshot()` — seqlock protocol identical to [[shm-ring-buffer]]

## Links
- Writers: [[fusion-node]], [[camera-node]]
- Reader: [[anti-death-node]]
- Sibling: [[shm-ring-buffer]] (pixel ring)
- ADR: [[adr-seqlock-ring]]
- Flow: [[flow-anti-death]]

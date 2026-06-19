---
title: "vigia_qos — QoS Profiles"
type: cpp-class
tags: [cpp-class, transport]
source: vigia_ws/src/vigia_edge_node/include/vigia_edge_node/vigia_qos.hpp
related: ["[[camera-node]]", "[[vision-node]]", "[[depth-node]]", "[[fusion-node]]", "[[sensor-bridge-node]]", "[[anti-death-node]]"]
updated: 2026-06-19
---

# vigia_qos — QoS Profile Definitions

**File:** `vigia_ws/src/vigia_edge_node/include/vigia_edge_node/vigia_qos.hpp`

Namespace `vigia::qos`. Five named QoS profiles used across all nodes.

## Profiles

| Profile | KeepLast | Reliability | Durability | Used for |
|---|---|---|---|---|
| `sensor_stream()` | 1 | best_effort | volatile | IMU (100 Hz), GPS — only latest matters |
| `camera_frames()` | 4 | best_effort | volatile | Raw frames — burst smoothing |
| `inference_results()` | 10 | reliable | volatile | YOLO detections, depth maps, spatial latent |
| `hazard_events()` | 10 | reliable | transient_local | HazardEvent — late joiners receive last event |
| `signed_et()` | 5 | reliable | volatile | SignedEt — 10 Hz, must not drop |

## Links
- Used by: [[camera-node]], [[vision-node]], [[depth-node]], [[fusion-node]], [[sensor-bridge-node]], [[anti-death-node]]

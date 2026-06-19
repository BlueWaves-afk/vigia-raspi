---
title: "params.yaml (unified ROS2 params)"
type: config
tags: [config]
source: vigia_ws/src/vigia_edge_node/config/params.yaml
related: ["[[camera-node]]", "[[vision-node]]", "[[depth-node]]", "[[fusion-node]]", "[[sensor-bridge-node]]", "[[anti-death-node]]", "[[ble-gatt-node]]", "[[vigia-qos]]"]
updated: 2026-06-19
---

# params.yaml

Single unified parameter file declaring runtime config for all edge-node ROS2 nodes,
loaded at launch. One source of truth for tuning without recompiling.

- **Per-node sections:** camera (resolution, fps, V4L2 device), vision (model path, spatial
  latent layer `/model.22/cv2/act/Mul_output_0`, conf thresholds), depth (MiDaS input size),
  fusion (`v_min_ms`, Kalman gains, gravity vector), sensor_bridge (baud 921600,
  `verify_ecdsa`), ble_gatt (UUIDs, stream rate), anti_death (GPIO chip/line, window seconds).
- Consumed by every node listed in [[index]]; QoS profiles live in [[vigia-qos]].

## Links
Configures: [[camera-node]], [[vision-node]], [[depth-node]], [[fusion-node]], [[sensor-bridge-node]], [[anti-death-node]], [[ble-gatt-node]].

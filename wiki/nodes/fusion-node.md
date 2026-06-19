---
title: "FusionNode"
type: ros2-node
tags: [ros2-node, fusion, realtime]
source: vigia_ws/src/vigia_edge_node/src/fusion_node.cpp
related: ["[[vision-node]]", "[[depth-node]]", "[[sensor-bridge-node]]", "[[anti-death-node]]", "[[vigia-rri]]", "[[frame-metadata-ring]]", "[[vigia-qos]]", "[[rt-thread]]", "[[params-yaml]]", "[[flow-capture-to-uplink]]", "[[adr-gravity-compensated-iss]]"]
updated: 2026-06-19
---

# FusionNode

**File:** `vigia_ws/src/vigia_edge_node/src/fusion_node.cpp` / `fusion_node.hpp`

Receives all inference outputs and sensor data. Performs gravity compensation (Eigen quaternion), ISS computation, Kalman filter velocity fusion, and weighted RRI scoring via [[vigia-rri]]. Publishes `HazardEvent` when `RRI >= rri_threshold`. Also writes per-frame metadata to [[frame-metadata-ring]].

## Thread Configuration

| Property | Value |
|---|---|
| SCHED_FIFO Priority | **70** |
| CPU Core Affinity | **Core 3** |
| pthread Name | `vigia_fusion` |

## Published Topics

| Topic | Message Type | QoS |
|---|---|---|
| `/vigia/hazard_event` | `vigia_msgs/msg/HazardEvent` | `hazard_events` (KeepLast 10, reliable, transient_local) |

## Subscribed Topics

| Topic | Source | Rate |
|---|---|---|
| `/vigia/detections` | [[vision-node]] | ~28 Hz |
| `/vigia/depth` | [[depth-node]] | ~9 Hz (stride 3) |
| `/vigia/imu` | [[sensor-bridge-node]] | 100 Hz |
| `/vigia/gps` | [[sensor-bridge-node]] | 1-10 Hz |
| `/vigia/signed_et` | [[sensor-bridge-node]] | 1-10 Hz |

## Message Synchronization
Independent callbacks with cached latest state — no `message_filters`. Fusion triggered when `on_detections()` fires; uses all currently cached state. Stale depth/IMU is used as-is (timestamps checked for age).

## Gravity Compensation Pipeline (Improvement 2)
```
Step 1: q = Eigen::Quaternionf(q_w, q_x, q_y, q_z).normalized()
Step 2: a_world = q * a_body   (Eigen Quaternion::operator*(Vector3))
Step 3: a_detrended = a_world - Vector3f(0, 0, 9.81)
Step 4: iss = |a_detrended.z()| / max(v_gps, v_min_ms)
```
Source: `.claude/design/01_system_architecture_and_roadmap.md §5 Improvement 2`

## Kalman Filter (2D velocity dead-reckoning)
- State: `[v_x, v_y]` (m/s, world frame), Eigen 2×2 matrices
- Predict: IMU a_detrended.x/y integrated at 100 Hz
- Update: GPS speed when `valid_fix && hdop <= gps_fallback_hdop_max`

## RRI Formula
Uses [[vigia-rri]] `compute_rri()` with graceful degradation:
```
RRI = renormalized_weighted(0.35×yolo_conf + 0.25×geo_conf + 0.15×temp_conf + 0.25×iss_norm)
```
Ports geometry confidence `dep*exp(-roughness*10)` from `src/fusion.cpp:25` and `TemporalAnalyzer` circular buffer from `src/temporal.cpp`.

## Frame Metadata Write
Calls `frame_metadata_ring_->write_metadata(FrameMetadata{...})` after each RRI computation — feeds [[anti-death-node]] emergency snapshot.

## Links
- Subscribes to: [[vision-node]], [[depth-node]], [[sensor-bridge-node]]
- Publishes to: [[anti-death-node]] (HazardEvent)
- Uses: [[vigia-rri]], [[frame-metadata-ring]]
- Config: [[params-yaml]] `fusion_node` section
- ADR: [[adr-gravity-compensated-iss]]
- Flow: [[flow-capture-to-uplink]]

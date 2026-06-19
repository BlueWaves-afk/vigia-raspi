---
title: "VisionNode"
type: ros2-node
tags: [ros2-node, inference, vision]
source: vigia_ws/src/vigia_edge_node/src/vision_node.cpp
related: ["[[camera-node]]", "[[fusion-node]]", "[[anti-death-node]]", "[[onnx-runtime]]", "[[yolo-int8]]", "[[kleidiai-acl]]", "[[io-binding]]", "[[vigia-qos]]", "[[rt-thread]]", "[[params-yaml]]", "[[raspberry-pi-5]]", "[[flow-capture-to-uplink]]"]
updated: 2026-06-19
---

# VisionNode

**File:** `vigia_ws/src/vigia_edge_node/src/vision_node.cpp` / `vision_node.hpp`

Runs YOLOv26 INT8 inference via ONNX Runtime + KleidiAI ACL EP on every received camera frame. Extracts the penultimate feature map as spatial latent vector S_t. Publishes detections and spatial latent.

## Thread Configuration

| Property | Value |
|---|---|
| SCHED_FIFO Priority | **75** |
| CPU Core Affinity | **Core 1** |
| pthread Name | `vigia_vision` |
| Executor | `StaticSingleThreadedExecutor` |

## Published Topics

| Topic | Message Type | QoS | Transfer |
|---|---|---|---|
| `/vigia/detections` | `vigia_msgs/msg/DetectionArray` | `inference_results` (KeepLast 10, reliable) | `unique_ptr` → single consumer (FusionNode) |
| `/vigia/spatial_latent` | `vigia_msgs/msg/SpatialLatent` | `inference_results` | `unique_ptr` → single consumer (AntiDeathNode) |

## Subscribed Topics

| Topic | Message Type |
|---|---|
| `/vigia/camera/image_raw` | `sensor_msgs/msg/Image` (shared_ptr — fan-in shared with DepthNode) |

## ONNX Session Configuration
- `ORT_ENABLE_ALL` graph optimization
- `SetIntraOpNumThreads(1)` — Core 1 isolation mandatory
- `DisableMemPattern()` + `DisableCpuMemArena()` — flat heap in RT context
- `session.set_denormal_as_zero = 1` — prevents 100× slowdown on FP32 denormals
- ACL Execution Provider: `OrtSessionOptionsAppendExecutionProvider_ACL(opts, 1)` — KleidiAI UDOT; currently commented out pending ACL build

## Pre-Allocated Buffers (static, BSS segment)
| Buffer | Size | Purpose |
|---|---|---|
| `letterbox_buf_` | 307,200 B (320×320×3) | BGR8 letterbox resize target |
| `chw_input_buf_` | 307,200 B (3×320×320) | NCHW RGB INT8 ONNX input |
| `det_output_buf_` | ~705 KB (84×2100 float32) | YOLO detection output |
| `latent_output_buf_` | 512 KB max (128K float32) | S_t spatial latent buffer |

## NEON Preprocessing
- `preprocess_letterbox()` — letterbox resize into `letterbox_buf_` (no allocation)
- `neon_bgr_hwc_to_rgb_chw()` — `vld3q_u8` single-pass BGR→RGB + HWC→CHW; ~8 µs for 320×320 on Cortex-A76

## S_t Extraction
- Penultimate layer: `/model.22/cv2/act/Mul_output_0` (from `params.yaml`), shape `[1,256,20,20]`, global-avg-pooled to 256-D
- Bound as second output in [[io-binding]] constructor; populated by `session_->Run()`
- Published as `SpatialLatent::latent_vector` (float32 array)

## Links
- Subscribes to: [[camera-node]]
- Publishes to: [[fusion-node]] (detections), [[anti-death-node]] (spatial latent)
- Uses: [[onnx-runtime]], [[yolo-int8]], [[kleidiai-acl]], [[io-binding]]
- Physical: [[raspberry-pi-5]] Core 1
- Config: [[params-yaml]] `vision_node` section
- Flow: [[flow-capture-to-uplink]]

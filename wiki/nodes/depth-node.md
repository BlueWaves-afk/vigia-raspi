---
title: "DepthNode"
type: ros2-node
tags: [ros2-node, inference, depth]
source: vigia_ws/src/vigia_edge_node/src/depth_node.cpp
related: ["[[camera-node]]", "[[fusion-node]]", "[[onnx-runtime]]", "[[midas-fp32]]", "[[io-binding]]", "[[vigia-qos]]", "[[rt-thread]]", "[[params-yaml]]", "[[raspberry-pi-5]]", "[[flow-capture-to-uplink]]"]
updated: 2026-06-19
---

# DepthNode

**File:** `vigia_ws/src/vigia_edge_node/src/depth_node.cpp` / `depth_node.hpp`

Runs MiDaS v2.1 small FP32 inference via ONNX Runtime on a stride-gated subset of camera frames. Publishes normalized inverse depth maps for use in geometry confidence scoring by [[fusion-node]].

## Thread Configuration

| Property | Value |
|---|---|
| SCHED_FIFO Priority | **75** |
| CPU Core Affinity | **Core 2** |
| pthread Name | `vigia_depth` |

## Published Topics

| Topic | Message Type |
|---|---|
| `/vigia/depth` | `vigia_msgs/msg/DepthMap` (256×256 float32, inference_results QoS) |

## Subscribed Topics
- `/vigia/camera/image_raw` — `shared_ptr<const sensor_msgs::msg::Image>` (shared with VisionNode)

## ONNX Session Configuration
- **FP32 ONLY** — INT8 MiDaS is prohibited (accuracy degradation approved decision)
- CPU EP only — NO ACL EP (adding ACL to FP32 model inserts NHWC transposes that increase latency)
- `SetIntraOpNumThreads(2)` — Core 2 dedicated; 2 threads fills pipeline stalls during memory-bound ops
- `session.disable_quant_qdq_cleanup = 1` — prevents accidental INT8 transformation

## Pre-Allocated Buffers
| Buffer | Size | Purpose |
|---|---|---|
| `midas_resize_buf_` | 196,608 B (256×256×3) | BGR8 resize target |
| `midas_float_hwc_buf_` | 786,432 B | Float HWC intermediate |
| `midas_chw_buf_` | 786,432 B | NCHW float32 ONNX input |
| `depth_output_buf_` | 262,144 B (256×256 float32) | MiDaS inverse depth output |

## NEON Preprocessing
- `neon_bgr_u8_to_normrgb_chw()` — `vld3_u8` (8 pixels/iter) + ImageNet normalization fused into `(x*scale + bias)` via `vmlaq_f32`; RGB CHW output; ~65K pixels in 2 cycles/pixel at 150 MHz

## Stride Gating
- `frame_counter_ % midas_stride_ == 0` — only infers on every Nth frame
- Default stride 3 → ~10 FPS at 30 FPS camera
- Thermal monitor reads `/sys/class/thermal/thermal_zone0/temp` at most 1 Hz; increases stride at 75°C → 4, at 85°C → 6
- On skipped frames, FusionNode uses cached previous depth result

## Zero-Copy Output
- Depth output tensor wrapped via `Ort::Value::GetTensorMutableData<float>()` — no clone of depth output

## Links
- Subscribes to: [[camera-node]]
- Publishes to: [[fusion-node]] (DepthMap)
- Uses: [[onnx-runtime]], [[midas-fp32]], [[io-binding]]
- Physical: [[raspberry-pi-5]] Core 2
- Config: [[params-yaml]] `depth_node` section
- Flow: [[flow-capture-to-uplink]]

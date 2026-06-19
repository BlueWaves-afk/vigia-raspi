---
title: "MiDaS v2.1 Small FP32"
type: inference
tags: [inference, depth]
source: models/midasv21/midas_v21_small_256.onnx
related: ["[[depth-node]]", "[[onnx-runtime]]", "[[io-binding]]", "[[fusion-node]]"]
updated: 2026-06-19
---

# MiDaS v2.1 Small FP32

**Model file:** `models/midasv21/midas_v21_small_256.onnx`  
**Status:** Model file NOT yet present on Pi — requires download

## Specifications
| Property | Value |
|---|---|
| Architecture | MiDaS v2.1 small (MobileNetV2 encoder) |
| Input | `input` — NCHW, [1, 3, 256, 256], float32 normalized RGB |
| Output | `output` — [1, 256, 256] float32 inverse depth (normalized, higher=closer) |
| Precision | **FP32 ONLY** — INT8 quantization is prohibited |
| Current latency | ~525 ms/frame (stride-5 adaptive) |
| Target latency | ≤200 ms/frame |

## Normalization Constants (ImageNet)
```
Mean: [0.485, 0.456, 0.406]
Std:  [0.229, 0.224, 0.225]
Fused: scale = 1/(255*std), bias = -mean/std
```
Applied via NEON `vmlaq_f32` in `neon_bgr_u8_to_normrgb_chw()`.

## Why FP32 Only?
- MiDaS depth estimation is sensitive to quantization
- Phase 3 acceptance criterion: ≤5% mean relative error vs FP32 baseline
- If INT8 exceeds this threshold: INT8 is permanently prohibited
- ACL EP explicitly excluded — adding ACL to FP32 model inserts costly NHWC transposes

## Output Use
- `DepthMap::data` — 256×256 float32 array published to `/vigia/depth`
- [[fusion-node]] uses depth for geometry confidence: `dep * exp(-roughness * 10)`
- `depth_hash_trunc` — truncated SHA-256 written to [[frame-metadata-ring]]

## Links
- Runs in: [[depth-node]], [[onnx-runtime]]
- Zero-copy: [[io-binding]]
- Consumer: [[fusion-node]] (geometry confidence)

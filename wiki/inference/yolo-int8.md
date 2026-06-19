---
title: "YOLOv26 Nano INT8"
type: inference
tags: [inference, vision]
source: models/yolo26/vigia_v26n_int8_convonly_with_latent.onnx
related: ["[[vision-node]]", "[[onnx-runtime]]", "[[kleidiai-acl]]", "[[io-binding]]", "[[adr-onnx-vs-openvino]]"]
updated: 2026-06-19
---

# YOLOv26 Nano INT8

**Model file:** `models/yolo26/vigia_v26n_int8_convonly_with_latent.onnx`  
**Status:** Model file NOT yet present on Pi — requires export from Ultralytics YOLOv26 repo + quantization pipeline

## Specifications
| Property | Value |
|---|---|
| Architecture | YOLOv26 Nano |
| Input | `images` — NCHW, [1, 3, 320, 320], QUInt8 |
| Detection output | `output0` — [1, 84, 2100] float32 (4 bbox + 80 class logits × 2100 anchors) |
| Latent output | `/model.22/cv2/act/Mul_output_0` — [1, 256, 20, 20], global-avg-pooled to 256-D S_t |
| Precision | QUInt8 (unsigned INT8 — KleidiAI UDOT preference) |
| Quantization | QDQ format via ONNX Runtime static quantization |
| Model size | ~6 MB (post-quantization) |

## Model Preparation Pipeline (`tools/model_prep/`)
1. `01_export_yolov26.py` — Ultralytics YOLO export, ONNX opset 17, 320px, simplify=True
2. `02_inspect_latent_layer.py` — identify penultimate layer name via Netron
3. `03_add_latent_output.py` — add penultimate layer as second ONNX graph output
4. `04_quantize_int8.py` — `quantize_static()` with `RoadCalibrationReader` (200-500 road images), QUInt8, QDQ format, per-tensor, reduce_range=True

## Why QUInt8 (not QInt8)?
ARM KleidiAI UDOT micro-kernels (`arm_gemm::GemmHybridQuantized`) are optimized for unsigned operands. UDOT throughput > SDOT on A76 for convolution workloads. ACL EP maps QUInt8 convolutions to `CpuAcc::NEGEMMConvolution2d` with UDOT dispatch.

## Performance Target
- Current (CPU EP, no KleidiAI): ~28 ms/frame (~35 FPS)
- Target (with KleidiAI UDOT): ~7 ms/frame (~4× speedup on INT8 GEMM)

## Postprocessing (NMS)
Full NMS + multi-layout parser + letterbox inverse scaling ported from `src/perception.cpp` into `vision_node.cpp`.

## Links
- Runs in: [[vision-node]], [[onnx-runtime]]
- EP: [[kleidiai-acl]]
- Zero-copy: [[io-binding]]
- ADR: [[adr-onnx-vs-openvino]]

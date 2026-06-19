---
title: "KleidiAI / ARM Compute Library EP"
type: inference
tags: [inference, hardware]
source: .claude/design/04_onnx_vision_engine_contracts.md
related: ["[[vision-node]]", "[[onnx-runtime]]", "[[yolo-int8]]", "[[raspberry-pi-5]]", "[[adr-onnx-vs-openvino]]"]
updated: 2026-06-19
---

# KleidiAI / ARM Compute Library Execution Provider

**Status: NOT YET BUILT** — ACL and ORT with KleidiAI must be compiled from source on Pi 5

KleidiAI is ARM's INT8 UDOT micro-kernel library bundled in ARM Compute Library (ACL) ≥ 24.01. Provides ~4× INT8 GEMM throughput on Cortex-A76 via `asimddp` (UDOT) instructions.

## Hardware Prerequisite
Pi 5 BCM2712 has Cortex-A76 cores with `asimddp` confirmed in `/proc/cpuinfo`. UDOT (unsigned dot product) dispatches ARM's `arm_gemm::GemmHybridQuantized` for quantized Conv2D and DepthwiseConv2D.

## Build Plan (from doc 01 §KleidiAI Build Plan)
```bash
# Step 1 — ARM Compute Library (~20 min)
git clone --depth 1 https://github.com/ARM-software/ComputeLibrary.git /opt/acl
cd /opt/acl && scons Werror=0 debug=0 neon=1 opencl=0 os=linux arch=arm64-v8.2-a -j4

# Step 2 — ONNX Runtime with KleidiAI (~45 min)
./build.sh --config Release --use_acl --acl_home /opt/acl --parallel \
  --cmake_extra_defines DONNXRUNTIME_USE_KLEIDIAI=ON

# Step 3 — install to /opt/onnxruntime-kleidi/
```

## Performance Impact
| Path | Latency |
|---|---|
| CPU EP only (current) | ~28 ms/frame |
| KleidiAI UDOT (target) | ~7 ms/frame |
| Speedup | ~4× on INT8 GEMM layers |

## ORT Session Configuration
```cpp
Ort::ThrowOnError(
    OrtSessionOptionsAppendExecutionProvider_ACL(opts, /*enable_fast_math=*/1));
// enable_fast_math=1 → routes QUInt8 Conv2D through UDOT assembly dispatch
```
Call-site currently **commented out** in `vision_node.cpp` pending ACL build.

## Verification
After build: `grep asimddp /proc/cpuinfo` + `onnxruntime.get_available_providers()` should include `AclExecutionProvider`.

## Links
- Used by: [[vision-node]]
- Model: [[yolo-int8]] (QUInt8 QDQ format required)
- Physical: [[raspberry-pi-5]] (Cortex-A76, asimddp)
- ADR: [[adr-onnx-vs-openvino]]

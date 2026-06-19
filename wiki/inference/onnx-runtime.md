---
title: "ONNX Runtime"
type: inference
tags: [inference, vision]
source: vigia_ws/src/vigia_edge_node/src/vision_node.cpp
related: ["[[vision-node]]", "[[depth-node]]", "[[yolo-int8]]", "[[midas-fp32]]", "[[kleidiai-acl]]", "[[io-binding]]", "[[adr-onnx-vs-openvino]]"]
updated: 2026-06-19
---

# ONNX Runtime

**Version:** 1.20.1 C++ headers at `/opt/onnxruntime/`  
**Install status:** Stock CPU EP only — KleidiAI/ACL EP NOT YET BUILT

ONNX Runtime C++ library powering both inference nodes. Replaces OpenVINO 2025 from Phase 0.

## Shared `Ort::Env` Singleton
Created once in `main.cpp`, passed by const-ref to both nodes:
```cpp
static Ort::Env g_ort_env{ORT_LOGGING_LEVEL_WARNING, "vigia_onnx"};
```
`Ort::Env` is thread-safe for concurrent `session.Run()` across multiple sessions.

## Session per Node
| Node | Model | Precision | EP |
|---|---|---|---|
| [[vision-node]] | [[yolo-int8]] | QUInt8 INT8 | ACL (KleidiAI) → CPU fallback |
| [[depth-node]] | [[midas-fp32]] | FP32 | CPU EP only |

## Key Session Options (both nodes)
- `ORT_ENABLE_ALL` graph optimization (enables QDQ cleanup for INT8 fusion on ACL)
- `DisableMemPattern()` + `DisableCpuMemArena()` — flat heap profile in RT context
- `session.set_denormal_as_zero = 1` — prevents 100× FP32 denormal slowdown on Cortex-A76
- `SetIntraOpNumThreads(1)` on VisionNode (Core 1 isolation), `SetIntraOpNumThreads(2)` on DepthNode

## Zero-Copy Pattern
All buffers pre-allocated in constructors as static arrays. Bound via [[io-binding]] `Ort::IoBinding`. `session_->Run(run_options_, *io_binding_)` — zero tensor construction overhead per frame.

## Links
- Used by: [[vision-node]], [[depth-node]]
- Models: [[yolo-int8]], [[midas-fp32]]
- EP: [[kleidiai-acl]] (planned)
- Zero-copy: [[io-binding]]
- ADR: [[adr-onnx-vs-openvino]]

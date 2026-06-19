---
title: "Ort::IoBinding — Zero-Copy Inference"
type: inference
tags: [inference, memory]
source: .claude/design/04_onnx_vision_engine_contracts.md
related: ["[[vision-node]]", "[[depth-node]]", "[[onnx-runtime]]", "[[yolo-int8]]", "[[midas-fp32]]"]
updated: 2026-06-19
---

# Ort::IoBinding — Pre-Bound Zero-Copy Inference

**File:** `vigia_ws/src/vigia_edge_node/include/vigia_edge_node/ort_utils.hpp` (helper)

`Ort::IoBinding` eliminates per-call tensor construction in the inference hot path. Input and output tensors are bound once in the node constructor; `session_->Run(run_options_, *io_binding_)` performs zero tensor construction overhead.

## Zero-Copy Memory Pattern
```cpp
// All pre-allocated buffers use OrtDeviceAllocator — caller owns pointer, ORT never frees
inline Ort::MemoryInfo make_cpu_memory_info() {
    return Ort::MemoryInfo("Cpu", OrtDeviceAllocator, 0, OrtMemTypeDefault);
}

template <typename T>
Ort::Value wrap_tensor(T* data, size_t elem_count, const std::array<int64_t, 4>& shape) {
    static auto mem_info = make_cpu_memory_info();
    return Ort::Value::CreateTensor<T>(mem_info, data, elem_count, shape.data(), shape.size());
}
```

## Constructor Flow (one-time)
```
static buffers → Ort::Value (zero-copy wrap) → io_binding_->BindInput/BindOutput
```

## Inference Hot Path (every frame)
```
fill pre_alloc input buffer (NEON preprocessing writes here)
session_->Run(run_options_, *io_binding_)   ← zero tensor construction overhead
read pre_alloc output buffers (results already in bound memory)
```

## VisionNode Bindings
| Binding | Buffer | Shape |
|---|---|---|
| Input `"images"` | `chw_input_buf_` (uint8) | [1, 3, 320, 320] |
| Output `"output0"` | `det_output_buf_` (float) | [1, 84, 2100] |
| Output `latent_layer_name` | `latent_output_buf_` (float) | [1, 256, 20, 20] |

## DepthNode Bindings
| Binding | Buffer | Shape |
|---|---|---|
| Input `"input"` | `midas_chw_buf_` (float) | [1, 3, 256, 256] |
| Output `"output"` | `depth_output_buf_` (float) | [1, 256, 256] |

## Links
- Used by: [[vision-node]], [[depth-node]]
- Runtime: [[onnx-runtime]]
- Models: [[yolo-int8]], [[midas-fp32]]

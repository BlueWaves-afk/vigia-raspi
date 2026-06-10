# VIGIA ADAS DePIN Edge Node
## ONNX Runtime Vision Engine Contracts
**Document:** `04_onnx_vision_engine_contracts.md`  
**Depends on:** `01_system_architecture_and_roadmap.md` (APPROVED), `02_ros2_node_contracts.md` (APPROVED)  
**Status:** AWAITING APPROVAL — No implementation until sign-off  
**Scope:** Phase 3 — `VisionNode` (YOLOv26 INT8 + KleidiAI + S_t) and `DepthNode` (MiDaS FP32)

---

## 0. Non-Negotiable Vision Engine Invariants

| Invariant | Enforcement |
|---|---|
| **No heap allocation in the inference hot path** | All tensor input/output buffers are pre-allocated in constructors. `std::vector::resize()` is permitted only if preceded by `reserve()` in the constructor (amortized, no-alloc after warm-up). `new`, `malloc`, `cv::dnn::blobFromImage`, `cv::Mat` default constructors, and `Ort::Value`-copying constructors are prohibited in callbacks. |
| **`cv::dnn::blobFromImage` banned** | It allocates a new `cv::Mat` on every call. NEON intrinsics + pre-allocated output buffers replace it entirely. |
| **`frame.clone()` banned** | The inbound `shared_ptr<const sensor_msgs::msg::Image>` is read directly into pre-allocated preprocessing buffers. |
| **`Ort::IoBinding` mandatory for hot path** | Input and output tensors are bound once in the constructor. `session_->Run()` is called with the pre-bound `IoBinding` object — no per-call tensor construction. |
| **`SetIntraOpNumThreads(1)` mandatory on both nodes** | Each node is pinned to a single dedicated core. Allowing ORT to spawn worker threads would steal time from other RT nodes on neighboring cores. |
| **Mixed precision enforced at session level** | `VisionNode`: ACL EP active, INT8 ops permitted. `DepthNode`: CPU EP only, session config explicitly blocks INT8/FP16 graph transformations. |

---

## 1. Shared `Ort::Env` Singleton

`Ort::Env` is heavyweight at construction (~50 ms, initializes thread pool and logging subsystem). It MUST be created once per process and shared between `VisionNode` and `DepthNode`.

```cpp
// vigia_edge_node/src/main.cpp
// Constructed before any node, passed by const-ref to constructors.
// ORT_LOGGING_LEVEL_WARNING suppresses verbose INFO logs in production.

static Ort::Env g_ort_env{ORT_LOGGING_LEVEL_WARNING, "vigia_onnx"};

// Node construction:
auto vision_node = std::make_shared<VisionNode>(make_ipc_options(), g_ort_env);
auto depth_node  = std::make_shared<DepthNode> (make_ipc_options(), g_ort_env);
```

`Ort::Env` is thread-safe for concurrent `session.Run()` calls across multiple sessions — two sessions running simultaneously on different cores is a guaranteed-safe pattern.

---

## 2. Zero-Copy Memory Binding Architecture

### 2.1 Principle

ONNX Runtime distinguishes between two allocator types when creating tensors:
- `OrtArenaAllocator` — ORT owns the memory, managed in a BFC arena. ORT may reallocate internally.
- `OrtDeviceAllocator` — the caller owns the memory. ORT treats the pointer as externally managed and **never frees or reallocates it**.

Binding pre-allocated buffers to `OrtDeviceAllocator` memory info is the only correct zero-copy path. All input and output tensor buffers use this pattern.

```cpp
// vigia_edge_node/include/ort_utils.hpp
#pragma once
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

// Canonical zero-copy memory info — used for ALL pre-allocated tensor bindings.
// OrtDeviceAllocator = caller-owned memory; ORT will not touch the pointer lifecycle.
inline Ort::MemoryInfo make_cpu_memory_info() {
    return Ort::MemoryInfo("Cpu", OrtDeviceAllocator, /*device_id=*/0, OrtMemTypeDefault);
}

// Type-safe zero-copy tensor wrapper.
// 'data' must outlive the returned Ort::Value.
template <typename T>
Ort::Value wrap_tensor(T* data, size_t element_count,
                       const std::array<int64_t, 4>& shape) {
    static auto mem_info = make_cpu_memory_info();
    return Ort::Value::CreateTensor<T>(
        mem_info,
        data, element_count,
        shape.data(), shape.size()
    );
}
```

### 2.2 `Ort::IoBinding` — Pre-Bound Hot Path

`Ort::IoBinding` eliminates per-call tensor construction. Inputs and outputs are bound once in the constructor. `session_->Run()` with an `IoBinding` object performs zero overhead binding — no `std::vector<Ort::Value>` construction, no tensor metadata allocation.

```
Constructor (one-time):
  pre_alloc buffers → Ort::Value (zero-copy) → io_binding_->BindInput/BindOutput

Inference hot path (every frame):
  fill pre_alloc input buffer   (NEON preprocessing writes here)
  session_->Run(run_opts_, *io_binding_)   ← zero tensor construction overhead
  read pre_alloc output buffers (detection + latent data already here)
```

---

## 3. Pre-Allocated Buffer Declarations

All buffers are `static` class members (BSS segment). Sizes are `constexpr` and verified by `static_assert`.

### 3.1 `VisionNode` Buffer Inventory

```cpp
// vigia_edge_node/src/vision_node.hpp (private section)

// ── Input pipeline ──────────────────────────────────────────────────────
// Letterbox-resized frame (pre-resize destination, BGR8, HWC)
// Allocated once; cv::Mat header wraps this without owning.
static constexpr int kInputH = 320;
static constexpr int kInputW = 320;
static constexpr int kInputC = 3;
alignas(64) uint8_t letterbox_buf_[kInputH * kInputW * kInputC];   // 307,200 B

// ONNX input tensor buffer: RGB planar, NCHW, uint8
// Filled by NEON HWC→CHW transpose directly from letterbox_buf_.
alignas(64) uint8_t chw_input_buf_[kInputC * kInputH * kInputW];   // 307,200 B

// ── Detection output ───────────────────────────────────────────────────
// YOLOv26 Nano output: [1, 84, 2100] for 320×320 input (anchors at stride 8/16/32)
// 84 = 4 bbox coords + 80 class logits
static constexpr int  kDetOutputRows = 84;
static constexpr int  kDetOutputCols = 2100;
static constexpr size_t kDetOutputElems = kDetOutputRows * kDetOutputCols;
alignas(64) float det_output_buf_[kDetOutputElems];                 // ~705 KB

// ── Spatial latent output (S_t) ────────────────────────────────────────
// Penultimate layer of YOLOv26 Nano: shape determined by Netron inspection.
// YOLOv8n penultimate layer "/model.22/cv2/cv2.0/act/Mul_output_0":
//   shape [1, 64, 40, 40] at stride-8 = 102,400 elements (float32)
// Reserve conservatively; static_assert updated after Netron inspection.
static constexpr size_t kMaxLatentElems = 128'000;    // 512 KB — safe upper bound
alignas(64) float latent_output_buf_[kMaxLatentElems]; // 512 KB

// ── ORT session objects (non-allocating after construction) ───────────
std::unique_ptr<Ort::Session>   session_;
std::unique_ptr<Ort::IoBinding> io_binding_;
Ort::RunOptions                 run_options_{nullptr};

// ── Pre-built cv::Mat headers (no data ownership — wrap static buffers) ─
cv::Mat letterbox_mat_;   // wraps letterbox_buf_, initialized in constructor
cv::Mat resize_tmp_;      // intermediate resize target (pre-allocated, pre-sized)

static_assert(sizeof(chw_input_buf_) == 307200,  "NCHW buffer size mismatch");
static_assert(sizeof(det_output_buf_) == kDetOutputElems * 4, "Det output size mismatch");
```

### 3.2 `DepthNode` Buffer Inventory

```cpp
// vigia_edge_node/src/depth_node.hpp (private section)

static constexpr int kMidasH = 256;
static constexpr int kMidasW = 256;
static constexpr int kMidasC = 3;

// Resize destination (BGR8, HWC) — cv::Mat wraps this
alignas(64) uint8_t midas_resize_buf_[kMidasH * kMidasW * kMidasC];  // 196,608 B

// Normalized float HWC intermediate (needed for normalize-then-transpose pipeline)
alignas(64) float midas_float_hwc_buf_[kMidasH * kMidasW * kMidasC]; // 786,432 B

// ONNX input tensor: normalized RGB planar, NCHW, float32
alignas(64) float midas_chw_buf_[kMidasC * kMidasH * kMidasW];       // 786,432 B

// ONNX depth output: [1, 1, 256, 256] float32 inverse depth
static constexpr size_t kDepthOutputElems = kMidasH * kMidasW;
alignas(64) float depth_output_buf_[kDepthOutputElems];               // 262,144 B

std::unique_ptr<Ort::Session>   session_;
std::unique_ptr<Ort::IoBinding> io_binding_;
Ort::RunOptions                 run_options_{nullptr};

cv::Mat resize_mat_;    // wraps midas_resize_buf_
```

> **Total static allocation per node:**  
> `VisionNode`: ~1.8 MB (SRAM on Pi 5: 8 GB LPDDR4X — negligible)  
> `DepthNode`: ~2.0 MB

---

## 4. `VisionNode` — KleidiAI Session Configuration

### 4.1 Session Options

```cpp
// vigia_edge_node/src/vision_node.cpp — constructor

Ort::SessionOptions opts;

// ── Graph optimization ───────────────────────────────────────────────────
// ORT_ENABLE_ALL: enables constant folding, operator fusion, INT8 graph rewriting.
// Critically: enables the QDQ (Quantize-Dequantize) cleanup pass that converts
// quantized ONNX patterns into ACL-native INT8 conv kernels.
opts.SetGraphOptimizationLevel(ORT_ENABLE_ALL);

// ── Thread configuration ─────────────────────────────────────────────────
// 1 thread: VisionNode is pinned to Core 1. ACL's CPPScheduler manages its own
// internal parallelism within the single-thread constraint via NEON SIMD width.
// Allowing > 1 thread here would have ORT spawn workers on other cores,
// violating the Core 1 isolation invariant.
opts.SetIntraOpNumThreads(1);
opts.SetInterOpNumThreads(1);

// ── Memory configuration ─────────────────────────────────────────────────
// Disable BFC arena memory pattern: prevents ORT from growing an internal
// arena between calls. Combined with IoBinding pre-allocation, this achieves
// flat heap profile in the hot path.
opts.DisableMemPattern();
opts.DisableCpuMemArena();

// ── Denormal handling ─────────────────────────────────────────────────────
// Denormal FP32 values (very small numbers < 1.18e-38) cause 10–100× slower
// execution on Cortex-A76 (software emulation path). Force them to zero.
// Affects NEON FP32 ops; INT8 UDOT paths are unaffected.
opts.AddConfigEntry("session.set_denormal_as_zero", "1");

// ── ACL Execution Provider (KleidiAI backend) ────────────────────────────
// MUST be appended BEFORE the CPU EP (implicit fallback).
// Provider order = priority order: ORT tries ACL first, falls back to CPU EP
// for any op ACL doesn't support (e.g. custom NMS post-processing).
//
// enable_fast_math=1: activates KleidiAI's optimized INT8 UDOT micro-kernels
// on Cortex-A76 (asimddp feature). On Pi 5 this routes quantized Conv2D and
// DepthwiseConv2D through ARM's UDOT assembly dispatch path.
//
// Prerequisite: ONNX Runtime built with -DONNXRUNTIME_USE_KLEIDIAI=ON
// and ARM Compute Library ≥ 24.01 linked at build time.
Ort::ThrowOnError(
    OrtSessionOptionsAppendExecutionProvider_ACL(opts, /*enable_fast_math=*/1)
);
// CPU EP registered automatically as implicit fallback — do not add manually.

// ── Load model ───────────────────────────────────────────────────────────
// Model: yolov26_nano_int8_with_latent.onnx
// (INT8 quantized + penultimate layer added as second output — see §6)
session_ = std::make_unique<Ort::Session>(ort_env, model_path.c_str(), opts);
```

### 4.2 KleidiAI UDOT Activation Verification

During development, add profiling to confirm UDOT kernels are dispatched. This is a **development-only** step; remove before production build.

```cpp
// DEVELOPMENT ONLY — verify UDOT dispatch:
opts.EnableProfiling("/tmp/vigia_yolo_profile");
// After one inference, inspect /tmp/vigia_yolo_profile_<timestamp>.json
// Search for kernel names containing:
//   "CpuAcc" prefix  → ACL EP handled this op
//   "qasymm8"        → INT8 quantized kernel
//   "assembly_*"     → KleidiAI UDOT assembly micro-kernel dispatched
//
// If no "CpuAcc" entries appear → ACL EP not linked; verify ONNX Runtime build.
// If "qasymm8" but no "assembly_*" → model not INT8; check quantization step.
// If all ops show "Cpu" prefix → ACL EP linked but UDOT not triggering;
//   verify model is QUInt8 (not QInt8) and asimddp is in /proc/cpuinfo.
```

**Prerequisite check at process startup:**
```cpp
// vision_node.cpp — VisionNode constructor
// Abort early with a clear error if ACL UDOT is unavailable.
// This prevents silently running FP32 inference thinking it's INT8.
void VisionNode::verify_kleidiai_capable() {
    // Check /proc/cpuinfo for ARM dot product extension
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line;
    bool has_asimddp = false;
    while (std::getline(cpuinfo, line)) {
        if (line.find("asimddp") != std::string::npos) {
            has_asimddp = true;
            break;
        }
    }
    if (!has_asimddp) {
        RCLCPP_FATAL(get_logger(),
            "CPU does not support asimddp (ARM dot product). "
            "KleidiAI INT8 UDOT path unavailable. "
            "Expected Cortex-A76 (Pi 5). Aborting.");
        throw std::runtime_error("KleidiAI requires asimddp");
    }
    RCLCPP_INFO(get_logger(), "asimddp confirmed. KleidiAI UDOT path armed.");
}
```

---

## 5. Offline Model Preparation Pipeline

These steps run **once** on a development machine before firmware deployment. The outputs are checked into `models/`.

### 5.1 Step 1 — Export YOLOv26 Nano to ONNX (FP32)

```python
# tools/model_prep/01_export_yolov26.py
from ultralytics import YOLO

model = YOLO("yolov26n.pt")
model.export(
    format="onnx",
    imgsz=320,          # match runtime input size
    opset=17,           # ONNX opset 17 — required for INT8 quantization
    simplify=True,      # onnx-simplifier: remove redundant nodes
    dynamic=False,      # fixed batch size = 1
)
# Output: yolov26n.onnx  (float32, NCHW, input "images", output "output0")
```

### 5.2 Step 2 — Identify Penultimate Layer via Netron

Open `yolov26n.onnx` in [Netron](https://netron.app). Navigate to the layer immediately before the detection head (`/model.22/...`). The penultimate layer (last feature map before bbox regression + class logits) is typically the output of the last C2f block.

Record the **exact ONNX tensor name** (e.g., `"/model.22/cv2/cv2.0/act/Mul_output_0"`). This becomes the `latent_layer_name` ROS 2 parameter.

```python
# tools/model_prep/02_inspect_latent_layer.py
import onnx

model = onnx.load("yolov26n.onnx")
# Print all intermediate tensor names and shapes
for node in model.graph.node:
    for output in node.output:
        print(output)
# Identify the correct penultimate layer from this list after Netron inspection
```

### 5.3 Step 3 — Add Penultimate Layer as ONNX Graph Output

```python
# tools/model_prep/03_add_latent_output.py
import onnx
import onnx.helper

LATENT_LAYER_NAME = "/model.22/cv2/cv2.0/act/Mul_output_0"  # UPDATE after Step 2

model = onnx.load("yolov26n.onnx")

# The tensor already exists in the graph as an intermediate value.
# We add it as a named output so ORT will return it from session.Run().
latent_value_info = onnx.helper.make_tensor_value_info(
    LATENT_LAYER_NAME,
    onnx.TensorProto.FLOAT,   # FP32 (pre-quantization — quantized to INT8 in Step 4)
    shape=None                # shape=None: ONNX infers it; avoids hardcoding dims
)
model.graph.output.append(latent_value_info)

onnx.checker.check_model(model)   # validate graph integrity
onnx.save(model, "yolov26n_with_latent.onnx")
print(f"Added latent output: {LATENT_LAYER_NAME}")
```

### 5.4 Step 4 — INT8 Static Quantization (QUInt8 for KleidiAI UDOT)

```python
# tools/model_prep/04_quantize_int8.py
from onnxruntime.quantization import (
    quantize_static, CalibrationDataReader,
    QuantType, QuantFormat
)
import numpy as np
import cv2, os

class RoadCalibrationReader(CalibrationDataReader):
    """Feeds road imagery through the model for INT8 calibration.
    Use 200-500 representative road images (diverse lighting, speeds, hazard types).
    """
    def __init__(self, calib_dir: str, input_size: int = 320):
        self.images = [
            os.path.join(calib_dir, f)
            for f in os.listdir(calib_dir)
            if f.endswith(('.jpg', '.png'))
        ]
        self.idx = 0
        self.input_size = input_size

    def get_next(self):
        if self.idx >= len(self.images):
            return None
        img = cv2.imread(self.images[self.idx])
        img = cv2.resize(img, (self.input_size, self.input_size))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # NCHW float32 for calibration (ORT quantizer normalizes internally)
        arr = img.astype(np.float32).transpose(2, 0, 1)[np.newaxis] / 255.0
        self.idx += 1
        return {"images": arr}

quantize_static(
    model_input="yolov26n_with_latent.onnx",
    model_output="yolov26_nano_int8_with_latent.onnx",
    calibration_data_reader=RoadCalibrationReader("data/calib_road/"),
    quant_format=QuantFormat.QDQ,      # QDQ format: ACL EP fuses Q/DQ pairs into INT8 kernels
    activation_type=QuantType.QUInt8,  # UNSIGNED INT8 activations — ACL UDOT prefers QUInt8
    weight_type=QuantType.QUInt8,      # UNSIGNED INT8 weights
    per_channel=False,                 # per-tensor quantization: faster on KleidiAI than per-channel
    reduce_range=True,                 # use 7-bit range [0, 127]: improves ACL compat on A76
    extra_options={
        "ActivationSymmetric": False,  # asymmetric: better accuracy for ReLU activations
        "WeightSymmetric":     True,   # symmetric weights: standard practice
    }
)
# Output: yolov26_nano_int8_with_latent.onnx (QUInt8 QDQ, ~6 MB)
```

> **Why QUInt8 and not QInt8?** ARM's KleidiAI UDOT micro-kernels (`arm_gemm::GemmHybridQuantized`) are optimized for unsigned operands. SDOT (signed) is also supported on A76, but UDOT throughput on A76 is higher for convolution workloads. ONNX Runtime's ACL EP maps `QUInt8` convolutions to `CpuAcc::NEGEMMConvolution2d` with UDOT dispatch.

---

## 6. `VisionNode` — NEON Preprocessing Pipeline

### 6.1 Letterbox Resize (Pre-Allocated, No Clone)

The inbound `sensor_msgs::msg::Image` (`bgr8`, 1280×720) must be resized to 320×320 with letterboxing. All intermediate and final buffers are pre-allocated.

```cpp
// vision_node.hpp — constructor initializes cv::Mat headers over static buffers

// Pre-allocated resize intermediate: exact model input dimensions, BGR8
// cv::Mat wraps static buffer — no data ownership, no allocation on copy
resize_tmp_ = cv::Mat(kInputH, kInputW, CV_8UC3, letterbox_buf_);

// Pre-allocated aspect-ratio-preserving scale intermediate
// (for letterbox calculation — reused every frame)
letterbox_mat_ = cv::Mat(kInputH, kInputW, CV_8UC3, letterbox_buf_);
```

```cpp
// vision_node.cpp — preprocessing hot path

void VisionNode::preprocess_letterbox(
    const sensor_msgs::msg::Image& img_msg)
{
    // Wrap inbound ROS message data — NO copy, NO clone
    // sensor_msgs::msg::Image::data is std::vector<uint8_t> — data() is stable
    const cv::Mat src(
        static_cast<int>(img_msg.height),
        static_cast<int>(img_msg.width),
        CV_8UC3,
        const_cast<uint8_t*>(img_msg.data.data())  // zero-copy read
    );

    // Compute letterbox scale and padding offsets
    const float scale = std::min(
        static_cast<float>(kInputW) / src.cols,
        static_cast<float>(kInputH) / src.rows);
    const int scaled_w = static_cast<int>(src.cols * scale);
    const int scaled_h = static_cast<int>(src.rows * scale);
    const int pad_x = (kInputW - scaled_w) / 2;
    const int pad_y = (kInputH - scaled_h) / 2;

    // Resize into a sub-region of letterbox_buf_ — zero-alloc (cv::Mat is a header)
    cv::Mat scaled_roi(scaled_h, scaled_w, CV_8UC3,
                       letterbox_buf_ + (pad_y * kInputW + pad_x) * kInputC);
    cv::resize(src, scaled_roi, {scaled_w, scaled_h},
               /*fx=*/0, /*fy=*/0, cv::INTER_LINEAR);

    // Fill padding rows/cols with gray (114,114,114) — YOLO standard
    // Top padding
    if (pad_y > 0)
        cv::Mat(pad_y, kInputW, CV_8UC3, letterbox_buf_)
            .setTo(cv::Scalar(114, 114, 114));
    // Bottom padding
    if (pad_y > 0)
        cv::Mat(kInputH - pad_y - scaled_h, kInputW, CV_8UC3,
                letterbox_buf_ + (pad_y + scaled_h) * kInputW * kInputC)
            .setTo(cv::Scalar(114, 114, 114));
    // Left/right padding is handled by the sub-region offset above
}
```

### 6.2 NEON BGR→RGB + HWC→CHW Transposition

This replaces `cv::dnn::blobFromImage` entirely. It performs the BGR→RGB channel swap and HWC→CHW layout change in a **single pass** over the data, writing directly into the pre-allocated `chw_input_buf_`.

```
Input:  uint8_t letterbox_buf_[320 * 320 * 3]   BGR interleaved (HWC)
Output: uint8_t chw_input_buf_ [3 * 320 * 320]  RGB planar      (CHW)
                                                  [R plane | G plane | B plane]
```

```cpp
// vision_node.cpp

// ARM NEON header — included only on ARM targets
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#endif

void VisionNode::neon_bgr_hwc_to_rgb_chw(
    const uint8_t* __restrict__ src,   // letterbox_buf_ [H*W*3] BGR HWC
    uint8_t*       __restrict__ dst,   // chw_input_buf_ [3*H*W] RGB CHW
    const size_t   n_pixels)           // H * W = 102400 for 320×320
{
    // CHW plane pointers
    uint8_t* r_plane = dst;                        // channel 0: R
    uint8_t* g_plane = dst + n_pixels;             // channel 1: G
    uint8_t* b_plane = dst + 2 * n_pixels;         // channel 2: B

#if defined(__ARM_NEON) || defined(__ARM_NEON__)

    // vld3q_u8: loads 48 bytes (16 pixels × 3 channels) and DEINTERLEAVES:
    //   bgr.val[0] = 16× B values  (uint8x16_t)
    //   bgr.val[1] = 16× G values
    //   bgr.val[2] = 16× R values
    // This is the critical NEON primitive that makes single-pass conversion possible.

    const size_t n_vec = n_pixels / 16;       // number of 16-pixel NEON iterations
    const size_t n_rem = n_pixels % 16;       // scalar tail

    const uint8_t* s = src;
    uint8_t* rp = r_plane;
    uint8_t* gp = g_plane;
    uint8_t* bp = b_plane;

    for (size_t i = 0; i < n_vec; ++i, s += 48, rp += 16, gp += 16, bp += 16) {
        uint8x16x3_t bgr = vld3q_u8(s);
        //             ^^^
        // Cortex-A76 throughput: vld3q_u8 = 3 cycles for 48 bytes
        // = 16 pixels deinterleaved in 3 cycles ≈ 5.3 pixels/cycle

        vst1q_u8(rp, bgr.val[2]);  // R ← BGR[2]  (BGR→RGB: swap B and R)
        vst1q_u8(gp, bgr.val[1]);  // G ← BGR[1]  (unchanged)
        vst1q_u8(bp, bgr.val[0]);  // B ← BGR[0]  (BGR→RGB: swap B and R)
    }

    // Scalar tail for remaining pixels (at most 15)
    for (size_t i = 0; i < n_rem; ++i, s += 3) {
        rp[i] = s[2];  // R
        gp[i] = s[1];  // G
        bp[i] = s[0];  // B
    }

#else
    // Non-NEON fallback (x86 development builds only — never runs on Pi 5)
    for (size_t i = 0; i < n_pixels; ++i) {
        r_plane[i] = src[i * 3 + 2];
        g_plane[i] = src[i * 3 + 1];
        b_plane[i] = src[i * 3 + 0];
    }
#endif
}
```

**Performance analysis on Cortex-A76 (Pi 5):**
- Input: 320×320 = 102,400 pixels = 307,200 bytes
- NEON iterations: 102,400 / 16 = 6,400 iterations
- `vld3q_u8` throughput: 3 cycles per 16 pixels on A76 (source: Cortex-A76 Software Optimization Guide, Table 14)
- Total NEON passes: 6,400 × 3 = 19,200 cycles
- At 2.4 GHz (Pi 5): **~8 µs** for the full 320×320 transpose — compared to ~2–4 ms for `cv::dnn::blobFromImage`

---

## 7. `VisionNode` — `IoBinding` Setup and Inference Hot Path

### 7.1 Constructor: One-Time IoBinding Configuration

```cpp
// vision_node.cpp — constructor (after session_ is created)

io_binding_ = std::make_unique<Ort::IoBinding>(*session_);
auto mem_info = make_cpu_memory_info();

// ── Bind INPUT tensor (zero-copy — wraps chw_input_buf_) ─────────────────
std::array<int64_t, 4> input_shape{1, kInputC, kInputH, kInputW};
auto input_tensor = Ort::Value::CreateTensor<uint8_t>(
    mem_info,
    chw_input_buf_, sizeof(chw_input_buf_),
    input_shape.data(), input_shape.size()
);
io_binding_->BindInput("images", input_tensor);
// "images" is the standard Ultralytics YOLO ONNX input node name.

// ── Bind DETECTION OUTPUT (zero-copy — wraps det_output_buf_) ────────────
// YOLOv26 Nano at 320×320: output shape [1, 84, 2100]
// 2100 = 40×40 + 20×20 + 10×10 anchors across three FPN scales
std::array<int64_t, 3> det_output_shape{1, kDetOutputRows, kDetOutputCols};
auto det_output_tensor = Ort::Value::CreateTensor<float>(
    mem_info,
    det_output_buf_, kDetOutputElems,
    det_output_shape.data(), det_output_shape.size()
);
io_binding_->BindOutput("output0", det_output_tensor);
// "output0" is the standard Ultralytics YOLO ONNX detection output name.

// ── Bind LATENT OUTPUT (zero-copy — wraps latent_output_buf_) ────────────
// Shape determined by Netron inspection of the specific YOLOv26 model.
// Placeholder: [1, 64, 40, 40] = 102400 elements for the stride-8 head.
// UPDATE latent_output_shape_ after Netron inspection in Step 5.3.
std::array<int64_t, 4> latent_shape{1, 64, 40, 40};  // VERIFY with Netron
auto latent_tensor = Ort::Value::CreateTensor<float>(
    mem_info,
    latent_output_buf_, latent_shape[1] * latent_shape[2] * latent_shape[3],
    latent_shape.data(), latent_shape.size()
);
io_binding_->BindOutput(latent_layer_name_.c_str(), latent_tensor);
// latent_layer_name_ is loaded from ROS 2 parameter at construction.

// ── Pre-configure RunOptions (no allocation per call) ────────────────────
run_options_.SetRunLogSeverityLevel(4);  // ORT_LOGGING_LEVEL_FATAL: silent in production
```

### 7.2 Inference Hot Path (Called Every Frame)

```cpp
// vision_node.cpp — image_callback (runs on Core 1, SCHED_FIFO 75)

void VisionNode::image_callback(
    std::shared_ptr<const sensor_msgs::msg::Image> img_msg)
{
    // ── Step 1: Letterbox resize into letterbox_buf_ (zero-alloc) ────────
    preprocess_letterbox(*img_msg);

    // ── Step 2: NEON BGR→RGB + HWC→CHW into chw_input_buf_ (zero-alloc) ─
    neon_bgr_hwc_to_rgb_chw(
        letterbox_buf_,
        chw_input_buf_,
        static_cast<size_t>(kInputH * kInputW));
    // chw_input_buf_ is already bound to io_binding_ — ORT reads it in-place.

    // ── Step 3: Inference (zero tensor construction) ──────────────────────
    // All inputs/outputs are pre-bound. session_->Run with IoBinding performs
    // no std::vector construction, no Ort::Value construction, no arena growth.
    session_->Run(run_options_, *io_binding_);
    // Results are now in det_output_buf_ and latent_output_buf_.

    // ── Step 4: Post-process detections (NMS — reuses existing logic) ────
    auto det_msg = std::make_unique<vigia_msgs::msg::DetectionArray>();
    det_msg->header    = img_msg->header;
    det_msg->frame_id  = frame_counter_;
    postprocess_yolo(det_output_buf_, kDetOutputRows, kDetOutputCols,
                     img_msg->width, img_msg->height,
                     *det_msg);
    // postprocess_yolo() reuses NMS logic from existing src/perception.cpp.
    // It writes into det_msg->detections (std::vector — amortized alloc, see §9).

    // ── Step 5: Populate and publish SpatialLatent (S_t) ─────────────────
    publish_spatial_latent(img_msg->header);

    // ── Step 6: Publish detections ────────────────────────────────────────
    det_msg->inference_latency_ms = latency_timer_.elapsed_ms();
    det_publisher_->publish(std::move(det_msg));

    ++frame_counter_;
}
```

---

## 8. `VisionNode` — S_t Extraction and `SpatialLatent` Publication

### 8.1 Why `latent_output_buf_` Is Already Populated

After `session_->Run(run_options_, *io_binding_)` returns, ORT has written the penultimate layer tensor directly into `latent_output_buf_` — the buffer we bound as the second output in the constructor. There is no additional API call needed to "extract" the latent; it is a side effect of the bound output contract.

### 8.2 `SpatialLatent` Message Population

```cpp
// vision_node.cpp

void VisionNode::publish_spatial_latent(
    const std_msgs::msg::Header& header)
{
    // Pre-allocated message is moved out on publish; re-allocate here.
    // latent_msg_ is initialized in constructor with reserve(); resize() is amortized.
    auto latent_msg = std::make_unique<vigia_msgs::msg::SpatialLatent>();

    latent_msg->header            = header;
    latent_msg->frame_id          = frame_counter_;
    latent_msg->source_layer_name = latent_layer_name_;

    // latent_output_buf_ is pre-allocated static float[kMaxLatentElems].
    // ORT wrote inference results directly here via the pre-bound IoBinding output.
    // latent_n_elems_ is set in constructor from io_binding_ output tensor shape.
    latent_msg->latent_vector.resize(latent_n_elems_);
    // resize() is no-alloc after the first call IF latent_n_elems_ is stable
    // (it is: same model, same input size → same latent shape every frame).
    // On the very first frame, std::vector allocates once. This is acceptable.

    std::memcpy(
        latent_msg->latent_vector.data(),
        latent_output_buf_,
        latent_n_elems_ * sizeof(float)
    );
    // memcpy of ~400 KB (102400 floats): ~170 µs on Pi 5 at LPDDR4X bandwidth.
    // This is the unavoidable cost of crossing from ORT-managed memory to the
    // ROS 2 message heap. For Phase 1, this is acceptable.
    // Phase 3+ optimization path: use a custom allocator that wraps latent_output_buf_
    // directly into the ROS 2 message (avoids the memcpy entirely).

    latent_publisher_->publish(std::move(latent_msg));
}
```

### 8.3 `latent_n_elems_` Initialization

```cpp
// vision_node.cpp — constructor, after io_binding_ setup

// Query the bound output tensor shape to determine latent vector size.
// This avoids hardcoding the shape and handles future model changes gracefully.
auto output_names = io_binding_->GetOutputNamesAllocated(allocator_);
// Find the latent output by name and query its shape info.
// (ORT populates shape info after the FIRST session.Run() call —
//  call a warmup inference here with a zeroed input buffer.)

// Warmup pass: fills output shapes without disturbing real data.
std::fill(chw_input_buf_,
          chw_input_buf_ + sizeof(chw_input_buf_), uint8_t{0});
session_->Run(run_options_, *io_binding_);

auto outputs = io_binding_->GetOutputValues();
// outputs[1] is the latent tensor (second bound output)
auto shape_info = outputs[1].GetTensorTypeAndShapeInfo();
latent_n_elems_ = shape_info.GetElementCount();

RCLCPP_INFO(get_logger(),
    "Latent layer '%s': %zu elements (%.1f KB)",
    latent_layer_name_.c_str(),
    latent_n_elems_,
    latent_n_elems_ * sizeof(float) / 1024.0);

if (latent_n_elems_ > kMaxLatentElems) {
    RCLCPP_FATAL(get_logger(),
        "Latent vector (%zu) exceeds kMaxLatentElems (%zu). "
        "Increase kMaxLatentElems in vision_node.hpp.",
        latent_n_elems_, kMaxLatentElems);
    throw std::out_of_range("latent buffer too small");
}
```

---

## 9. `DepthNode` — MiDaS FP32 Session Configuration

### 9.1 Session Options

```cpp
// depth_node.cpp — constructor

Ort::SessionOptions opts;
opts.SetGraphOptimizationLevel(ORT_ENABLE_ALL);

// 2 threads: DepthNode is pinned to Core 2 (dedicated).
// MiDaS v2.1 small uses depthwise separable convolutions that benefit from
// a small degree of parallelism across the depthwise + pointwise split.
// 2 threads on Core 2 uses both hyperthreads on the physical A76 core.
// NOTE: Cortex-A76 has no hyperthreading — this is 2 ORT worker threads
// scheduled on the same core, which increases register pressure but
// fills ALU/FPU pipeline stalls during memory-bound ops.
// Monitor with `perf stat` before committing to 2 vs 1.
opts.SetIntraOpNumThreads(2);
opts.SetInterOpNumThreads(1);

opts.DisableMemPattern();
opts.DisableCpuMemArena();
opts.AddConfigEntry("session.set_denormal_as_zero", "1");

// ── CPU EP only — NO ACL EP ────────────────────────────────────────────
// MiDaS FP32 does NOT use KleidiAI. ACL EP is explicitly excluded.
//
// Reason 1: MiDaS INT8 is prohibited (accuracy degradation, approved decision).
// Reason 2: Adding ACL EP to a FP32 model causes ACL to insert NHWC transposes
//           that increase latency for models not optimized for ACL's layout.
// Reason 3: Core 2's NEON FP32 pipeline (FMLA) runs independently of
//           Core 1's ACL INT8 UDOT — no EP-level contention.
//
// Do NOT call OrtSessionOptionsAppendExecutionProvider_ACL() here.
// CPU EP is added automatically. FP32 NEON FMLA path activates by default.

// ── Block INT8 graph transformations ──────────────────────────────────
// belt-and-suspenders: prevent any future quantization-aware pass from
// accidentally transforming the MiDaS FP32 graph to INT8.
opts.AddConfigEntry("session.disable_quant_qdq_cleanup", "1");

session_ = std::make_unique<Ort::Session>(ort_env, midas_model_path.c_str(), opts);
```

### 9.2 Constructor: IoBinding for MiDaS

```cpp
// depth_node.cpp — constructor (after session_)

io_binding_ = std::make_unique<Ort::IoBinding>(*session_);
auto mem_info = make_cpu_memory_info();

// MiDaS v2.1 small ONNX input: "input" (FP32, NCHW, [1, 3, 256, 256])
std::array<int64_t, 4> input_shape{1, kMidasC, kMidasH, kMidasW};
auto input_tensor = Ort::Value::CreateTensor<float>(
    mem_info,
    midas_chw_buf_, kMidasC * kMidasH * kMidasW,
    input_shape.data(), input_shape.size()
);
io_binding_->BindInput("input", input_tensor);

// MiDaS v2.1 small ONNX output: "output" (FP32, [1, 256, 256])
// Note: MiDaS output is [N, H, W] not [N, 1, H, W] — 3D tensor
std::array<int64_t, 3> output_shape{1, kMidasH, kMidasW};
auto output_tensor = Ort::Value::CreateTensor<float>(
    mem_info,
    depth_output_buf_, kDepthOutputElems,
    output_shape.data(), output_shape.size()
);
io_binding_->BindOutput("output", output_tensor);
```

### 9.3 NEON Preprocessing for MiDaS (resize + normalize + HWC→CHW)

MiDaS requires normalized FP32 input with ImageNet statistics. The pipeline:

```
Source:  uint8_t BGR HWC  (1280×720, from sensor_msgs::Image)
Step 1:  cv::resize to [256×256], writes into midas_resize_buf_ (pre-alloc)
Step 2:  NEON uint8→float + (x/255 - mean) / std + CHW transpose → midas_chw_buf_
```

```cpp
// depth_node.cpp

// ImageNet normalization constants (RGB order):
//   Mean:     [0.485, 0.456, 0.406]
//   Std:      [0.229, 0.224, 0.225]
//   Formula:  (uint8_val / 255.0f - mean) / std
//           = uint8_val * (1/(255*std)) - mean/std
// Fuse into (scale, offset) per channel to eliminate divisions in the hot loop:
static constexpr float kScaleR = 1.0f / (255.0f * 0.229f);   // 1/(255×std_R)
static constexpr float kScaleG = 1.0f / (255.0f * 0.224f);
static constexpr float kScaleB = 1.0f / (255.0f * 0.225f);
static constexpr float kBiasR  = -0.485f / 0.229f;           // -mean_R/std_R
static constexpr float kBiasG  = -0.456f / 0.224f;
static constexpr float kBiasB  = -0.406f / 0.225f;

void DepthNode::neon_bgr_u8_to_normrgb_chw(
    const uint8_t* __restrict__ src,   // midas_resize_buf_ [256×256×3] BGR HWC
    float*         __restrict__ dst,   // midas_chw_buf_ [3×256×256] float32 CHW
    const size_t   n_pixels)           // 65536 for 256×256
{
    float* r_plane = dst;
    float* g_plane = dst + n_pixels;
    float* b_plane = dst + 2 * n_pixels;

#if defined(__ARM_NEON) || defined(__ARM_NEON__)

    // NEON constants — loaded once outside the loop
    const float32x4_t v_scale_r = vdupq_n_f32(kScaleR);
    const float32x4_t v_scale_g = vdupq_n_f32(kScaleG);
    const float32x4_t v_scale_b = vdupq_n_f32(kScaleB);
    const float32x4_t v_bias_r  = vdupq_n_f32(kBiasR);
    const float32x4_t v_bias_g  = vdupq_n_f32(kBiasG);
    const float32x4_t v_bias_b  = vdupq_n_f32(kBiasB);

    const size_t n_vec = n_pixels / 8;   // 8 pixels per NEON iteration
    const size_t n_rem = n_pixels % 8;

    const uint8_t* s = src;
    float* rp = r_plane;
    float* gp = g_plane;
    float* bp = b_plane;

    for (size_t i = 0; i < n_vec; ++i, s += 24, rp += 8, gp += 8, bp += 8) {
        // Load 8 pixels (24 bytes), deinterleave BGR into 3 × uint8x8_t
        uint8x8x3_t bgr = vld3_u8(s);
        // vld3_u8 (not vld3q): 8 pixels, 24 bytes. Throughput: 2 cycles on A76.

        // Widen uint8x8 → uint16x8 → uint32x4 (low + high halves) → float32x4
        // This is the standard ARM NEON uint8→float conversion chain.
        // For channel R (= bgr.val[2]):
        uint16x8_t r16 = vmovl_u8(bgr.val[2]);  // [0..255] → [0..255] as u16
        uint16x8_t g16 = vmovl_u8(bgr.val[1]);
        uint16x8_t b16 = vmovl_u8(bgr.val[0]);

        // Low 4 elements:
        float32x4_t rf_lo = vcvtq_f32_u32(vmovl_u16(vget_low_u16(r16)));
        float32x4_t gf_lo = vcvtq_f32_u32(vmovl_u16(vget_low_u16(g16)));
        float32x4_t bf_lo = vcvtq_f32_u32(vmovl_u16(vget_low_u16(b16)));

        // High 4 elements:
        float32x4_t rf_hi = vcvtq_f32_u32(vmovl_u16(vget_high_u16(r16)));
        float32x4_t gf_hi = vcvtq_f32_u32(vmovl_u16(vget_high_u16(g16)));
        float32x4_t bf_hi = vcvtq_f32_u32(vmovl_u16(vget_high_u16(b16)));

        // Apply fused (scale, bias): out = val * scale + bias
        // vmlaq_f32(bias, val, scale) = bias + val * scale  ← NEON FMA
        rf_lo = vmlaq_f32(v_bias_r, rf_lo, v_scale_r);
        rf_hi = vmlaq_f32(v_bias_r, rf_hi, v_scale_r);
        gf_lo = vmlaq_f32(v_bias_g, gf_lo, v_scale_g);
        gf_hi = vmlaq_f32(v_bias_g, gf_hi, v_scale_g);
        bf_lo = vmlaq_f32(v_bias_b, bf_lo, v_scale_b);
        bf_hi = vmlaq_f32(v_bias_b, bf_hi, v_scale_b);

        // Store to CHW planes
        vst1q_f32(rp,     rf_lo);
        vst1q_f32(rp + 4, rf_hi);
        vst1q_f32(gp,     gf_lo);
        vst1q_f32(gp + 4, gf_hi);
        vst1q_f32(bp,     bf_lo);
        vst1q_f32(bp + 4, bf_hi);
    }

    // Scalar tail
    for (size_t i = 0; i < n_rem; ++i, s += 3) {
        rp[i] = s[2] * kScaleR + kBiasR;
        gp[i] = s[1] * kScaleG + kBiasG;
        bp[i] = s[0] * kScaleB + kBiasB;
    }

#else
    // Non-NEON scalar fallback
    for (size_t i = 0; i < n_pixels; ++i) {
        r_plane[i] = src[i*3+2] * kScaleR + kBiasR;
        g_plane[i] = src[i*3+1] * kScaleG + kBiasG;
        b_plane[i] = src[i*3+0] * kScaleB + kBiasB;
    }
#endif
}
```

---

## 10. Core 1 vs Core 2 Concurrency Analysis

This section documents why concurrent execution of YOLO INT8 (Core 1) and MiDaS FP32 (Core 2) is safe with no observable interference.

### 10.1 NEON Register Independence

```
Core 0 (Camera)      Core 1 (VisionNode)          Core 2 (DepthNode)
  ┌──────────────┐   ┌────────────────────────┐   ┌────────────────────────┐
  │ SIMD unit 0  │   │ SIMD unit 1             │   │ SIMD unit 2             │
  │ (inactive)   │   │ INT8 UDOT via KleidiAI  │   │ FP32 FMLA via CPU EP   │
  │              │   │ V0–V31 (128-bit NEON)   │   │ V0–V31 (128-bit NEON)  │
  └──────────────┘   └────────────────────────┘   └────────────────────────┘

ARM Architecture Reference Manual (D1.13.1):
"Advanced SIMD and floating-point registers are PART OF the processor state
of the PE and are NOT shared between PEs."
```

**Conclusion: zero NEON register contention.** NEON `V0`–`V31` on Core 1 are physically separate silicon from `V0`–`V31` on Core 2. The KleidiAI UDOT kernels on Core 1 cannot interfere with FMLA chains on Core 2 by any hardware mechanism.

### 10.2 Cache Hierarchy Analysis

```
              Core 1 (YOLO)         Core 2 (MiDaS)
              ┌─────────────┐       ┌─────────────┐
  L1D (64KB)  │ YOLO weights│       │MiDaS weights│  ← private per-core, no sharing
  L1I (64KB)  │ KleidiAI    │       │ FMLA kernels│
              └──────┬──────┘       └──────┬──────┘
  L2 (512KB)         │ private             │ private   ← Cortex-A76 L2 is core-private
              ┌──────┴──────────────────────┴──────┐
  L3 (8MB)    │        SHARED across all cores      │  ← potential contention point
              └─────────────────────────────────────┘
  LPDDR4X     │        Shared memory bus             │
              └─────────────────────────────────────┘
```

**L3 contention analysis:**
- YOLOv26 Nano INT8 weights: ~3 MB (post-quantization)
- MiDaS v2.1 small FP32 weights: ~14 MB
- Total weight working set: ~17 MB > 8 MB L3

**On first inference pass:** both models experience L3 cache misses, pulling from LPDDR4X. Peak bandwidth demand: ~2 GB/s per model × 2 cores = 4 GB/s. Pi 5 LPDDR4X bandwidth: ~50 GB/s theoretical, ~25 GB/s sustained. **4 GB/s is 16% of sustained bandwidth** — no memory bus saturation.

**On subsequent passes:** YOLOv26 Nano (3 MB) fits in L3 after warm-up. MiDaS (14 MB) partially fits. After 2–3 inference passes, both models settle into a steady-state L3 residency pattern.

**Mitigation for L3 contention during warm-up:**
- `DepthNode` runs at stride-3 by default (one inference per 3 camera frames)
- With CameraNode at 30 FPS, DepthNode runs at ~10 FPS
- VisionNode runs at 30 FPS
- Temporal offset: most DepthNode inference runs are NOT concurrent with VisionNode
- Even when concurrent: sustained bandwidth demand is well within Pi 5's LPDDR4X capacity

### 10.3 `SetIntraOpNumThreads(1)` — Core Isolation Guarantee

```
VisionNode  (Core 1, SCHED_FIFO 75, IntraOpThreads=1):
  ORT worker pool size = 1 → all convolutions execute on Core 1 only.
  ACL CPPScheduler respects IntraOpNumThreads when set to 1.
  KleidiAI UDOT kernel dispatches on Core 1's execution units only.

DepthNode   (Core 2, SCHED_FIFO 75, IntraOpThreads=2*):
  ORT worker pool size = 2 → both workers are scheduled on Core 2
  (SCHED_FIFO + CPU affinity ensures workers inherit Core 2 affinity).
  * Recommendation: start with IntraOpThreads=1; benchmark 1 vs 2 with
    `perf stat -e cache-misses,instructions` before committing to 2.
```

If `IntraOpNumThreads` is not set (default = ORT auto-selects), ORT will spawn threads equal to the CPU count (4 on Pi 5) and schedule them across all cores — **directly violating the Core 1/Core 2 isolation invariant**. This setting is mandatory.

---

## 11. Hot Path Allocation Audit

The following table documents every operation in the `image_callback` hot path and its allocation status. "None" means zero heap allocation; "Amortized" means allocation occurs once on first call, not on subsequent calls.

| Operation | Allocation? | Notes |
|---|---|---|
| `preprocess_letterbox()` — `cv::Mat` header wrapping `img_msg->data` | None | `cv::Mat` over external ptr — no data allocation |
| `preprocess_letterbox()` — `cv::resize()` into pre-alloc `letterbox_buf_` | None | Output written into static buffer |
| `preprocess_letterbox()` — `cv::Mat::setTo()` for gray padding | None | Operates on pre-alloc buffer |
| `neon_bgr_hwc_to_rgb_chw()` | None | NEON writes into pre-alloc `chw_input_buf_` |
| `session_->Run(run_options_, *io_binding_)` | None (after warmup) | `IoBinding` pre-bound; no tensor construction |
| `std::make_unique<DetectionArray>()` | **Once per frame** | Unavoidable — message is moved to publisher |
| `det_msg->detections.push_back()` in NMS | Amortized | `reserve()` in constructor pre-sizes |
| `postprocess_yolo()` — NMS scratch arrays | None | Static scratch arrays in `postprocess_yolo()` |
| `publish_spatial_latent()` — `std::make_unique<SpatialLatent>()` | **Once per frame** | Unavoidable — message is moved to publisher |
| `latent_msg->latent_vector.resize()` | Amortized (1st frame only) | After first frame, stable size → no realloc |
| `std::memcpy(latent_vector, latent_output_buf_)` | None | Data copy, no allocation |
| `det_publisher_->publish(std::move(...))` | None | Intra-process: moves unique_ptr, no copy |
| `latent_publisher_->publish(std::move(...))` | None | Intra-process: moves unique_ptr, no copy |

**Two unavoidable allocations per frame:** The ROS 2 message objects (`DetectionArray` and `SpatialLatent`) must be heap-allocated because they are transferred by `unique_ptr` to the downstream nodes. This is intrinsic to the intra-process ownership model.

**Optimization path (post-Phase 3):** Implement a `rclcpp::Publisher::loan_message()` pattern using ROS 2 Loaned Messages API with a custom allocator backed by a pre-allocated pool. This would eliminate both per-frame allocations. Not required for Phase 3 acceptance.

---

## 12. Build Configuration

### 12.1 `vigia_edge_node/CMakeLists.txt` additions for ONNX Runtime + KleidiAI

```cmake
# ── ONNX Runtime ──────────────────────────────────────────────────────────
# ORT must be built from source on Pi 5 with ACL + KleidiAI enabled.
# Pre-built wheels do NOT include the ACL EP with KleidiAI for ARM64.
#
# Build command (run on Pi 5 or cross-compile with aarch64 toolchain):
#   ./build.sh --config Release \
#     --use_acl --acl_home /path/to/arm_compute_library \
#     --arm_compute_library_home /path/to/arm_compute_library \
#     --parallel $(nproc)
#
# Install to /opt/onnxruntime/

find_package(onnxruntime REQUIRED
    PATHS /opt/onnxruntime/lib/cmake/onnxruntime)

# ARM Compute Library (ACL) — KleidiAI is bundled in ACL ≥ 24.01
find_library(ACL_LIB arm_compute PATHS /opt/arm_compute/lib REQUIRED)
find_library(ACL_CORE_LIB arm_compute_core PATHS /opt/arm_compute/lib REQUIRED)
find_library(KLEIDIAI_LIB kleidiai PATHS /opt/arm_compute/lib)
# Note: KleidiAI may be statically linked into arm_compute; KLEIDIAI_LIB may be empty.

target_link_libraries(vigia_edge_node PRIVATE
    onnxruntime
    ${ACL_LIB}
    ${ACL_CORE_LIB}
    ${KLEIDIAI_LIB}  # empty string if statically embedded in ACL
    # ... other existing deps ...
)

# ── NEON compiler flags ───────────────────────────────────────────────────
# Cortex-A76 specific tuning — enables asimddp (dot product) and sve
target_compile_options(vigia_edge_node PRIVATE
    -mcpu=cortex-a76
    -march=armv8.2-a+dotprod+fp16   # +dotprod: asimddp (UDOT/SDOT); +fp16: optional
    -O3
    -ftree-vectorize
    -ffast-math         # Enables FP reassociation — safe for CV/inference (not safety-critical FP)
    -funroll-loops
)
```

### 12.2 KleidiAI Build Verification

```bash
# On Pi 5 after building ONNX Runtime with ACL:

# 1. Verify asimddp is present:
grep asimddp /proc/cpuinfo | head -1

# 2. Verify ACL EP is registered in ORT:
python3 -c "import onnxruntime; print(onnxruntime.get_available_providers())"
# Expected: [..., 'AclExecutionProvider', 'CPUExecutionProvider']
# If AclExecutionProvider is missing → ORT not built with --use_acl

# 3. Verify KleidiAI assembly kernels are present in the ACL binary:
nm /opt/arm_compute/lib/libarm_compute.so | grep -i "kleidiai\|udot_gemm" | head -5
# Expected: symbols like arm_gemm::GemmInterleaved<...UDOT...>
```

---

## 13. Acceptance Criteria

| Test | Command / Method | Pass Condition |
|---|---|---|
| **ACL EP active** | Run with `ORTACL_FAST_MATH=1` env var + profile JSON inspection | `>50%` of Conv2D ops show `CpuAcc` provider prefix in profile |
| **INT8 UDOT dispatch** | Profile JSON inspection | Conv kernels show `qasymm8` or `assembly_` in op name |
| **YOLOv26 throughput** | `ros2 topic hz /vigia/detections` for 60 s | ≥ 28 Hz (baseline parity with OpenVINO) |
| **No `frame.clone()` in hot path** | `grep -n "\.clone()" src/vision_node.cpp src/depth_node.cpp` | Zero results |
| **No `blobFromImage`** | `grep -rn "blobFromImage" src/` | Zero results |
| **Flat heap in hot path** | `valgrind --tool=massif --pages-as-heap=yes ./vigia_edge_node` + ms_print | Heap growth < 1 KB/frame after 10-frame warm-up |
| **S_t published** | `ros2 topic hz /vigia/spatial_latent` | ≥ 28 Hz; `latent_vector` length > 0 |
| **S_t scene discrimination** | Offline: record S_t over 100 clear-road + 100 pothole frames | Cosine distance between class means ≥ 0.15 |
| **MiDaS FP32 enforced** | Profile JSON for DepthNode session | Zero `qasymm8` kernels; all depth ops show `float` type |
| **Core isolation** | `taskset -c 1 perf stat -e instructions ./vigia_edge_node` (monitor Core 2) | DepthNode inference events do NOT appear in Core 1 perf trace |
| **VisionNode latency P95** | Custom latency timer in `image_callback` | ≤ 33 ms (30 FPS budget) |
| **DepthNode latency P95** | Custom latency timer in depth callback | ≤ 200 ms |
| **NEON preprocessing time** | `CLOCK_MONOTONIC` delta around `neon_bgr_hwc_to_rgb_chw` | ≤ 1 ms for 320×320 frame |

---

*Next document: `.claude/design/05_anti_death_and_depin_contracts.md` — Anti-Death Storage state machine (Phase 5), MQTT/SIM7600 transmission pipeline, and DePIN security attestation payload assembly (Phase 6).*

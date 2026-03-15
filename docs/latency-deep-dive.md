# Latency Deep-Dive: src/ Files Referenced by system_visual_test

**Scope:** Every latency-relevant inefficiency found in `perception.cpp`, `analytical.cpp`, `coordinator.cpp`, `fusion.cpp`, `temporal.cpp` as called by `system_visual_test.cpp`.  
**Current baseline (Mac M2):** ~52ms avg latency, ~19 FPS. Pi 4 target: reduce from ~120ms (YOLO+MiDaS sequential) toward sub-100ms per processed frame.

---

## 1. `perception.cpp` — `preprocess()` allocates 3 intermediate `cv::Mat` objects every frame

**Severity: HIGH**

```cpp
cv::Mat rgb;
cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);   // alloc #1

cv::Mat resized;
cv::resize(rgb, resized, ...);                  // alloc #2

cv::Mat padded;
cv::copyMakeBorder(resized, padded, ...);       // alloc #3

// INT8 path: padded returned by value → another copy
return padded;
```

Three heap allocations per frame in the hottest path. On Pi 4, each `cv::Mat` allocation for a 320×320 image is ~300KB. The `cvtColor` → `resize` → `copyMakeBorder` chain also means the pixel data is read and written 3 times before it reaches the tensor.

**Fix:** Pre-allocate `rgb_`, `resized_`, `padded_` as persistent `cv::Mat` members. Use `cv::resize` with `dst` parameter to write directly into the pre-allocated buffer. For the INT8 path, write directly into the `inputTensor_` data pointer via a `cv::Mat` header wrapping it — eliminating the `padded` → tensor memcpy entirely.

```cpp
// In header: add persistent preprocessing buffers
cv::Mat preprocRgb_;    // 320×320 U8C3
cv::Mat preprocResized_; // new_unpad_w × new_unpad_h U8C3
cv::Mat preprocPadded_;  // 320×320 U8C3, wraps inputTensor_ data for INT8

// In loadNetwork(), after inputTensor_ is allocated (INT8 path):
preprocPadded_ = cv::Mat(inputHeight_, inputWidth_, CV_8UC3,
                          inputTensor_.data<uint8_t>());
// Now copyMakeBorder writes directly into the tensor — zero copy.
```

**Expected gain:** Eliminates ~3 allocations + 1 memcpy per frame. ~2–4ms on Pi 4.

---

## 2. `perception.cpp` — `preprocess()` does `cvtColor` then `resize` — wrong order

**Severity: MEDIUM**

```cpp
cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);  // operates on full-res frame (e.g. 1280×720)
cv::resize(rgb, resized, cv::Size(new_unpad_w, new_unpad_h));  // then downscales
```

`cvtColor` runs on the full-resolution input (1280×720 = 921,600 pixels). If you resize first, `cvtColor` only processes ~102,400 pixels (320×320) — a 9× reduction in work.

**Fix:** Swap the order: resize first, then cvtColor.

```cpp
cv::resize(frame, preprocResized_, cv::Size(new_unpad_w, new_unpad_h));
cv::cvtColor(preprocResized_, preprocRgb_, cv::COLOR_BGR2RGB);
cv::copyMakeBorder(preprocRgb_, preprocPadded_, ...);
```

**Expected gain:** ~3–5ms on Pi 4 for 1280×720 input.

---

## 3. `analytical.cpp` — `runInference()` allocates `resized` and `inputBlob` every frame

**Severity: HIGH**

```cpp
cv::Mat resized;
cv::resize(frame, resized, cv::Size(inputWidth_, inputHeight_));  // alloc

cv::Mat inputBlob;
resized.convertTo(inputBlob, CV_32F, 1.0f / 255.0f);  // alloc
```

Two allocations per MiDaS invocation. `inputBlob` is a 256×256×3 float32 Mat = 786KB. On Pi 4 with SD-card memory pressure, this matters.

**Fix:** Pre-allocate both as persistent members, same pattern as perception fix:

```cpp
// In header:
cv::Mat midasResized_;   // 256×256 U8C3
cv::Mat midasBlob_;      // 256×256 CV_32FC3

// In loadNetwork():
midasResized_.create(inputHeight_, inputWidth_, CV_8UC3);
midasBlob_.create(inputHeight_, inputWidth_, CV_32FC3);

// In runInference():
cv::resize(frame, midasResized_, cv::Size(inputWidth_, inputHeight_));
midasResized_.convertTo(midasBlob_, CV_32F, 1.0f / 255.0f);
```

**Expected gain:** Eliminates 2 allocations per MiDaS call. ~1–2ms.

---

## 4. `analytical.cpp` — `runInference()` allocates a new `cv::Mat depthMap` and `memcpy`s into it every frame

**Severity: MEDIUM**

```cpp
cv::Mat depthMap(
    static_cast<int>(inputHeight_),
    static_cast<int>(inputWidth_),
    CV_32F
);
std::memcpy(depthMap.data, depthData, inputWidth_ * inputHeight_ * sizeof(float));
return depthMap;
```

Allocates 256×256×4 = 262KB and copies the entire depth output every frame. The output tensor already owns this memory — we're copying it unnecessarily.

**Fix:** Wrap the output tensor data directly in a `cv::Mat` header (zero-copy). The tensor lifetime is valid until the next `infer()` call, which is after the caller has finished using the depth map.

```cpp
// Zero-copy: wrap tensor data directly
const ov::Tensor output = inferRequest_.get_tensor(outputTensor_);
const float* depthData = output.data<float>();
// Return a Mat that references the tensor buffer — no copy
cv::Mat depthMap(static_cast<int>(inputHeight_),
                 static_cast<int>(inputWidth_),
                 CV_32F,
                 const_cast<float*>(depthData));
return depthMap.clone();  // only clone if caller needs it to outlive next infer()
```

Or better: add a persistent `cv::Mat midasOutput_` member that wraps the tensor, and return a shallow copy. The coordinator calls `analytical_.runInference()` and immediately uses the result before the next MiDaS call, so the tensor data is stable.

**Expected gain:** Eliminates 262KB allocation + memcpy per MiDaS frame. ~0.5–1ms.

---

## 5. `analytical.cpp` — `extractDepthROI()` clones the ROI unnecessarily

**Severity: MEDIUM**

```cpp
cv::Mat roiDepth = depthMap(boundedROI).clone();  // ← deep copy
cv::GaussianBlur(roiDepth, roiDepth, cv::Size(3, 3), 0);
cv::normalize(roiDepth, roiDepth, 0.0f, 1.0f, cv::NORM_MINMAX);
```

The `.clone()` is needed because `GaussianBlur` and `normalize` operate in-place. But the ROI is small (proportional to the bounding box, typically 20–80px). The real cost is the function call overhead and the fact that this runs **once per detection per MiDaS frame**.

With the zero-copy depth map from fix #4, the clone is still needed (can't blur in-place on a tensor-backed Mat). But we can pre-allocate a reusable ROI buffer:

```cpp
// In header:
cv::Mat roiScratch_;  // reused across calls, resized as needed

// In extractDepthROI():
const cv::Rect boundedROI = clampROIToMat(roi, depthMap);
if (boundedROI.empty()) return {};
depthMap(boundedROI).copyTo(roiScratch_);  // reuses allocation if same size
cv::GaussianBlur(roiScratch_, roiScratch_, cv::Size(3, 3), 0);
cv::normalize(roiScratch_, roiScratch_, 0.0f, 1.0f, cv::NORM_MINMAX);
return roiScratch_;  // return by value (shallow copy of header, shared data)
```

**Expected gain:** Eliminates per-detection ROI allocation. Minor on its own, significant with many detections.

---

## 6. `coordinator.cpp` — YOLO and MiDaS run **sequentially** on the same thread

**Severity: CRITICAL — this is the dominant latency source**

```cpp
// processLoop() — single thread, Core 1:
auto detections = perception_.runInference(frame);   // ~83ms

const bool runMidas = (currentIdx % midasStride_ == 0);
if (runMidas) {
    depthMap = analytical_.runInference(frame);       // ~525ms
}
// Total sequential: 83 + 525 = 608ms when MiDaS runs
```

Despite the README describing a "4-stage parallel pipeline", YOLO and MiDaS are actually running **sequentially on Core 1** in `processLoop()`. The README's 10.3 FPS EMA is achieved because MiDaS runs at stride 5 (only 1 in 5 frames), not because they're truly parallel.

When MiDaS does run (stride=1 at cool temps), the frame latency is 608ms — that's the P95 latency of 608.8ms in the README benchmarks.

**Fix:** Run MiDaS on a dedicated thread (Core 2) with its own frame queue. YOLO posts detections immediately; MiDaS processes asynchronously and posts depth results when ready. The `InstrumentationBus` already handles out-of-order arrival via `recordDepth()` and the 100ms timeout in `tryPopFrame()`.

```cpp
// coordinator.hpp — add:
std::thread midasThread_;
SafeQueue<std::pair<std::uint64_t, cv::Mat>> midasQueue_;  // {frameIdx, frame}

// coordinator.cpp — midasLoop() on Core 2:
void Coordinator::midasLoop() {
    pinToCore(2);
    while (running_) {
        auto item = midasQueue_.wait_and_pop();  // blocks until frame available
        if (!item) break;
        auto [frameIdx, frame] = std::move(*item);
        analytical_.runInference(frame);  // posts to bus via InstrumentedAnalyticalAgent
    }
}

// processLoop() — YOLO only, posts to midasQueue_ instead of calling directly:
auto detections = perception_.runInference(frame);
if (currentIdx % midasStride_ == 0)
    midasQueue_.push({currentIdx, frame});  // non-blocking
```

**Expected gain:** YOLO latency becomes the frame latency (83ms) even on MiDaS frames. Eliminates the 608ms P95 spike entirely. This is the single biggest latency improvement available.

---

## 7. `coordinator.cpp` — `processFrame()` reads a shallow copy of the frame buffer under lock, but MiDaS then uses the same `frame` Mat

**Severity: LOW-MEDIUM**

```cpp
{
    std::lock_guard<std::mutex> lock(bufferMutex_);
    frame = frameBuffer_[(frameIndex_ - 1) % FRAME_BUFFER_SIZE];  // shallow copy
    currentIdx = frameIndex_;
}
// frame is a shallow copy — shares pixel data with frameBuffer_
// captureLoop() may overwrite frameBuffer_[same slot] while MiDaS is running
```

With fix #6 (MiDaS on separate thread), `frame` is passed to `midasQueue_` and used asynchronously. If `captureLoop` wraps around the ring buffer (size 4) and overwrites the slot before MiDaS finishes, MiDaS reads corrupted data.

**Fix:** When pushing to `midasQueue_`, push a `.clone()` — or increase `FRAME_BUFFER_SIZE` to 8 to ensure the slot isn't overwritten within MiDaS's ~525ms window. At 15 FPS capture, 8 slots = 533ms of buffer — just enough.

---

## 8. `perception.cpp` — `postprocess()` allocates `boxes`, `scores`, `classIds` vectors every frame

**Severity: LOW**

```cpp
std::vector<cv::Rect> boxes;
std::vector<float> scores;
std::vector<int> classIds;
boxes.reserve(128);
scores.reserve(128);
classIds.reserve(128);
```

`reserve(128)` avoids reallocation during fill, but the initial allocation still happens every frame. These are small (~3KB total) but it's 3 heap allocations in the postprocessing hot path.

**Fix:** Make them persistent members, call `.clear()` at the start of `postprocess()` instead of constructing new vectors.

---

## 9. `temporal.cpp` — `std::deque` for history has poor cache locality

**Severity: LOW**

```cpp
std::deque<float> depressionHistory_;
std::deque<float> roughnessHistory_;
```

`std::deque` uses segmented memory — iterating it for `computePersistence()` and `computeStability()` causes cache misses on every segment boundary. With `historySize_=10`, this is a 10-element deque.

**Fix:** Replace with a fixed-size circular buffer (`std::array<float, 10>` + head index). Contiguous memory, no heap allocation, better cache behavior.

```cpp
// In header:
static constexpr std::size_t kMaxHistory = 10;
std::array<float, kMaxHistory> depressionBuf_{};
std::array<float, kMaxHistory> roughnessBuf_{};
std::size_t head_{0};
std::size_t count_{0};
```

---

## 10. `perception.cpp` — INT8 path: `preprocess()` returns `padded` by value, then the caller does a NEON memcpy into the tensor

**Severity: MEDIUM**

```cpp
// preprocess() returns padded (320×320 U8C3 = 307KB) by value
cv::Mat blob = preprocess(frame, lb);

// runInference() then copies it into the tensor:
uint8_t* tensorData = inputTensor_.data<uint8_t>();
const uint8_t* blobData = blob.ptr<uint8_t>();
// NEON memcpy of 307KB
```

With fix #1 (wrapping `inputTensor_` data as `preprocPadded_`), `copyMakeBorder` writes directly into the tensor — this entire copy disappears.

---

## Priority Order for Implementation

| # | File | Fix | Latency Impact |
|---|---|---|---|
| 1 | `coordinator.cpp` | MiDaS on dedicated thread (Core 2) | **CRITICAL** — eliminates 608ms P95 |
| 2 | `perception.cpp` | Swap resize/cvtColor order | HIGH — ~3–5ms/frame |
| 3 | `perception.cpp` | Pre-allocate preprocessing Mats, zero-copy INT8 tensor write | HIGH — ~2–4ms/frame |
| 4 | `analytical.cpp` | Pre-allocate `midasResized_` + `midasBlob_` | MEDIUM — ~1–2ms/frame |
| 5 | `analytical.cpp` | Zero-copy depth output (wrap tensor as Mat) | MEDIUM — ~0.5–1ms/frame |
| 6 | `analytical.cpp` | Pre-allocate `roiScratch_` in `extractDepthROI` | LOW-MEDIUM |
| 7 | `coordinator.cpp` | Increase `FRAME_BUFFER_SIZE` to 8 for async MiDaS safety | Required for fix #1 |
| 8 | `perception.cpp` | Persistent `boxes/scores/classIds` vectors | LOW |
| 9 | `temporal.cpp` | `std::deque` → circular `std::array` | LOW |

Fix #1 alone (true YOLO/MiDaS parallelism) reduces the worst-case frame latency from 608ms to ~83ms — matching YOLO's own P50. Fixes #2–#5 together save another ~8–12ms per frame on Pi 4.

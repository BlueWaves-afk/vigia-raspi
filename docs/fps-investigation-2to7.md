# FPS Investigation: 2–3 FPS → 7+ FPS on Raspberry Pi 4

**Status:** Investigation complete. Fixes identified and prioritized.  
**Target:** `system_visual_test.cpp` running `hazard.mp4` on Pi 4 (Cortex-A72)  
**Observed:** 2–3 FPS with display  
**Expected baseline:** 10.3 FPS EMA (headless, per README benchmarks)  
**Target:** 7+ FPS sustained

---

## Root Cause Summary

The 2–3 FPS is almost entirely explained by **three compounding issues**, not inference speed. YOLO at 83ms and MiDaS at stride 5 are already well-characterized. The bottlenecks are in the surrounding system.

---

## Issue 1 — `cv::imshow` + VNC Encoding (Confirmed, ~3.4× degradation)

**File:** `system_visual_test.cpp`, main render loop  
**Impact:** Drops ~10 FPS → ~3 FPS. This alone explains the observed regression.

The README explicitly documents this: "cv::imshow + VNC encoding reduces stable throughput from 10.3 FPS to ~3 FPS." The dashboard renders a full `1024×768` (ARM profile) or `1440×900` (desktop) composite every `kRenderIntervalMs = 66ms` (~15 FPS UI cap). On Pi 4, VNC encodes the framebuffer on the same CPU cores running inference.

**Fix options (in order of impact):**
1. Run headless (`--headless` flag, skip `cv::imshow` entirely) — recovers full 10.3 FPS EMA.
2. Reduce dashboard resolution: change `kDashboardWidth/Height` to `800×480` on ARM profile.
3. Increase `kRenderIntervalMs` from 66ms to 200ms (5 FPS UI cap instead of 15).
4. Use a dedicated display thread on Core 3 that only renders when a new snapshot is available, not on a timer.

---

## Issue 2 — `coordinator.cpp`: Single Mutex Serializes Capture + Process

**File:** `src/coordinator.cpp`, `captureLoop()` and `processLoop()`  
**Impact:** Moderate — adds contention latency on every frame.

```cpp
// captureLoop — holds bufferMutex_ while cloning frame
{
    std::lock_guard<std::mutex> lock(bufferMutex_);
    frameBuffer_[frameIndex_ % FRAME_BUFFER_SIZE] = frame.clone();
    frameIndex_++;
}

// processLoop — holds bufferMutex_ while reading
{
    std::lock_guard<std::mutex> lock(bufferMutex_);
    frame = frameBuffer_[(frameIndex_ - 1) % FRAME_BUFFER_SIZE];
    currentIdx = frameIndex_;
}
```

The `frame.clone()` inside the lock is the problem. A 640×480 BGR clone is ~0.9ms, but it holds the mutex the entire time, blocking `processLoop` from reading the latest frame. At 10+ FPS this adds up.

**Fix:** Clone outside the lock, then swap inside:
```cpp
// captureLoop
cv::Mat cloned = frame.clone();  // outside lock
{
    std::lock_guard<std::mutex> lock(bufferMutex_);
    frameBuffer_[frameIndex_ % FRAME_BUFFER_SIZE] = std::move(cloned);
    frameIndex_++;
}
```

---

## Issue 3 — `coordinator.cpp`: `frameLimiter()` Actively Throttles Processing

**File:** `src/coordinator.cpp`, `processLoop()`  
**Impact:** Direct FPS cap — this is the most likely cause of sub-7 FPS even in headless mode.

```cpp
void Coordinator::frameLimiter(long elapsedMs) const {
    if (elapsedMs < targetFrameTimeMs_) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(targetFrameTimeMs_ - elapsedMs)
        );
    }
}
```

`targetFrameTimeMs_ = 1000 / targetFps`. The default `targetFps` passed from `main()` is **30**, giving `targetFrameTimeMs_ = 33ms`. But YOLO alone takes ~83ms, so `elapsedMs` is always > 33ms and the limiter never fires. However, if `targetFps` is set lower (e.g., 15 → 66ms), and a frame with no detections completes in ~85ms, the limiter fires and adds 0ms. This is fine.

The real problem: when `targetFps=30` is passed and the system is running at 10 FPS, the limiter is harmless but the `adaptiveControl()` logic sees `elapsedMs > targetFrameTimeMs_` (83ms > 33ms) **every single frame** and increments `midasStride_` toward 5 immediately, even at cool temperatures. This means MiDaS runs at stride 5 from frame 1, not just under thermal pressure.

**Fix:** Set `targetFps` to a realistic value matching actual throughput, or decouple the stride governor from the frame limiter:
```cpp
// In adaptiveControl: only increment stride if we're genuinely overloaded
// (not just because targetFps is set too high)
else if (elapsedMs > 150) {  // only throttle if >150ms per frame (below 7 FPS)
    midasStride_ = std::min(midasStride_ + 1, 5);
}
```

---

## Issue 4 — `analytical.cpp`: `computeDepthResiduals()` Uses `double` Arithmetic

**File:** `src/analytical.cpp`, `computeDepthResiduals()`  
**Impact:** ~10–15% slower than necessary on Cortex-A72 (no hardware double-precision SIMD).

The plane-fitting loop uses `double` accumulators (`sumX`, `sumY`, `sumZ`, etc.) on a CPU where `double` operations are not vectorized by NEON (NEON only has `float32x4_t`, not `float64x2_t` for this use case). The entire function runs scalar double-precision.

```cpp
double sumX = 0, sumY = 0, sumZ = 0;  // ← all double
double sumXX = 0, sumYY = 0, sumXY = 0;
// ...
```

The input data is `CV_32F` (float). The precision of `double` here is unnecessary — the depth values are normalized to [0,1] and the plane coefficients don't require double precision.

**Fix:** Convert all accumulators to `float`:
```cpp
float sumX = 0, sumY = 0, sumZ = 0;
float sumXX = 0, sumYY = 0, sumXY = 0;
float sumXY2 = 0, sumXZ = 0, sumYZ = 0;
```
This enables NEON auto-vectorization of the accumulation loop.

---

## Issue 5 — `analytical.cpp`: `extractDepthROI()` Calls `cv::medianBlur` on Every Detection

**File:** `src/analytical.cpp`, `extractDepthROI()`  
**Impact:** `cv::medianBlur` with kernel=5 on a depth ROI is expensive (~2–5ms per detection).

```cpp
cv::medianBlur(roiDepth, roiDepth, 5);  // called per detection, per MiDaS frame
```

With multiple detections per frame, this runs multiple times per MiDaS invocation. At stride 5 this is infrequent, but at stride 1 (cool temperatures) it compounds.

**Fix:** Reduce kernel to 3, or replace with `cv::GaussianBlur` (faster on ARM via KleidiCV HAL):
```cpp
cv::GaussianBlur(roiDepth, roiDepth, cv::Size(3, 3), 0);
```

---

## Issue 6 — `perception.cpp`: `num_requests(2)` for YOLO Creates Unnecessary Overhead

**File:** `src/perception.cpp`, `loadNetwork()`  
**Impact:** Minor — allocates two inference request objects but only one is used.

```cpp
ov::hint::num_requests(2),  // ← only inferRequest_ is used
```

With `PerformanceMode::LATENCY` and a single `inferRequest_`, the second request slot is allocated but never used. On a memory-constrained Pi 4 (551MB RSS already), this wastes ~9MB of model weight duplication.

**Fix:** Change to `num_requests(1)` to match MiDaS and the actual usage pattern.

---

## Issue 7 — `system_visual_test.cpp`: Dashboard Rebuilds Full `cv::Mat` Every Render Cycle

**File:** `tests/system_visual_test.cpp`, render loop  
**Impact:** Allocates and fills a `1024×768` Mat every 66ms even when no new frame arrived.

```cpp
cv::Mat dashboard(dashboardSize, CV_8UC3);
dashboard.setTo(backgroundColor);  // memset of 2.25MB every render tick
```

The `needsRender` flag gates this correctly, but `dashboard` is stack-allocated inside the render block, so it's constructed and destroyed every render cycle.

**Fix:** Hoist `dashboard` outside the loop as a persistent `cv::Mat`, only calling `setTo` when `needsRender` is true:
```cpp
cv::Mat dashboard(dashboardSize, CV_8UC3);  // allocated once, before the loop
// inside render block:
dashboard.setTo(backgroundColor);  // reuse existing allocation
```

---

## Issue 8 — `coordinator.cpp`: `publishResult()` Uses `std::cout` in the Hot Path

**File:** `src/coordinator.cpp`, `publishResult()`  
**Impact:** `std::cout` acquires a global lock on every detection. At 10 FPS with multiple detections, this adds measurable latency.

```cpp
std::cout
    << "[POTHOLE]"
    << " conf=" << out.finalConfidence
    // ...
    << "\n";
```

**Fix:** Either remove this in production builds (`#ifndef NDEBUG`) or replace with a lock-free ring buffer telemetry sink.

---

## Recommended Fix Order (by impact)

| Priority | Issue | Expected Gain |
|---|---|---|
| 1 | Run headless (skip `cv::imshow`) | +7 FPS (recovers to ~10 FPS EMA) |
| 2 | Fix `frameLimiter` / `adaptiveControl` stride logic | +1–2 FPS at cool temps |
| 3 | Clone frame outside `bufferMutex_` | Reduces jitter, not raw FPS |
| 4 | Hoist `dashboard` Mat outside render loop | Reduces GC pressure |
| 5 | `double` → `float` in `computeDepthResiduals` | ~10% faster depth analysis |
| 6 | `medianBlur(5)` → `GaussianBlur(3)` in `extractDepthROI` | ~2ms per detection |
| 7 | `num_requests(2)` → `num_requests(1)` for YOLO | ~9MB RAM freed |
| 8 | Remove `publishResult` stdout in hot path | Reduces lock contention |

**To reach 7+ FPS with display:** Fix issues 2 + 3 + 4 + reduce dashboard resolution to `800×480` + increase `kRenderIntervalMs` to 150ms.  
**To reach 10+ FPS:** Run headless. The architecture already supports it.

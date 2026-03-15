# Coding Standards — VIGIA-ARM

## Language & Standard
- C++17 strictly. No C++20 features (Pi OS compiler may not support them fully).
- No exceptions in hot paths (inference loop, queue operations).
- No heap allocation in the inference hot path — use pre-allocated tensors and buffers.

## ARM-Specific Rules
- NEON intrinsics (`arm_neon.h`) must be guarded with `#if defined(__aarch64__) || defined(__ARM_NEON)` and always have a scalar fallback.
- Prefer `vld3q_f32` / `vst1q_f32` for HWC→CHW transposition — already implemented in `perception.cpp` and `analytical.cpp`. Do not regress to scalar loops.
- All thread pinning (`pthread_setaffinity_np`) must be guarded with `#ifdef __linux__`.

## Threading Model
- Coordinator owns two threads: `captureThread_` (Core 0) and `mainThread_` (Core 1).
- UI/main thread is pinned to Core 3 in `system_visual_test.cpp`.
- Core 2 is reserved for OpenVINO/TBB worker threads — do not pin application threads to it.
- Inter-stage communication: `SafeQueue<T>` (lock-free try_pop) or `InstrumentationBus` (mutex, <1% contention). Do not add new mutexes in the inference hot path.

## OpenVINO Usage
- Always use a **shared `ov::Core`** instance — never construct per-agent cores. Duplicate cores waste 50–100ms on Pi 4 plugin discovery.
- `ov::enable_mmap(false)` must be set — SD-card mmap faults cause SIGBUS.
- `ov::hint::PerformanceMode::LATENCY` + `num_requests(1)` for MiDaS; `num_requests(2)` for YOLO.
- `inference_precision(ov::element::dynamic)` for YOLO (allows native INT8). `inference_precision(ov::element::f32)` for MiDaS.
- SIGBUS recovery via `sigsetjmp/siglongjmp` is required around `compile_model()` — do not remove.

## OpenCV Usage
- `cv::setNumThreads(0)` at startup — lets TBB use all cores for KleidiCV HAL dispatch.
- `cv::ocl::setUseOpenCL(false)` — no OpenCL on Pi 4.
- Prefer `cv::resize` directly into a dashboard sub-ROI (zero intermediate allocation) over resize-then-copy.
- `cv::parallel_for_` for per-detection rendering loops — dispatches to TBB automatically.

## Model Files
- YOLO26 INT8: `models/yolo26/yolo26_model_int8.xml` — production default.
- YOLO26 FP32: `models/yolo26/yolo26_model.xml` — validation only.
- MiDaS FP32: `models/midasv21/openvino_midas_v21_small_256.xml` — always FP32.
- **Never commit INT8 MiDaS** — dynamic range collapse failure is documented. See `docs/int8_midas_failure.md`.

## Build
- Always compile with `-mcpu=cortex-a72 -O3 -ftree-vectorize` (set in CMakeLists.txt).
- `make -j$(nproc)` on Pi 4 uses all 4 cores for compilation.
- CPU governor must be `performance` before running any benchmark: `echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor`

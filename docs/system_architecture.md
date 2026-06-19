# VIGIA — System Architecture

> [!IMPORTANT] Two architectures live in this repo
> - **Current production runtime** = the ROS 2 workspace (`vigia_ws/`) with **ONNX
>   Runtime** inference + the **AWS serverless** backend. Documented in Part A below
>   and in `.wiki/08_ROS2_Edge_Nodes`, `.wiki/09_Cloud_Pipeline`, `.wiki/10_Cloud_Security_Model`.
> - **Legacy monolith** = the original Bharat-AI-SoC hackathon pipeline in `src/`
>   (OpenVINO + `coordinator.cpp`). Preserved as **Part B** — it is no longer the
>   deployed runtime but remains a valid record of the edge-optimization work.

---

# Part A — Current Production Architecture (ROS 2 + ONNX + AWS)

## A.1 Edge runtime (`vigia_ws/src/vigia_edge_node`)

Multi-node ROS 2 graph; each node runs on a dedicated thread with an explicit
scheduling class (`launch_rt_node()` → `pthread_setschedparam(SCHED_FIFO, prio)`).

| Node | Inference / role | Sched prio |
|------|------------------|-----------|
| `camera_node` | CSI capture → `/dev/shm` frame ring + `ShmMetaRing` | FIFO 80 |
| `vision_node` | **YOLO26 INT8 via ONNX Runtime** (`Ort::IoBinding`); emits spatial latent `S_t` | FIFO 75 |
| `depth_node` | **MiDaS v2.1 via ONNX Runtime** | FIFO 75 |
| `fusion_node` | Gravity-compensated ISS, RRI, Kalman dead-reckoning → `/vigia/hazard_events` | FIFO 70 |
| `sensor_bridge_node` | COBS@921600 from Pico; IMU/GPS/`SignedEtPacketPi`; ECDSA verify | FIFO 85 |
| `anti_death_node` | UPS GPIO (libgpiod v2); seqlock snapshot; emergency uplink in 15 s | FIFO 99 |
| `ble_gatt_node` | sdbus-c++ v2 GATT; ECDH handshake; phone telemetry | OTHER 40 |
| `hazard_uplink_node` | MsgPack → MQTT QoS-1 mTLS → AWS IoT Core | OTHER 30 |

> [!NOTE]
> Inference is **ONNX Runtime**, not OpenVINO. `vision_node.cpp`/`depth_node.cpp`
> use `Ort::` APIs. The optional KleidiAI/ACL execution provider is built via
> `scripts/build_ort_acl.sh` (`onnxruntime_providers_acl.so`).

## A.2 Cloud backend (`vigia-amazon`, CDK `VigiaStack`)

Fully serverless since **M12** — no Docker, Mosquitto, FastAPI, or PostgreSQL.

```
Pi hazard_uplink ──MQTT mTLS──▶ IoT Core ──rule vigia_hazard_attest──▶ AttestationFn ─┐
Phone ──HTTPS Ed25519──▶ ValidatorFn ────────────────────────────────────────────────┴─▶ HazardsTable
                                                            (DynamoDB stream INSERT pipe) │
                                                                                          ▼
                                              OrchestratorFn  (2% Bedrock VLM / 98% ONNX fast path)
                                                                  │ VERIFIED
                                                  tryCreditReward (atomic) ─▶ RewardsLedger + Solana
```

- **AttestationFn** — MsgPack decode → EtHash SHA-256 → ECDSA P-256 verify → advance
  anti-replay watermark → H3 res-10 dedup → `HazardsTable`. See `.wiki/10_Cloud_Security_Model`.
- **OrchestratorFn** — `VLM_SAMPLE_RATE` (default 0.02) gates Bedrock Nova + ReAct Agent;
  rewards are atomic + deduped per (wallet, geohash)/30 days.
- DynamoDB: `HazardsTable`, `VigiaPiDeviceRegistry`, `VigiaDeviceBindings`,
  `RewardsLedger`, Ledger (`ContributorGeohashIndex`), Cooldown, `AttestationLog`.

Full detail: `.wiki/09_Cloud_Pipeline` and `.claude/design/` contracts.

---

# Part B — Legacy `src/` Monolith (hackathon system)

> Generated from source inspection of `src/`, `include/`, `tests/`, and `CMakeLists.txt`.
> All claims are grounded in actual code. This is the **original** OpenVINO pipeline,
> retained for reference; it is **not** the deployed production runtime (see Part A).

---

## 1. Build System & Dependencies

### CMakeLists.txt

- C++17, `CMAKE_CXX_EXTENSIONS OFF`
- Compile flags: `-mcpu=cortex-a72 -O3 -ftree-vectorize -Wall -Wextra`
- All agent sources compiled into a single static library `vigia_lib`
- Every `.cpp` in `tests/` gets its own executable, linked against `vigia_lib`
- Asset symlinks (`models/`, `hazard.mp4`) created post-build into `CMAKE_BINARY_DIR`

### Hard Dependencies (`cmake/Dependencies.cmake`)

| Library | Version | Components |
|---|---|---|
| OpenCV | ≥ 4.x | core, imgproc, imgcodecs, videoio, highgui |
| OpenVINO | 2025 | Runtime |
| Threads (POSIX) | — | `-pthread` via `Threads::Threads` |
| libdl | — | `-ldl` for OpenVINO plugin loading |
| TBB | optional | Parallel backend for OpenCV + KleidiCV HAL dispatch |

OpenVINO is expected at `/opt/intel/openvino_2025/`. TBB is sourced from OpenVINO's bundled `3rdparty/tbb`.

---

## 2. Source Layout

```
src/
  coordinator.cpp   — thread lifecycle, frame buffer, MiDaS queue, thermal control
  perception.cpp    — YOLO26 model load, preprocessing, inference, postprocessing
  analytical.cpp    — MiDaS model load, inference, ROI extraction, plane residuals
  fusion.cpp        — RRI computation (W_DET=0.40, W_GEO=0.35, W_TMP=0.25)
  temporal.cpp      — sliding window SNR (persistence = mean/stddev over 10 frames)
  main.cpp          — headless entry point, shared ov::Core, Coordinator start/stop

include/
  coordinator.hpp   — Coordinator class, MidasWork struct, SafeQueue<MidasWork>
  perception.hpp    — PerceptionAgent, Detection, FramePacket, PerceptionResult
  analytical.hpp    — AnalyticalAgent, DepthResidualStats, DepthGeometryMetrics
  fusion.hpp        — FusionEngine, FusionInput, FusionOutput
  temporal.hpp      — TemporalAnalyzer, TemporalMetrics (kHistorySize=10)
  safe_queue.hpp    — Lock-free ring buffer (try_pop / wait_and_pop / shutdown)
  roi_utils.hpp     — clampROIToMat helper

tests/
  system_visual_test.cpp      — Full pipeline + 5-panel dashboard (primary integration test)
  perception_video_test.cpp   — YOLO-only benchmark with UDP telemetry streaming
  system_sequential_test.cpp  — Sequential (non-parallel) pipeline validation
  performance_benchmark_test.cpp — Detailed latency/FPS benchmarking
  yolo_benchmark_test.cpp     — YOLO-isolated benchmark
  perception_test.cpp         — Unit: PerceptionAgent model load + single inference
  coordinator_test.cpp        — Unit: Coordinator lifecycle + thermal stride
  fusion_test.cpp             — Unit: RRI weight correctness
  temporal_test.cpp           — Unit: persistence/stability SNR computation
  analytical_test.cpp         — Unit: depth residuals + plane fitting
```

---

## 3. Thread Architecture & Core Pinning

The Coordinator spawns **three threads** and pins them via `pthread_setaffinity_np`. Pinning is guarded by `#ifdef __linux__` and only active on `aarch64`.

```
Thread              Core    Pinned?   Source
─────────────────────────────────────────────────────────────────────
captureThread_       0      YES       coordinator.cpp captureLoop()
mainThread_          1      YES       coordinator.cpp processLoop()
midasThread_         2      YES       coordinator.cpp midasLoop()
UI / main thread     3      NO*       system_visual_test.cpp
```

\* The UI thread pinning call (`pinCurrentThreadToCore(3)`) is **commented out** in `system_visual_test.cpp` with the explicit comment:

> "Thread pinning disabled — ACL CPPScheduler needs all cores available"

The same comment appears in `perception_video_test.cpp` (`pinToCore(1)` is commented out).

### What this means in practice

The Coordinator's three inference threads are hard-pinned to cores 0, 1, and 2. Core 3 is left free for the OpenVINO/ACL `CPPScheduler` worker threads to use as needed. The UI/main thread is not pinned and floats across whatever cores the OS scheduler assigns.

**OpenVINO threading config (set in `perception.cpp` and `analytical.cpp`):**

```cpp
// YOLO (PerceptionAgent)
ov::num_streams(1)
ov::inference_num_threads(4)   // all 4 cores available to OpenVINO's thread pool
ov::hint::PerformanceMode::LATENCY
ov::hint::num_requests(1)

// MiDaS (AnalyticalAgent)
ov::inference_num_threads(2)
ov::hint::PerformanceMode::LATENCY
ov::hint::num_requests(1)
```

So the actual execution model is: application threads are pinned to specific cores for deterministic scheduling, but OpenVINO's internal thread pool (`inference_num_threads(4)`) is allowed to use all cores for parallelizing convolution ops within a single inference call.

---

## 4. Pipeline Data Flow

```
VideoCapture / Camera
        │
        ▼
captureLoop (Core 0)
  └─ frame.clone() → frameBuffer_[frameIndex_ % 8]
        │
        ▼
processLoop (Core 1)
  ├─ perception_.runInference(frame)   ← YOLO26, every frame
  │     └─ InstrumentationBus::beginFrame + storeDetections
  │
  ├─ if (frameIndex_ % midasStride_ == 0):
  │     midasQueue_.push({frameIdx, frame.clone(), detections})
  │
  └─ for each pothole detection:
        FusionOutput (YOLO-only baseline) → publishResult()
        InstrumentationBus::notifyProcessingComplete()
        │
        ▼
midasLoop (Core 2)
  ├─ analytical_.runInference(frame)   ← MiDaS FP32
  │     └─ InstrumentationBus::recordDepth
  │
  └─ for each pothole detection:
        scaleROIToDepth → extractDepthROI → computeDepthResiduals
        → computeGeometryMetrics → temporal_.update()
        → fusion_.fuse()
              └─ InstrumentationBus::recordFusion
```

The `InstrumentationBus` (in `system_visual_test.cpp`) is a ring buffer of 8 `FrameSlot`s protected by a single `std::mutex`. It connects the processing pipeline to the UI thread. Slots time out after 100ms — bounding boxes are always drawn even if MiDaS/fusion didn't complete for that frame.

---

## 5. Inference Agents

### PerceptionAgent (YOLO26)

- Shared `ov::Core&` accepted in constructor — no duplicate plugin init
- `ov::enable_mmap(false)` — prevents SIGBUS from SD-card mmap faults
- `sigsetjmp/siglongjmp` around `compile_model()` — recovers from SIGBUS on ARM64 without ACL
- INT8 model auto-detected by scanning for `FakeQuantize` ops in the IR graph
- Preprocessing:
  - Letterbox resize (maintains aspect ratio, pads with 114)
  - BGR→RGB cvtColor on the small resized image (not the full-res frame)
  - INT8 path: `preprocPadded_` wraps `inputTensor_` data directly — `copyMakeBorder` writes straight into the tensor (zero-copy)
  - FP32 path: manual NEON `vld3q_f32` HWC→CHW deinterleave with scalar fallback
- Postprocessing: handles `[1,N,6]`, `[1,6,N]`, `[1,N,5]`, `[1,5,N]` output layouts; custom NMS (not OpenCV's) with IOU threshold 0.45
- Confidence thresholds: FP32 → 0.25, INT8 → 0.008

### AnalyticalAgent (MiDaS v2.1)

- Same shared `ov::Core&` pattern and SIGBUS recovery as PerceptionAgent
- Always FP32 (`ov::hint::inference_precision(ov::element::f32)`)
- `inference_num_threads(2)` — MiDaS gets 2 threads vs YOLO's 4
- Uses `inferRequest_.infer()` (synchronous) not `start_async()`
- NEON `vld3q_f32` HWC→CHW deinterleave with scalar fallback (same pattern as YOLO)
- Output tensor wrapped as a `cv::Mat` header (`depthOutput_`) — zero-copy read; `.clone()` returned to caller
- Plane residual fitting: least-squares 3-parameter plane fit (`ax + by + c`) over the ROI depth patch; `meanResidual`, `minResidual`, `stdResidual` computed in a single pass

### FusionEngine

Weights are compile-time constants:

```cpp
constexpr float W_DET = 0.40f;   // YOLO confidence
constexpr float W_GEO = 0.35f;   // geometry confidence
constexpr float W_TMP = 0.25f;   // temporal confidence
```

Geometry confidence: `clamp01(depression) * exp(-roughness * 10.0f)`  
Temporal confidence: `0.5 * tanh(persistence * 0.1) + 0.5 * tanh(stability * 0.01)`  
RRI threshold for hazard promotion: **0.55** in `system_visual_test.cpp` (`HAZARD_THRESHOLD`), not 0.75 as documented in README.

### TemporalAnalyzer

- Fixed-size circular buffer: `kHistorySize = 10` frames
- Persistence = `mean(depression) / (stddev(depression) + 1e-4)`
- Stability = `1.0 / (stddev(roughness) + 1e-4)`

---

## 6. Thermal-Adaptive Stride Governor

Implemented in `Coordinator::adaptiveControl()`. Temperature read from `/sys/class/thermal/thermal_zone0/temp`, throttled to 1Hz via `cachedTemp_`.

```
Condition                    midasStride_
─────────────────────────────────────────
temp > 85°C (CRITICAL)       5
temp > 75°C (WARM)           3
elapsedMs > 150ms            min(stride+1, 5)   ← FPS-based throttle
else                         max(1, stride-1)   ← recovery
```

YOLO runs every frame regardless of stride. MiDaS runs when `frameIndex_ % midasStride_ == 0`.

---

## 7. Frame Buffer

- Size: **8 slots** (`FRAME_BUFFER_SIZE = 8`) — sized to cover MiDaS's ~525ms window at 15 FPS
- Protected by `bufferMutex_` (`std::mutex`)
- `captureLoop` clones the frame outside the lock, then moves it into the buffer under the lock
- `processLoop` reads `frameBuffer_[(frameIndex_ - 1) % 8]` under the lock — always the latest frame

---

## 8. system_visual_test.cpp

The primary integration test and the only executable with a visual dashboard. It does not use `Coordinator` directly for the MiDaS/fusion path — instead it wraps all agents in instrumented subclasses that intercept every inference call and record results to `InstrumentationBus`.

**Class hierarchy:**

```
PerceptionAgent
  └─ VideoPerceptionAgent        — adds VideoCapture, pause/step, grab-only optimisation
       └─ InstrumentedPerceptionAgent  — intercepts runInference(), calls bus.beginFrame/storeDetections

AnalyticalAgent
  └─ InstrumentedAnalyticalAgent — intercepts runInference(), calls bus.recordDepth

TemporalAnalyzer
  └─ InstrumentedTemporalAnalyzer — intercepts update(), calls bus.recordTemporal

FusionEngine
  └─ InstrumentedFusionEngine    — intercepts fuse(), calls bus.recordFusion
```

**Key implementation details:**

- `VideoPerceptionAgent::captureFrame()` uses grab-only optimisation: `cap.grab()` advances the demuxer every call, but `cap.retrieve()` (full decode) only happens when `pendingDecode_` is false — i.e., when the inference thread has consumed the previous frame. This avoids decoding frames that will never be processed.
- Camera mode uses a GStreamer pipeline: `libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! video/x-raw, format=BGR ! appsink`
- `InstrumentationBus` ring buffer: 8 in-flight slots, 16 ready slots max. Slots finalize when all potholes are fused, or after 100ms timeout (unfused detections are promoted with raw YOLO confidence as fallback).
- Dashboard render is throttled to ~7 FPS (`kRenderIntervalMs = 150ms`) to reduce VNC encode overhead.
- Dashboard size: 800×480 on ARM64, 1440×900 on desktop.
- `cv::setNumThreads(0)` — lets TBB use all cores for KleidiCV HAL dispatch.
- `cv::ocl::setUseOpenCL(false)` — no OpenCL on Pi 4.
- `ov::enable_mmap(false)` set on the shared core before any model load.
- SIGBUS handler installed at startup; prints which step was in progress before crash.
- CPU governor checked at startup; warns if not `performance`.

**Display modes:**

| Mode | Toggle | Description |
|---|---|---|
| Dashboard | default | 5-panel: Detection Feed, Depth Map, Last Pothole Frame, Operational Insights, Event Log |
| Fullscreen | `-F` flag | Full-frame with per-detection info panels (YOLO conf, geometry, temporal, fusion) |
| Headless | `--headless` | No `cv::imshow`, maximum throughput |

**CLI:**
```
system_visual_test [-F] [--headless] [--fp32] (--video <path> | --cam [index]) [yolo_xml] [midas_xml] [target_fps]
```

---

## 9. perception_video_test.cpp

YOLO-only benchmark — no MiDaS, no temporal, no fusion. Used for isolated YOLO latency measurement and UDP telemetry streaming.

**What it does:**
- Loads YOLO26 via shared `ov::Core` (same setup as production)
- Runs `cap.grab()` + `cap.retrieve()` on every frame (no skip optimisation — this is a benchmark, not a production pipeline)
- Measures per-frame latency, computes EMA FPS (α=0.1), min/max/P95 latency
- Prints a formatted benchmark table at exit

**UDP streaming mode (`--stream <ip>`):**
- Non-blocking UDP socket (`O_NONBLOCK` + `SO_BROADCAST`)
- Annotated frame → resize to 640×360 → JPEG encode at quality 50 → send if < 65000 bytes
- Fire-and-forget: if UDP buffer is full, packet is dropped, inference continues uninterrupted

**Thread pinning:** `pinToCore(1)` is present but **commented out** — same reason as `system_visual_test.cpp` (ACL CPPScheduler needs all cores).

**OpenVINO config:**
```cpp
ov::enable_mmap(false)
ov::num_streams(1)
ov::inference_num_threads(4)
```

**CLI:**
```
perception_video_test [--headless] [--fp32] [--stream <mac-ip>] <video.mp4> [yolo_xml]
```

---

## 10. OpenVINO Model Configuration Summary

| Agent | Model | Precision | Input Layout | inference_num_threads | Async |
|---|---|---|---|---|---|
| PerceptionAgent | YOLO26 | FP16 (prod) / FP32 (dev) / INT8 (future) | NCHW (FP32) / NHWC (INT8) | 4 | `start_async()` + `wait()` |
| AnalyticalAgent | MiDaS v2.1 | FP32 (always) | NCHW | 2 | `infer()` (synchronous) |

Both agents:
- Share a single `ov::Core` instance
- Set `ov::enable_mmap(false)`
- Use `ov::hint::PerformanceMode::LATENCY` + `num_requests(1)`
- Wrap `compile_model()` in `sigsetjmp/siglongjmp` SIGBUS recovery

---

## 11. Known Discrepancies Between Code and README

| Claim in README/steering | Actual code |
|---|---|
| RRI threshold = 0.75 | `HAZARD_THRESHOLD = 0.55f` in `system_visual_test.cpp` |
| Core 3 for Fusion + UI | UI thread is NOT pinned; Core 3 is free for ACL workers |
| "4-stage parallel pipeline" | 3 Coordinator threads (capture/process/midas) + floating UI thread |
| FRAME_BUFFER_SIZE = 4 | Actual value is **8** in `coordinator.hpp` |
| MiDaS uses `start_async()` | MiDaS uses synchronous `inferRequest_.infer()` |
| INT8 YOLO as production default | `system_visual_test.cpp` defaults to `yolo26_320_fp16.xml` (FP16) |

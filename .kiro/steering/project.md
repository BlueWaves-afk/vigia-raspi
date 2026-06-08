# VIGIA-ARM — Project Steering

## What This Project Is
A 4-stage parallel AI perception pipeline for real-time road hazard detection on a Raspberry Pi 5 (ARM Cortex-A76, CPU-only, no GPU, no cloud). It fuses YOLO26 (INT8/KleidiAI) object detection with MiDaS v2.1 monocular depth estimation into a single Road Risk Index (RRI) score per detected hazard.

## Target Hardware
- **Board:** Raspberry Pi 5 (4 GB min, 8 GB recommended)
- **CPU:** ARMv8.2-A aarch64, Cortex-A76, 4 cores @ 2.4 GHz (`FEAT_DotProd`)
- **OS:** Raspberry Pi OS Lite 64-bit (Trixie / Debian 13)
- **No GPU. No cloud. No external accelerator** (Hailo NPU planned for next stage).

## Stack
- **Inference:** OpenVINO 2025.4.2 ARM CPU Plugin + KleidiAI INT8
- **Vision:** OpenCV 4.x with KleidiCV 26.03 HAL + TBB (GTK off)
- **Language:** C++17
- **Build:** CMake 3.16+, `-mcpu=cortex-a76`

## Pipeline Architecture
```
Core 0: Coordinator (frame dispatch, thermal monitoring, adaptive stride)
Core 1: Perception  (YOLO26 INT8 — ~28ms/frame via KleidiAI)
Core 2: Depth       (MiDaS v2.1 FP32 — stride-adaptive)
Core 3: Fusion + UI (RRI computation, temporal filtering, dashboard)
```
Stages communicate via lock-free ring buffer queues (`safe_queue.hpp`). Zero heap allocation in the inference hot path.

## Key Design Decisions
- **INT8 YOLO, FP32 MiDaS:** INT8 MiDaS collapses depth dynamic range — experimentally validated failure. Do not attempt INT8 MiDaS without a new calibration strategy.
- **MiDaS stride governor:** Three-tier thermal-adaptive stride (1/3/5) keeps SoC below 80°C throttle threshold. YOLO always runs every frame.
- **RRI threshold = 0.75:** Tri-factor fused score (semantic + geometric + temporal). Detections below this are suppressed.
- **CPU governor must be `performance`:** Frequency scaling causes unpredictable latency spikes. Lock it at startup.
- **Headless for production:** `cv::imshow` + VNC drops throughput from ~11 FPS to ~3 FPS. Dashboard is dev/validation only.
- **OpenCV built without GTK:** Skip `cv::namedWindow` when `--headless`; do not call `destroyAllWindows` in headless mode.

## Performance Baselines (Pi 5, hardware-verified)
| Metric | Value |
|---|---|
| Full pipeline FPS (headless) | 11.4 FPS |
| YOLO26 INT8 avg latency (KleidiAI) | 28.4 ms |
| YOLO-only throughput | 32.4 FPS |
| End-to-end latency range | 52 ms (YOLO-only) → 139 ms (YOLO+MiDaS) |
| MiDaS FP32 avg latency | ~400–525 ms (stride-adaptive) |

## Legacy Pi 4 Baselines (reference only)
| Metric | Value |
|---|---|
| Full pipeline FPS (headless EMA) | 10.3 FPS |
| YOLO26 FP16 avg latency (ACL) | 83.4 ms |

## Source Layout
```
src/           — coordinator, perception, analytical, fusion, temporal, main
include/       — all headers + safe_queue.hpp, roi_utils.hpp
tests/         — system_visual_test.cpp (primary integration test), unit tests
models/        — yolo26/ (FP32 + INT8 IR), midasv21/ (FP32 IR)
cmake/         — Dependencies.cmake
.kiro/steering — project steering docs (this folder)
docs/          — fix/feature/upgrade documentation
```

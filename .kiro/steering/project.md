# VIGIA-ARM — Project Steering

## What This Project Is
A 4-stage parallel AI perception pipeline for real-time road hazard detection on a Raspberry Pi 4 (ARM Cortex-A72, CPU-only, no GPU, no cloud). It fuses YOLO26 (INT8) object detection with MiDaS v2.1 monocular depth estimation into a single Road Risk Index (RRI) score per detected hazard.

## Target Hardware
- **Board:** Raspberry Pi 4B (2GB min, 8GB recommended)
- **CPU:** ARMv8-A aarch64, Cortex-A72, 4 cores @ 1.5 GHz
- **OS:** Raspberry Pi OS Lite 64-bit (Bookworm / Debian 12)
- **No GPU. No cloud. No external accelerator.**

## Stack
- **Inference:** OpenVINO 2025 ARM CPU Plugin + KleidiAI + ACL
- **Vision:** OpenCV 4.14 with KleidiCV 0.7.0 HAL + TBB
- **Language:** C++17
- **Build:** CMake 3.16+

## Pipeline Architecture
```
Core 0: Coordinator (frame dispatch, thermal monitoring, adaptive stride)
Core 1: Perception  (YOLO26 INT8 — ~83ms/frame)
Core 2: Depth       (MiDaS v2.1 FP32 — ~525ms/frame, stride-adaptive)
Core 3: Fusion + UI (RRI computation, temporal filtering, dashboard)
```
Stages communicate via lock-free ring buffer queues (`safe_queue.hpp`). Zero heap allocation in the inference hot path.

## Key Design Decisions
- **INT8 YOLO, FP32 MiDaS:** INT8 MiDaS collapses depth dynamic range — experimentally validated failure. Do not attempt INT8 MiDaS without a new calibration strategy.
- **MiDaS stride governor:** Three-tier thermal-adaptive stride (1/3/5) keeps SoC below 80°C throttle threshold. YOLO always runs every frame.
- **RRI threshold = 0.75:** Tri-factor fused score (semantic + geometric + temporal). Detections below this are suppressed.
- **CPU governor must be `performance`:** Frequency scaling causes unpredictable latency spikes. Lock it at startup.
- **Headless for production:** `cv::imshow` + VNC drops throughput from ~10 FPS to ~3 FPS. Dashboard is dev/validation only.

## Performance Baselines (Pi 4, hardware-verified)
| Metric | Value |
|---|---|
| Stable EMA FPS (headless) | 10.3 FPS |
| YOLO26 INT8 avg latency | 83.4 ms |
| MiDaS FP32 avg latency | 524.8 ms |
| Memory RSS | 551.5 MB |
| Power draw | ~3.0 W |
| CPU temp (sustained) | 41.9–47.2°C |

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

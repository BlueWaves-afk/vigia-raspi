<div align="center">

#  VIGIA-ARM

**ARM-Based Real-Time Road Hazard Detection System**

A multimodal, event-driven perception system with geometry-aware and temporally consistent fusion, optimized for edge ARM devices.

[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue?style=flat-square&logo=cplusplus)](https://en.cppreference.com/w/cpp/17)
[![Platform](https://img.shields.io/badge/Platform-ARM_Cortex--A72-orange?style=flat-square&logo=arm)](https://www.arm.com/)
[![Inference](https://img.shields.io/badge/Inference-OpenVINO_2025-purple?style=flat-square&logo=intel)](https://docs.openvino.ai/)
[![Vision](https://img.shields.io/badge/Vision-OpenCV_4.14-green?style=flat-square&logo=opencv)](https://opencv.org/)
[![License](https://img.shields.io/badge/License-MIT-brightgreen?style=flat-square)](LICENSE)

</div>

---

## Project Overview

VIGIA is a real-time, ARM-optimized road hazard detection system designed for edge deployment on a Raspberry Pi 4 (ARM Cortex-A72). The system performs on-device perception, monocular depth analysis, and risk fusion using a CPU-only stack powered by OpenVINO and OpenCV — without relying on GPUs, cloud inference, or external accelerators.

The project demonstrates a **hardware–software co-design** approach that maximizes performance on constrained ARM edge hardware while maintaining deterministic real-time behavior.

---
![VigiaSense MultiModal System.](vigia_700p_final.gif)
## Table of Contents

- [Objective](#objective)
- [System Architecture](#system-architecture)
- [Pipeline Stages](#pipeline-stages)
- [ARM Optimization Strategy](#arm-optimization-strategy)
- [Target Platform](#target-platform)
- [Project Structure](#project-structure)
- [Design Philosophy](#design-philosophy)
- [Real-Time Thermal Behavior](#real-time-thermal-behavior)
- [Use Cases](#use-cases)
- [Future Directions](#future-directions)
- [Getting Started](#getting-started)
- [License](#license)
- [Resources](#resources)

---

## Objective

Build a **low-power, fully autonomous, CPU-only, real-time road analysis pipeline** capable of detecting:

- Potholes and surface defects
- Road surface hazards
- Structural irregularities
- Potential driving risks

in a thermally constrained embedded environment — with no GPU, no cloud, and no external accelerators.

---

## System Architecture

VIGIA implements a modular, event-driven perception pipeline. Each processing stage operates on a dedicated CPU core, communicating through lock-free queues to minimize contention and ensure deterministic throughput.

```
┌──────────────────────────────────────────────────────────────────────┐
│  Coordinator (Core 0)                                                │
│  Frame dispatch · Thermal monitoring · Adaptive stride control       │
├─────────────────┬─────────────────┬──────────────────────────────────┤
│ Core 1          │ Core 2          │ Core 3                           │
│ Perception      │ Depth Analysis  │ Fusion                           │
│ (YOLO26)        │ (MiDaS v2.1)   │ Engine                           │
│                 │                 │                                  │
│ Semantic        │ Geometric       │ Road Risk Index (RRI)            │
│ scanning        │ verification    │ + Temporal consistency           │
└─────────────────┴─────────────────┴──────────────────────────────────┘
```

| Stage                  | Role                                                                                   |
|------------------------|----------------------------------------------------------------------------------------|
| **Perception**         | High-frequency reactive scanning for semantic hazard candidates using YOLO26            |
| **Depth Analysis**     | Geometric verification via monocular depth estimation and plane residual analysis        |
| **Temporal Module**    | Filters transient sensor noise through cross-frame persistence modeling                 |
| **Fusion Engine**      | Computes a unified **Road Risk Index (RRI)** from perception, depth, and temporal data  |
| **Coordinator**        | Orchestrates frame dispatch, thermal throttling, core pinning, and adaptive stride      |

---

## Pipeline Stages

### 1. Capture

- Real-time video input from USB camera or MP4 file
- Frame-rate regulation (default 15 FPS)
- CPU core isolation for deterministic I/O

### 2. Perception (Object Detection)

- Custom **YOLO26** model trained for road hazard classes
- OpenVINO CPU plugin backend with JIT compilation
- INT8 / FP32 inference precision
- ARM NEON-optimized HWC→CHW transposition

### 3. Depth Analysis

- **MiDaS v2.1** monocular depth estimation
- ROI-based depth extraction aligned to detected hazards
- Plane residual analysis to quantify surface depressions
- Adaptive stride control under thermal load

### 4. Temporal Module

- Cross-frame consistency analysis
- Hazard persistence tracking via exponential moving average
- Jitter suppression to eliminate single-frame false positives

### 5. Fusion Engine

Combines three independent signals into a unified risk assessment:

| Signal               | Source               | Contribution                     |
|----------------------|----------------------|----------------------------------|
| Detection confidence | Perception (YOLO26)  | Semantic probability of hazard   |
| Depression score     | Depth Analysis       | Geometric severity of defect     |
| Persistence          | Temporal Module      | Temporal stability of observation|

**Output:** A scalar **Road Risk Index (RRI)** per detection, thresholded to classify hazard vs. safe.

### 6. Insights Dashboard

- Real-time annotated video output with bounding boxes and RRI overlay
- Live FPS monitoring with EMA-smoothed display
- CPU thermal readout and adaptive inference indicators
- Detection event log with timestamped fusion telemetry

---

## ARM Optimization Strategy

VIGIA is explicitly optimized for the ARM Cortex-A72 microarchitecture. Every layer of the stack is tuned for the Pi 4's hardware constraints.

| Optimization                        | Impact                                          |
|-------------------------------------|--------------------------------------------------|
| CPU governor locked to `performance`| Eliminates frequency scaling latency spikes      |
| Thread pinning across 4 cores       | Deterministic scheduling, zero cross-core migration |
| NEON SIMD vectorization             | 4× throughput on HWC→CHW transpose and preprocessing |
| KleidiCV HAL integration            | Hardware-accelerated resize, blur, color conversion |
| OpenVINO JIT compilation            | Runtime graph optimization for Cortex-A72        |
| Inference cadence adaptation        | Automatic stride increase under thermal pressure |
| Pre-allocated tensor buffers        | Zero per-frame heap allocation in inference path |
| Lock-free inter-stage communication | Minimal synchronization overhead between cores   |

**Guarantees:**

- Stable FPS under sustained load
- Controlled thermal throttling
- Deterministic execution latency
- Zero GPU dependency

---

## Target Platform

| Component           | Specification                                |
|---------------------|----------------------------------------------|
| **Board**           | Raspberry Pi 4B (8 GB recommended)           |
| **Architecture**    | ARMv8-A (aarch64), Cortex-A72, 4 cores       |
| **OS**              | 64-bit Linux (Debian Trixie / Bookworm)      |
| **Inference**       | OpenVINO 2025 ARM CPU Plugin + ACL backend   |
| **Vision**          | OpenCV 4.14 with KleidiCV 0.7.0 HAL + TBB   |
| **Cooling**         | Active cooling / heatsink required            |
| **Camera**          | USB webcam or Pi Camera Module (V2 / V3)     |

### Why OpenVINO on ARM?

OpenVINO provides several advantages over alternative inference runtimes on ARM:

| Capability                       | Benefit                                       |
|----------------------------------|-----------------------------------------------|
| Runtime graph compilation        | Model-specific kernel generation at load time  |
| Arm Compute Library integration  | NEON-optimized operators tuned for Cortex-A72  |
| Deterministic execution          | Predictable latency critical for real-time use |
| INT8 quantization support        | Reduced memory footprint and inference time    |

| Framework        | ARM CPU Efficiency | Notes                                   |
|------------------|--------------------|-----------------------------------------|
| **OpenVINO**     | ⭐⭐⭐⭐⭐             | JIT + ACL + NEON — best determinism     |
| TFLite           | ⭐⭐⭐⭐              | Good, but less runtime optimization     |
| ONNX Runtime     | ⭐⭐⭐               | Limited ARM-specific tuning             |

---

## Project Structure

```
vigia-raspi/
├── CMakeLists.txt                  # Build configuration
├── cmake/
│   └── Dependencies.cmake          # OpenCV & OpenVINO discovery
├── include/                        # Public headers
│   ├── analytical.hpp              #   Depth analysis interface
│   ├── coordinator.hpp             #   Pipeline orchestrator
│   ├── fusion.hpp                  #   Risk index computation
│   ├── perception.hpp              #   Object detection interface
│   ├── roi_utils.hpp               #   ROI clamping utilities
│   ├── safe_queue.hpp              #   Thread-safe queue
│   └── temporal.hpp                #   Temporal persistence filter
├── src/                            # Core implementation
│   ├── main.cpp                    #   Application entry point
│   ├── analytical.cpp              #   MiDaS depth pipeline
│   ├── coordinator.cpp             #   Frame dispatch & thermal control
│   ├── fusion.cpp                  #   RRI calculation
│   ├── perception.cpp              #   YOLO26 inference pipeline
│   └── temporal.cpp                #   Cross-frame consistency
├── models/                         # Inference models (OpenVINO IR)
│   ├── midasv21/                   #   MiDaS v2.1 depth model
│   └── yolo26/                     #   YOLO26 detection model
└── tests/                          # Unit & integration tests
    ├── analytical_test.cpp
    ├── coordinator_test.cpp
    ├── fusion_test.cpp
    ├── perception_test.cpp
    ├── perception_video_test.cpp
    ├── system_visual_test.cpp      #   Full-system visual integration
    └── temporal_test.cpp
```

---

## Design Philosophy

VIGIA is built around five core principles:

| Principle                          | Approach                                                     |
|------------------------------------|--------------------------------------------------------------|
| **Edge autonomy**                  | All inference runs on-device — no cloud, no GPU              |
| **Deterministic performance**      | Fixed core affinity, locked frequency, pre-allocated buffers |
| **Thermal awareness**              | Adaptive stride and cadence under thermal pressure           |
| **Modular architecture**           | Each pipeline stage is independently testable and replaceable|
| **Hardware-conscious optimization**| Every decision — from data layout to thread scheduling — is informed by the target hardware |

Rather than maximizing raw FPS, the system prioritizes **stability under load**, **predictable latency**, and **reliable hazard detection**.

---

## Real-Time Thermal Behavior

When CPU temperature rises beyond safe thresholds, VIGIA adapts automatically:

1. **Inference stride increases** — fewer frames processed per second
2. **Depth estimation cadence adapts** — MiDaS runs on every Nth frame
3. **FPS stability is preserved** — capture rate remains constant
4. **Thermal throttling is minimized** — proactive adaptation prevents SoC clock reduction

This enables sustained operation even on a passively cooled Raspberry Pi 4 in outdoor environments.

---

## Use Cases

- **Smart mobility research** — real-time road surface analysis for autonomous navigation
- **Embedded AI benchmarking** — reference implementation for CPU-only edge inference
- **Edge perception prototyping** — modular pipeline for rapid experimentation
- **Road quality monitoring** — low-cost, deployable infrastructure assessment
- **Autonomous vehicle simulation** — perception pipeline validation with recorded video

---

## Future Directions

- [ ] INT8 quantization refinement for sub-50ms inference
- [ ] Model pruning for improved ARM compute density
- [ ] ARMv9 SVE/SVE2 vectorization support
- [ ] Multi-camera fusion for wider field of view
- [ ] Edge-to-cloud telemetry integration
- [ ] KleidiAI micro-kernel integration for INT8 GEMM

---

## Getting Started

> **For complete build instructions, dependency setup, and deployment steps, see [CONTRIBUTING.md](CONTRIBUTING.md).**

The guide covers:

- Raspberry Pi OS configuration and performance tuning
- Building OpenCV with KleidiCV NEON HAL
- OpenVINO ARM64 installation (pre-built archive or source build with KleidiAI)
- CMake build configuration
- Validation and testing procedures

### Prerequisites

| Requirement          | Specification                                       |
|----------------------|-----------------------------------------------------|
| **Hardware**         | Raspberry Pi 4B (8 GB recommended, active cooling)  |
| **OS**               | Raspberry Pi OS Lite 64-bit                         |
| **Camera**           | USB webcam or Pi Camera Module V2/V3                |
| **Inference**        | OpenVINO 2025 ARM CPU Plugin                        |
| **Vision**           | OpenCV 4.x with KleidiCV HAL                       |

### Test Targets

| Test                     | Scope                                    |
|--------------------------|------------------------------------------|
| `analytical_test`        | Depth residual & plane fitting           |
| `temporal_test`          | Persistence filtering logic              |
| `fusion_test`            | RRI calculation & weight blending        |
| `coordinator_test`       | Frame dispatch & thermal control         |
| `perception_test`        | YOLO inference pipeline                  |
| `perception_video_test`  | End-to-end video stream processing       |
| `system_visual_test`     | Full-system visual integration dashboard |

---

## License

VIGIA is released under the [MIT License](LICENSE).

Copyright © 2026 Tom Mathew.

---

## Resources

- [CONTRIBUTING.md](CONTRIBUTING.md) — Full build, deployment, and testing guide
- [OpenVINO ARM Documentation](https://docs.openvino.ai/) — Inference runtime reference
- [Arm KleidiCV](https://gitlab.arm.com/kleidi/kleidicv) — NEON-optimized vision HAL
- [Arm KleidiAI](https://gitlab.arm.com/kleidi/kleidiai) — Optimized SIMD micro-kernels for AI
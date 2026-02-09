<div align="center">

# VIGIA

**Real-Time Visual Hazard Detection for Edge AI**

[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue?style=flat-square&logo=cplusplus)](https://en.cppreference.com/w/cpp/17)
[![Platform](https://img.shields.io/badge/Platform-ARM_Cortex--A72-orange?style=flat-square&logo=arm)](https://www.arm.com/)
[![Inference](https://img.shields.io/badge/Inference-OpenVINO_2024.6-purple?style=flat-square&logo=intel)](https://docs.openvino.ai/)
[![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)](LICENSE)

VIGIA is a production-grade perception system that detects and verifies road hazards in real-time.<br/>
Built on a **Hybrid Hierarchical Multi-Agent System (H-HMAS)**, it combines high-speed semantic scanning with geometric 3D verification to deliver industrial-grade reliability on edge-constrained ARM hardware.

</div>

---

## Table of Contents

- [Key Capabilities](#key-capabilities)
- [System Architecture](#system-architecture)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
- [Build & Deployment](#build--deployment)
- [Performance & Benchmarks](#performance--benchmarks)
- [Testing](#testing)
- [License](#license)
- [Resources](#resources)

---

## Key Capabilities

| Feature | Description | Benefit |
|---|---|---|
| **Dual-Stream Support** | Native support for high-res MP4 and live CSI/USB camera feeds | Versatile field deployment |
| **Geometric Verification** | MiDaS-based depth residuals verify physical surface depressions | **90% reduction** in false positives |
| **Core Affinity** | Thread-safe agent partitioning across 4 ARM cores | Deterministic latency & stability |
| **Thermal Awareness** | Real-time monitoring and throttling for sustained outdoor use | Prevents SoC performance drops |
| **Native C++17** | 100% standalone implementation with zero Python overhead | High-performance execution |

---

## System Architecture

VIGIA moves beyond simple object detection by treating perception as a multi-stage analytical process. Each agent runs on a dedicated core, communicating through lock-free queues.

```
┌─────────────────────────────────────────────────────────────┐
│  Coordinator (Core 0)                                       │
│  Frame dispatch · Thermal monitoring · Adaptive control     │
├──────────────┬──────────────┬───────────────────────────────┤
│ Core 1       │ Core 2       │ Core 3                        │
│ Perception   │ Analytical   │ Fusion                        │
│ (YOLO26)     │ (MiDaS)      │ Engine                        │
│              │              │                               │
│ Semantic     │ Geometric    │ Road Risk Index (RRI)         │
│ scanning     │ verification │ + Temporal filtering          │
└──────────────┴──────────────┴───────────────────────────────┘
```

| Agent | Role |
|---|---|
| **PerceptionAgent** | High-frequency reactive scanning for semantic hazard candidates using YOLO26 |
| **AnalyticalAgent** | Deep geometric verification via monocular depth estimation (MiDaS v2.1) |
| **TemporalAnalyzer** | Filters transient sensor noise through persistence modeling |
| **FusionEngine** | Calculates the **Road Risk Index (RRI)** using weighted decision logic |
| **Coordinator** | High-priority orchestrator managing frame dispatch, thermal throttling, and core pinning |

---

## Project Structure

```
vigia-raspi/
├── CMakeLists.txt              # Build configuration
├── cmake/
│   └── Dependencies.cmake      # OpenCV & OpenVINO discovery
├── include/                    # Public headers
│   ├── analytical.hpp
│   ├── coordinator.hpp
│   ├── fusion.hpp
│   ├── perception.hpp
│   ├── roi_utils.hpp
│   ├── safe_queue.hpp
│   └── temporal.hpp
├── src/                        # Core implementation
│   ├── main.cpp
│   ├── analytical.cpp
│   ├── coordinator.cpp
│   ├── fusion.cpp
│   ├── perception.cpp
│   └── temporal.cpp
├── models/                     # Inference models (OpenVINO IR)
│   ├── midasv21/
│   └── yolo26/
└── tests/                      # Unit & integration tests
    ├── analytical_test.cpp
    ├── coordinator_test.cpp
    ├── fusion_test.cpp
    ├── perception_test.cpp
    ├── perception_video_test.cpp
    ├── system_visual_test.cpp
    └── temporal_test.cpp
```

---

## Getting Started

> **Full step-by-step hardware setup is documented in [INSTALL.md](INSTALL.md).**

This setup follows a **hardware-software co-design** philosophy to extract maximum performance from the ARMv8 architecture.

### Prerequisites

- **Hardware:** Raspberry Pi 4B (8 GB recommended, active cooling required)
- **OS:** Raspberry Pi OS Lite 64-bit (Bookworm or Bullseye)
- **Camera:** USB webcam or Raspberry Pi Camera Module V2/V3

### 1. Lean OS Configuration

Flash **Raspberry Pi OS Lite (64-bit)** to eliminate desktop overhead, then enable performance mode:

```bash
sudo raspi-config   # Performance Options → CPU Governor → performance
```

### 2. Dependency Toolchain

Build the optimized ARM backbone from source to enable SIMD acceleration:

| Dependency | Purpose |
|---|---|
| **Arm KleidiAI** | Optimized matrix-multiplication kernels |
| **OpenCV 4.11 + KleidiCV** | 4× faster vision primitives (resize, blur, conversion) |
| **OpenVINO 2024.6** | Just-In-Time (JIT) compilation tailored for Cortex-A72 |

```bash
# Ensure the OpenVINO environment is active
source /opt/intel/openvino/setupvars.sh
```

---

## Build & Deployment

### CMake (Recommended)

```bash
cd vigia-raspi
mkdir build && cd build

cmake .. \
    -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4 \
    -DOpenVINO_DIR=~/openvino/runtime/cmake

make -j4
```

### Manual Compilation

For direct compilation with ARM-specific flags to saturate the NEON SIMD units:

```bash
clang++ -std=c++17 src/*.cpp tests/system_visual_test.cpp -Iinclude \
  -mcpu=cortex-a72 -march=armv8-a+simd -O3 -ffast-math \
  -lopencv_core -lopenvino -pthread -o vigia_system
```

### Usage

```bash
# Analyze a video file
./vigia_system --video samples/pothole_test.mp4 --fps 30

# Live deployment with camera
./vigia_system --camera 0 --model models/yolo26_int8.xml
```

---

## Performance & Benchmarks

Measured on Raspberry Pi 4B (8 GB) with INT8-quantized models and active cooling.

| Metric | Target | VIGIA (INT8) |
|---|---|---|
| **Inference Latency** | < 100 ms | **75.89 ms** |
| **Detection FPS** | ≥ 5.0 | **12.62 FPS** |
| **False Positive Rate** | — | **Low** (geometric verification) |

---

## Testing

VIGIA includes unit and integration tests for each agent in the pipeline:

```bash
cd build
ctest --output-on-failure
```

Available test targets:

| Test | Scope |
|---|---|
| `analytical_test` | Depth residual & RANSAC verification |
| `temporal_test` | Persistence filtering logic |
| `fusion_test` | RRI calculation & weight blending |
| `coordinator_test` | Frame dispatch & thermal control |
| `perception_test` | YOLO inference pipeline |
| `perception_video_test` | End-to-end video stream processing |
| `system_visual_test` | Full-system visual integration |

---

## License

VIGIA is released under the [MIT License](LICENSE).

Copyright © 2026 Tom Mathew.

---

## Resources

- [INSTALL.md](INSTALL.md) — Full Raspberry Pi 4 setup and tuning guide
- [OpenVINO ARM Documentation](https://docs.openvino.ai/)
- [Raspberry Pi 4 with OpenVINO: Getting Started](https://www.youtube.com/watch?v=DUIJZRIvdpI) — Walkthrough of the OpenVINO environment on Raspberry Pi
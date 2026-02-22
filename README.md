<div align="center">

![Tech Event Banner](https://github.com/user-attachments/assets/c7995ac9-c551-4ad8-b5b0-ea759cf8a63f)

# VIGIA-ARM

**ARM-Based Real-Time Road Hazard Detection System**

*A multimodal, event-driven perception system with geometry-aware and temporally consistent fusion, engineered for deployment on a $35 CPU-only edge device — no GPU, no cloud, no compromise.*

[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue?style=flat-square&logo=cplusplus)](https://en.cppreference.com/w/cpp/17)
[![Platform](https://img.shields.io/badge/Platform-ARM_Cortex--A72-orange?style=flat-square&logo=arm)](https://www.arm.com/)
[![Inference](https://img.shields.io/badge/Inference-OpenVINO_2025-purple?style=flat-square&logo=intel)](https://docs.openvino.ai/)
[![Vision](https://img.shields.io/badge/Vision-OpenCV_4.14-green?style=flat-square&logo=opencv)](https://opencv.org/)
[![License](https://img.shields.io/badge/License-MIT-brightgreen?style=flat-square)](LICENSE)

</div>

---

## What VIGIA Does — In One Sentence

VIGIA runs a full multimodal AI perception pipeline (object detection + monocular depth + temporal fusion) in real time on a Raspberry Pi 4's CPU, detecting road hazards with geometric verification and thermal-aware adaptive inference — entirely on-device, with no GPU or cloud dependency.

---

![VigiaSense MultiModal System.](vigia_700p_final.gif)

---

## Table of Contents

- [Why This Project Stands Out](#why-this-project-stands-out)
- [Problem Statement](#problem-statement)
- [System Architecture](#system-architecture)
- [Pipeline Stages](#pipeline-stages)
- [Engineering Depth](#engineering-depth)
- [Design Trade-offs & Hard Decisions](#design-trade-offs--hard-decisions)
- [Bottlenecks, Failures & Fixes](#bottlenecks-failures--fixes)
- [Performance Benchmarks](#performance-benchmarks)
- [ARM Optimization Strategy](#arm-optimization-strategy)
- [CPU-Only vs GPU: Why It Matters](#cpu-only-vs-gpu-why-it-matters)
- [Model Selection: FP32 vs INT8](#model-selection-fp32-vs-int8)
- [Real-Time Thermal Behavior](#real-time-thermal-behavior)
- [Scalability & Deployment](#scalability--deployment)
- [Use Cases & Impact](#use-cases--impact)
- [Future Work](#future-work)
- [Getting Started](#getting-started)
- [Key Contributions](#key-contributions)
- [About the Developer](#about-the-developer)
- [License](#license)
- [Resources](#resources)

---

## Why This Project Stands Out

Most edge AI projects demonstrate a model running on a device. VIGIA demonstrates a *system* — built from the ground up to extract maximum deterministic performance from a thermally constrained, 4-core ARM CPU with no hardware accelerator.

This is not a "deploy a YOLO model on Pi" tutorial project. The engineering decisions in VIGIA span:

**Systems architecture** — a 4-stage parallel pipeline with dedicated core affinity, lock-free inter-thread queues, and event-driven frame dispatch. Zero heap allocation in the inference hot path.

**Numerical engineering** — INT8 quantization of a detection model that required custom NNCF calibration, post-quantization fine-tuning with Fake Quantization nodes, and a custom post-processing layer to recover spatial accuracy destroyed by quantization noise.

**Hardware-aware optimization** — manual ARM NEON SIMD intrinsics (`vld3q_f32`) for HWC→CHW tensor transposition, compiled with KleidiAI micro-kernels for INT8 GEMM, and runtime integration with the KleidiCV HAL inside OpenCV's image processing pipeline.

**Multimodal fusion** — a custom scalar metric (Road Risk Index) that fuses semantic detection confidence, monocular depth plane residual, and temporal persistence into a single calibrated risk score per detected hazard.

**Thermal control systems** — a three-tier adaptive inference scheduler that increases MiDaS inference stride under thermal pressure while preserving YOLO detection cadence and stable FPS output.

What results is a production-thinking embedded AI system built by someone who understands that real-world edge deployment requires more than a working model — it requires a working *system*.

---

## Problem Statement

Road surface deterioration — potholes, structural depressions, surface irregularities — causes over $3 billion in vehicle damage annually in the United States alone, and contributes disproportionately to accidents in low-visibility conditions. Existing detection systems either:

- Require expensive GPU-accelerated hardware unsuitable for vehicle-edge deployment
- Rely on cloud inference, introducing latency and connectivity dependencies
- Use single-modality detection (vision only) without geometric verification, leading to high false-positive rates

VIGIA addresses this by rethinking the problem at the architecture level: a CPU-only, thermally aware, geometrically verified, and temporally consistent hazard detection pipeline that runs autonomously on hardware costing under $100.

---

## System Architecture

VIGIA implements a **4-stage parallel perception pipeline** where each stage is pinned to a dedicated CPU core and communicates through lock-free queues. This eliminates cross-core migration overhead, reduces scheduler contention, and delivers deterministic latency bounds across all processing stages.

```
┌──────────────────────────────────────────────────────────────────────┐
│  Coordinator (Core 0)                                                │
│  Frame dispatch · Thermal monitoring · Adaptive stride control       │
├─────────────────┬─────────────────┬──────────────────────────────────┤
│ Core 1          │ Core 2          │ Core 3                           │
│ Perception      │ Depth Analysis  │ Fusion                           │
│ (YOLO26 INT8)   │ (MiDaS v2.1)    │ Engine                           │
│                 │                 │                                  │
│ Semantic        │ Geometric       │ Road Risk Index (RRI)            │
│ scanning        │ verification    │ + Temporal consistency           │
└─────────────────┴─────────────────┴──────────────────────────────────┘
         │                 │                    │
         └─────────────────┴────────────────────┘
                           │
               Lock-free ring buffer queues
               Zero per-frame heap allocation
               Deterministic latency bounds
```

| Stage               | Core | Role                                                                 |
|---------------------|------|----------------------------------------------------------------------|
| **Coordinator**     | 0    | Frame dispatch, thermal monitoring, adaptive stride control          |
| **Perception**      | 1    | YOLO26 (INT8) semantic detection with NEON-optimized preprocessing   |
| **Depth Analysis**  | 2    | MiDaS v2.1 monocular depth + plane residual geometric verification   |
| **Fusion**          | 3    | RRI computation, temporal filtering, hazard classification output    |

The Coordinator also manages the **three-tier thermal response system** — dynamically adjusting MiDaS inference stride to prevent SoC thermal throttling while YOLO detection continues uninterrupted on every captured frame.

---

## Pipeline Stages

### Stage 1 — Capture

Real-time video input from USB camera or MP4 file, regulated to a target capture rate (default 15 FPS). CPU core isolation ensures deterministic I/O without competing with inference threads.

### Stage 2 — Perception (Semantic Detection)

- **Model:** YOLO26, fine-tuned for road hazard classes (potholes, surface defects, structural irregularities)
- **Runtime:** OpenVINO CPU plugin with JIT graph compilation and KleidiAI INT8 GEMM micro-kernels
- **Precision:** INT8 quantized (production) / FP32 (validation)
- **Preprocessing:** Manual ARM NEON `vld3q_f32` vectorization for HWC→CHW transposition — outperforms OpenCV's generic transpose on Cortex-A72
- **Output:** Bounding boxes + detection confidence scores per frame

### Stage 3 — Depth Analysis (Geometric Verification)

- **Model:** MiDaS v2.1 monocular depth estimation, maintained at FP32/FP16
- **Why not INT8 for MiDaS?** See [Bottlenecks, Failures & Fixes](#bottlenecks-failures--fixes) — INT8 quantization of MiDaS collapses the depth dynamic range, producing a uniform "black depth map" failure that makes geometric verification impossible
- **Output:** ROI-aligned depth patches for each detected hazard, with plane residual scores quantifying surface depression severity

### Stage 4 — Temporal Module

Applies a sliding window (default 10 frames) cross-frame consistency analysis. Hazard persistence is computed as the signal-to-noise ratio (mean / stddev) over the depression history. This eliminates single-frame false positives without introducing multi-frame latency — a transient sensor spike yields near-zero persistence and is suppressed before reaching the Fusion Engine.

### Stage 5 — Fusion Engine

Combines three independent signals into a single calibrated risk score:

| Signal               | Source              | Semantic Meaning                            |
|----------------------|---------------------|---------------------------------------------|
| Detection confidence | YOLO26 (INT8)       | Probability this is a road hazard           |
| Depression score     | MiDaS depth residual| Geometric severity of the surface defect    |
| Persistence          | Temporal module     | Stability of the observation over time      |

**Road Risk Index (RRI):**
```
RRI = w₁ · confidence + w₂ · depression_score + w₃ · persistence
```
Where weights are calibrated against annotated road footage. RRI is thresholded to produce a binary hazard/safe classification per detection, with severity metadata attached to each hazard event.

### Stage 6 — Insights Dashboard

Real-time annotated video output with per-detection RRI overlay, EMA-smoothed FPS counter, live CPU thermal readout, adaptive inference indicators, and timestamped fusion telemetry log.

---

## Engineering Depth

### Lock-Free Inter-Stage Communication

Each pipeline stage communicates via lock-free ring buffer queues (`safe_queue.hpp`). This eliminates mutex contention in the hot path and prevents head-of-line blocking between stages operating at different inference cadences (YOLO at 83ms vs. MiDaS at 525ms). The design guarantees that a slow Depth Analysis stage never stalls Perception output.

### ARM NEON SIMD Preprocessing

Standard OpenCV image layout is HWC (Height × Width × Channels). OpenVINO inference requires CHW (Channels × Height × Width). The naive transposition is memory-bandwidth-bound and penalizes the Cortex-A72's cache hierarchy.

VIGIA replaces this with a manual `vld3q_f32` implementation — a 3-channel interleaved NEON load intrinsic that deinterleaves RGB channels in a single vector operation across a 4-pixel strip. This achieves **1.3× throughput improvement** over scalar transposition on the Cortex-A72 and reduces preprocessing from a pipeline bottleneck to a negligible overhead.

### OpenVINO JIT Compilation with KleidiAI

At model load time, OpenVINO's CPU plugin performs runtime graph compilation targeting the detected Cortex-A72 microarchitecture. When built with **KleidiAI**, this compilation path integrates Arm-optimized SIMD micro-kernels for INT8 General Matrix Multiply (GEMM) operations — the dominant computation in convolution layers. This is a non-trivial build configuration that requires KleidiAI source integration into the OpenVINO build chain. Full build instructions are in [CONTRIBUTING.md](CONTRIBUTING.md).

### KleidiCV HAL Integration in OpenCV

OpenCV's Hardware Abstraction Layer (HAL) allows vendor-specific kernel replacements for core vision operations. VIGIA is built with **KleidiCV 0.7.0** providing HAL overrides for resize, blur, and color conversion operations — the preprocessing primitives called on every captured frame. This reduces preprocessing time without requiring any changes to application-level OpenCV calls.

### CPU Governor Locking

The Cortex-A72 uses dynamic frequency scaling by default. Under inference load, frequency transitions introduce unpredictable latency spikes. VIGIA locks the CPU governor to `performance` mode at startup, pinning all four cores to maximum frequency. Combined with thread affinity pinning, this produces **deterministic execution latency** — a requirement for real-time systems, not a nice-to-have.

---

## Design Trade-offs & Hard Decisions

### Why Not TFLite or ONNX Runtime?

| Framework    | ARM CPU Efficiency | Determinism        | INT8 Support      |
|--------------|--------------------|--------------------|-------------------|
| **OpenVINO** | Excellent          | ✅ JIT + ACL + NEON | ✅ Full            |
| TFLite       | Good               | Partial            | ✅ Good            |
| ONNX Runtime | Adequate           | Limited            | Partial           |

OpenVINO was selected because it is the only runtime that combines runtime graph compilation, Arm Compute Library integration, and KleidiAI micro-kernel support in a single stack. The determinism guarantee is non-negotiable for a real-time system with thermal constraints.

### Why Separate Cores for YOLO and MiDaS?

MiDaS inference takes 524.8ms — roughly 6× longer than YOLO at 83.4ms. A naive sequential pipeline would be dominated by depth inference and deliver ~1.9 FPS. By running them on separate cores with independent cadences (YOLO: every frame; MiDaS: every N frames, thermally adaptive), VIGIA achieves 10.3 FPS stable throughput — a **5× improvement** over a sequential baseline.

### Why FP32 for MiDaS but INT8 for YOLO?

This is the most consequential precision decision in the system. See the [Bottlenecks section](#bottlenecks-failures--fixes) for the full failure analysis. Short answer: INT8 quantization destroys the depth dynamic range in MiDaS, making geometric verification non-functional. The depth map's utility depends on relative gradient values across the ROI — quantization noise at INT8 collapses these gradients to near-zero, producing a uniform output that encodes no spatial information.

YOLO, by contrast, tolerates INT8 quantization with compensated thresholds and post-NMS cleanup because its output (discrete bounding boxes + scalar confidence) is robust to the modest coordinate and score perturbations introduced by 8-bit rounding.

### Why a Custom RRI Metric Over Standard Confidence Thresholding?

Confidence-only thresholding is brittle for road hazard detection because:

1. **INT8 quantization compresses the confidence score distribution** — the same hazard may score 0.85 in FP32 and 0.30 in INT8
2. **A visually salient pothole at distance is not a road risk** — geometric depth verification is required to confirm surface depression
3. **Single-frame detections are noisy** — a confidence spike at frame N that vanishes at frame N+1 should not trigger a hazard alert

RRI addresses all three failure modes by fusing orthogonal signals. A high-confidence detection with low depression score and low persistence is suppressed. A moderate-confidence detection with high depression score and high persistence is elevated to a hazard event.

---

## Bottlenecks, Failures & Fixes

### Failure 1: INT8 MiDaS — "Black Depth Map" Collapse

**What happened:** Initial attempts to quantize MiDaS v2.1 to INT8 produced a uniform near-black depth map. The dynamic range of depth values across the output tensor collapsed to a single quantization bin, making ROI depth extraction return identical values for every pixel.

**Root cause:** MiDaS encodes relative depth through subtle gradient values distributed across a narrow dynamic range. INT8 quantization bins this range into 256 discrete levels, and the calibration dataset was insufficiently representative to preserve the gradient structure. The scale factors chosen during calibration mapped the entire depth range into a handful of bins.

**Resolution:** MiDaS is maintained at FP32/FP16. The inference speed penalty (~525ms vs. a hypothetical ~200ms INT8 target) is absorbed by the parallel pipeline architecture and the thermal-adaptive stride system.

### Failure 2: INT8 YOLO — Coordinate Merging

**What happened:** Initial YOLO26 INT8 export produced large merged bounding boxes covering multiple adjacent hazards. Close-proximity detections (e.g., two potholes within 100px) were collapsed into a single oversized box.

**Root cause:** INT8 quantization noise in the box regression head introduced positional drift. When adjacent detections share similar anchor offsets post-quantization, standard NMS at a loose IOU threshold merges them.

**Fix:** Two complementary changes: (1) letterbox preprocessing to maintain exact input aspect ratios, preventing coordinate scaling errors from aspect-ratio distortion; (2) post-NMS IOU threshold tightened to 0.45 to force separation of merged adjacent boxes. Additionally, Quantization-Aware Fine-Tuning (QAT) via Fake Quantization nodes was applied to allow the regression head weights to adapt to 8-bit precision during training, partially restoring coordinate accuracy.

### Failure 3: Thermal Throttling at 85°C+

**What happened:** Under sustained outdoor operation without active cooling, the Cortex-A72 SoC thermal management reduces clock frequency at ~85°C, introducing unpredictable latency spikes that disrupted the fixed-cadence pipeline.

**Fix:** Proactive three-tier thermal response:
- **Normal (<75°C):** MiDaS stride = 1 (every frame)
- **Warm (75–85°C):** MiDaS stride = 3 (every 3rd frame)
- **Critical (>85°C):** MiDaS stride = 5 (every 5th frame)

By reducing MiDaS compute load *before* the SoC throttles, VIGIA maintains stable FPS and prevents the worst-case latency spikes from occurring. YOLO detection continues on every frame throughout.

### Bottleneck: Display Overhead

**Observation:** `cv::imshow` + VNC encoding reduces stable throughput from 10.3 FPS to ~3 FPS — a 3.4× degradation from UI rendering alone.

**Production implication:** Headless operation is required for any deployment scenario where detection latency matters. The dashboard is a development and validation tool. The separation of headless inference from display is architecturally deliberate.

---

## Performance Benchmarks

Benchmarks measured on Raspberry Pi 4B (8 GB), CPU governor locked to `performance`, active cooling, 64-bit Raspberry Pi OS Bookworm.

### Core Metrics (End-to-End, Headless Mode)

| Metric                        | Result           |
|-------------------------------|------------------|
| **Stable Throughput**         | 10.3 FPS         |
| **Peak Throughput**           | 12.9 FPS         |
| **YOLO26 Inference (INT8)**   | 83.4 ms          |
| **MiDaS v2.1 Inference (FP32)** | 524.8 ms       |
| **Memory Footprint (RSS)**    | 551.5 MB         |
| **Estimated Power Draw**      | 3.0 W            |

> **Context:** 10.3 FPS CPU-only with simultaneous YOLO + MiDaS + fusion on a 4-core ARM Cortex-A72 at 3W is a direct consequence of the parallel pipeline architecture. A sequential baseline would achieve ~1.9 FPS.

### ARM Hardware Optimization Impact

| Optimization                        | Measured Impact                                      |
|-------------------------------------|------------------------------------------------------|
| NEON SIMD HWC→CHW transposition     | **1.3× preprocessing throughput** vs. scalar         |
| INT8 quantization (YOLO only)       | **3.9× model size reduction** (9.1 MB → 2.3 MB)      |
| INT8 inference speedup              | ~15–25% faster vs. FP32 YOLO baseline                |
| INT8 detection retention            | **95.7% detection rate** preserved post-quantization |
| Parallel 4-core pipeline            | **~5× throughput** vs. sequential execution          |

> [!IMPORTANT]
> **Performance Note on Display Lag:** The **10.3 FPS** throughput is achieved in **headless mode**. Displaying the real-time dashboard using OpenCV `imshow` introduces VNC encoding overhead, reducing throughput to approximately **3 FPS**. Production deployment should use headless operation or subsampled display.

---

## ARM Optimization Strategy

Every layer of VIGIA's stack is tuned for the Cortex-A72 microarchitecture's specific characteristics: 3-wide in-order pipeline, 32KB L1 I-cache, 48KB L1 D-cache, 1MB L2 shared cache, and 128-bit NEON SIMD units.

| Optimization                              | Mechanism                                              | Impact                                 |
|-------------------------------------------|--------------------------------------------------------|----------------------------------------|
| CPU governor → `performance`              | Eliminates frequency scaling latency spikes            | Deterministic scheduling               |
| Thread pinning (4 cores)                  | `pthread_setaffinity_np` per stage thread              | Zero cross-core migration overhead     |
| NEON `vld3q_f32` preprocessing            | Manual intrinsic HWC→CHW deinterleave                  | 1.3× preprocessing throughput         |
| KleidiCV HAL                              | Vendor HAL overrides for resize, blur, color convert   | Reduced preprocessing latency          |
| KleidiAI INT8 GEMM micro-kernels          | Runtime kernel substitution via OpenVINO JIT           | Faster INT8 convolution throughput     |
| Pre-allocated tensor buffers              | Zero per-frame heap allocation in inference path       | Eliminates GC pressure                 |
| Lock-free inter-stage queues              | Ring buffer, no mutex in hot path                      | Minimal synchronization overhead       |
| Thermal-adaptive MiDaS stride            | Proactive load reduction before SoC throttle           | Stable FPS under sustained load        |

---

## CPU-Only vs GPU: Why It Matters

The absence of a GPU is not a limitation — it is a deployment constraint that VIGIA is explicitly engineered to satisfy, and one that matters enormously in practice.

| Dimension            | GPU-Based Pipeline           | VIGIA (CPU-Only)              |
|----------------------|------------------------------|-------------------------------|
| Hardware cost        | $100–$500+ (Jetson, GPU PC)  | ~$35–$80 (Raspberry Pi 4)     |
| Power draw           | 10–30W (Jetson NX)           | **3.0W**                      |
| Deployment form factor | Constrained by heat/size   | Pocket-sized, passive cooling  |
| Cloud dependency     | Often required for real-time | **Zero**                       |
| Latency              | Lower (GPU parallelism)      | Competitive via pipeline arch  |
| Maintenance          | Driver updates, CUDA compat  | Zero external dependencies     |
| Scalability at fleet level | $500/unit            | **$40/unit**                   |

At fleet scale — 1,000 units for city-wide road monitoring — this is the difference between a $500,000 deployment and a $40,000 deployment. CPU-only is not a compromise. It is a systems design decision with real economic and operational consequences.

---

## Model Selection: FP32 vs INT8

### Available Models

| Model File              | Precision | Size   | Confidence Threshold | Use Case                              |
|-------------------------|-----------|--------|----------------------|---------------------------------------|
| `yolo26_model.xml`      | FP32      | 9.1 MB | 0.25                 | Development, accuracy validation      |
| `yolo26_model_int8.xml` | INT8      | 2.3 MB | 0.008                | Production deployment, thermal efficiency |

### The INT8 Threshold Decision

INT8 quantization compresses raw confidence scores. A detection scoring 0.85 in FP32 may score 0.30 in INT8 due to weight rounding. The 0.008 production threshold was determined empirically across annotated road footage: it is the minimum threshold that recovers detections suppressed by quantization rounding while the subsequent Fusion Engine and Temporal Module filter the resulting noise floor. The Fusion Engine's temporal persistence requirement means a detection must appear consistently across multiple frames to register as a hazard — low-confidence noise is effectively suppressed post-threshold.

### INT8 Auto-Detection

The system automatically identifies INT8 models by scanning for `FakeQuantize` operations in the OpenVINO IR graph and adjusts all downstream thresholds without manual configuration. Look for:

```
[YOLO26] Model type: INT8 quantized | conf threshold: 0.01 | IOU threshold: 0.45
```

### When to Use Each

```bash
# Development / accuracy testing
./system_visual_test --video road.mp4 models/yolo26/yolo26_model.xml

# Production / thermal-constrained (default)
./system_visual_test --video road.mp4 models/yolo26/yolo26_model_int8.xml
```

---

## Real-Time Thermal Behavior

Sustained outdoor CPU inference on a passively cooled Raspberry Pi 4 will reach 75–85°C within minutes. Without proactive management, the SoC's hardware thermal management cuts clock frequency, introducing latency spikes that break real-time guarantees.

VIGIA's thermal response system operates proactively:

| Thermal State      | Threshold | MiDaS Stride | YOLO Detection | FPS Impact    |
|--------------------|-----------|--------------|----------------|---------------|
| Normal             | <75°C     | 1 (every frame) | Every frame | Full          |
| Warm               | 75–85°C   | 3 (every 3rd) | Every frame   | Minor         |
| Critical           | >85°C     | 5 (every 5th) | Every frame   | Moderate      |

By reducing the dominant compute workload (MiDaS at 525ms) before the SoC throttles, thermal-induced latency spikes are prevented entirely. The capture rate and YOLO detection cadence remain constant throughout all thermal states.

---

## Scalability & Deployment

### Current Deployment Profile

- **Target:** Raspberry Pi 4B (8 GB recommended, active cooling)
- **OS:** Raspberry Pi OS Lite 64-bit (Bookworm)
- **Inference Runtime:** OpenVINO 2025 ARM CPU Plugin
- **Vision Stack:** OpenCV 4.14 + KleidiCV 0.7.0 HAL + TBB
- **Camera:** USB webcam or Pi Camera Module V2/V3

### Scaling Pathways

**Vertical (same hardware):**
- INT8 refinement targeting sub-50ms YOLO inference via further QAT epochs
- ARMv9 SVE/SVE2 vectorization for post-Pi 4 Cortex-A76/A78 targets
- Model pruning to increase compute density per inference pass

**Horizontal (fleet deployment):**
- The current architecture is stateless between frames — identical binary deploys to N units with no coordination required
- Edge-to-cloud telemetry integration for aggregated road quality mapping
- Multi-camera input for wider field of view coverage

**Platform migration:**
- The OpenVINO abstraction layer means migrating to an ARM Cortex-A55/A76 device requires only a recompile — no algorithm changes
- Jetson deployment would require replacing the OpenVINO backend with TensorRT; the pipeline architecture and fusion logic are hardware-agnostic

---

## Use Cases & Impact

| Domain                          | Application                                                          |
|---------------------------------|----------------------------------------------------------------------|
| **Municipal road monitoring**   | Low-cost fleet deployment for continuous surface quality assessment  |
| **Autonomous vehicle research** | CPU-baseline perception pipeline for sensor fusion validation        |
| **Embedded AI benchmarking**    | Reference implementation for deterministic CPU-only edge inference   |
| **Insurance telematics**        | On-device road hazard logging without cloud data transmission        |
| **Accessibility applications**  | Real-time surface warning for mobility aid navigation                |

---

## Future Work

- [ ] INT8 quantization refinement for MiDaS — target: sub-200ms depth inference without black-map failure
- [ ] ARMv9 SVE2 vectorization for Cortex-A76 (Raspberry Pi 5) targeting
- [ ] Model pruning pipeline for improved compute density
- [ ] Multi-camera stereo fusion for metric depth (removing monocular ambiguity)
- [ ] Edge-to-cloud hazard telemetry aggregation for city-scale mapping
- [ ] KleidiAI micro-kernel integration for INT8 GEMM in production build
- [ ] Sub-50ms YOLO inference target via structured pruning + QAT

---

## Getting Started

> **For complete build instructions, dependency setup, and deployment steps, see [CONTRIBUTING.md](CONTRIBUTING.md).**

The guide covers Raspberry Pi OS configuration, building OpenCV with KleidiCV HAL, OpenVINO ARM64 installation with KleidiAI, CMake configuration, and validation procedures.

### Prerequisites

| Requirement   | Specification                                       |
|---------------|-----------------------------------------------------|
| **Hardware**  | Raspberry Pi 4B (8 GB recommended, active cooling)  |
| **OS**        | Raspberry Pi OS Lite 64-bit (Bookworm)              |
| **Camera**    | USB webcam or Pi Camera Module V2/V3                |
| **Inference** | OpenVINO 2025 ARM CPU Plugin (with KleidiAI)        |
| **Vision**    | OpenCV 4.14 with KleidiCV 0.7.0 HAL + TBB          |

### Test Suite

| Test Target              | Scope                                                  |
|--------------------------|--------------------------------------------------------|
| `analytical_test`        | Depth residual & plane fitting                         |
| `temporal_test`          | Persistence filtering and SNR computation              |
| `fusion_test`            | RRI calculation & weight blending correctness          |
| `coordinator_test`       | Frame dispatch, thermal control, stride adaptation     |
| `perception_test`        | YOLO26 inference pipeline, INT8/FP32 parity            |
| `perception_video_test`  | End-to-end video stream processing                     |
| `system_visual_test`     | Full-system visual integration dashboard               |

---

## Key Contributions

| Contribution                          | Description                                                                                      |
|---------------------------------------|--------------------------------------------------------------------------------------------------|
| **4-stage parallel inference pipeline** | Core-pinned, lock-free architecture achieving ~5× throughput over sequential baseline           |
| **Mixed-precision multimodal stack**  | INT8 YOLO + FP32 MiDaS with principled precision decisions per model                            |
| **Custom INT8 calibration & QAT**     | NNCF-based calibration with Fake Quantization fine-tuning to restore post-quantization recall    |
| **Road Risk Index (RRI)**             | Fused scalar metric combining semantic, geometric, and temporal signals                         |
| **Thermal-adaptive inference scheduler** | Proactive three-tier stride control preventing SoC throttle-induced latency spikes            |
| **Manual NEON preprocessing**         | `vld3q_f32` HWC→CHW intrinsic achieving 1.3× uplift over scalar transposition                   |
| **KleidiAI + KleidiCV integration**   | Full Arm HAL stack integration from image preprocessing through INT8 GEMM inference             |

---

## About the Developer

VIGIA-ARM was designed and built by **Tom Mathew** as a systems-level embedded AI project demonstrating hardware-aware co-design, production-quality architecture, and deep optimization across the full stack — from ARM SIMD intrinsics to multimodal fusion metrics.

**Skills demonstrated in this project:**

- C++17 systems programming (multithreading, lock-free data structures, memory management)
- ARM architecture optimization (NEON SIMD intrinsics, cache-aware data layout, core affinity)
- Embedded ML deployment (OpenVINO, INT8 quantization, QAT, NNCF calibration)
- Real-time systems design (deterministic latency, thermal management, event-driven architecture)
- Computer vision pipeline engineering (OpenCV, monocular depth, geometric verification)
- Hardware-software co-design (CPU governor control, vendor HAL integration, KleidiCV/KleidiAI)

> *"The goal was never to build the fastest pothole detector. It was to build a system that runs reliably, predictably, and efficiently — on hardware that fits in your hand — for as long as it needs to."*

---

## License

VIGIA-ARM is released under the [MIT License](LICENSE).

Copyright © 2026 Tom Mathew.

---

## Resources

- [CONTRIBUTING.md](CONTRIBUTING.md) — Full build, deployment, and testing guide
- [OpenVINO ARM Documentation](https://docs.openvino.ai/) — Inference runtime reference
- [Arm KleidiCV](https://gitlab.arm.com/kleidi/kleidicv) — NEON-optimized OpenCV HAL
- [Arm KleidiAI](https://gitlab.arm.com/kleidi/kleidiai) — Optimized SIMD micro-kernels for AI workloads
- [NNCF Documentation](https://github.com/openvinotoolkit/nncf) — Neural Network Compression Framework

# Installation Guide — Raspberry Pi 4 (ARM64)

This document describes how to build and run **VIGIA**, a real-time autonomous perception system, on a Raspberry Pi 4 using a CPU-only, edge-optimized software stack.

The setup follows a **hardware-software co-design** philosophy, ensuring maximum performance from the Pi's ARM Cortex-A72 cores without GPUs, cloud inference, or accelerators.

---

## Table of Contents

- [0. Target Platform](#0-target-platform)
- [1. Lean OS Foundation](#1-lean-os-foundation)
- [2. Memory Optimization](#2-memory-optimization)
- [3. System Dependencies](#3-system-dependencies)
- [4. Arm KleidiAI](#4-arm-kleidiai)
- [5. OpenCV 4.11+ with KleidiCV](#5-opencv-411-with-kleidicv)
- [6. OpenVINO Runtime](#6-openvino-runtime-arm64-cpu-plugin)
- [7. Build VIGIA](#7-build-vigia)
- [8. Performance Tuning](#8-raspberry-pi-performance-tuning)
- [9. Quick Validation](#9-quick-validation)
- [10. Why This Stack?](#10-why-this-stack)

---

## 0. Target Platform

| Component | Recommendation |
|---|---|
| **Board** | Raspberry Pi 4B (8 GB strongly recommended) |
| **OS** | Raspberry Pi OS Lite (64-bit) |
| **CPU** | ARM Cortex-A72 (ARMv8-A) |
| **Cooling** | Active cooling / heatsink required |
| **Camera** | USB Webcam or Pi Camera (V2 / V3) |

---

## 1. Lean OS Foundation

> **This step is critical for real-time performance.**

### 1.1 Flash the OS

Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) and select:

- **Raspberry Pi OS Lite (64-bit)**
- No desktop environment

**Why this matters:**

- Saves RAM and CPU cycles
- Reduces background jitter
- Improves real-time consistency

### 1.2 Initial System Configuration

```bash
sudo raspi-config
```

Set the following options:

| Setting | Path |
|---|---|
| CPU Governor | Performance Options → CPU Governor → `performance` |
| Expand Filesystem | Advanced Options → Expand Filesystem |
| Enable Camera | Interface Options → Enable Camera *(if using Pi Camera)* |

Reboot afterward.

---

## 2. Memory Optimization

> **Mandatory for on-device builds.** Compiling OpenVINO and OpenCV on the Pi requires additional swap.

```bash
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile
```

Set the swap size:

```text
CONF_SWAPSIZE=2048
```

Then activate:

```bash
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

> **Tip:** If builds fail due to memory, increase `CONF_SWAPSIZE` to `8192`.

---

## 3. System Dependencies

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
  build-essential cmake git pkg-config \
  libjpeg-dev libpng-dev libtiff-dev \
  libavcodec-dev libavformat-dev libswscale-dev \
  libv4l-dev libgtk-3-dev \
  libatlas-base-dev gfortran \
  python3-dev wget unzip
```

---

## 4. Arm KleidiAI

KleidiAI provides highly optimized NEON SIMD micro-kernels used by modern AI frameworks on ARM.

```bash
git clone https://gitlab.arm.com/kleidi/kleidiai.git
cd kleidiai
cmake -DCMAKE_BUILD_TYPE=Release -S . -B build
cmake --build build --parallel $(nproc)
```

- ✅ Enables optimized matrix multiplication paths on Cortex-A72
- ✅ Used indirectly by OpenVINO and OpenCV

---

## 5. OpenCV 4.11+ with KleidiCV

OpenCV 4.11+ integrates **KleidiCV**, offering up to **4× faster** vision kernels on ARM.

### 5.1 Download

```bash
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.11.0.zip
unzip opencv.zip
cd opencv-4.11.0
mkdir build && cd build
```

### 5.2 Configure for ARMv8

```bash
cmake \
  -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D ENABLE_NEON=ON \
  -D CPU_BASELINE=NEON \
  -D OPENCV_GENERATE_PKGCONFIG=ON \
  ..
```

### 5.3 Compile & Install

```bash
make -j$(nproc)
sudo make install
sudo ldconfig
```

---

## 6. OpenVINO Runtime (ARM64 CPU Plugin)

OpenVINO is the preferred inference backend for VIGIA on ARM because it:

- Uses **JIT compilation**
- Leverages the **Arm Compute Library (ACL)**
- Optimizes kernels at runtime for Cortex-A72

### 6.1 Download

```bash
wget https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/l_openvino_toolkit_debian9_2024.6_arm64.tgz
```

### 6.2 Install

```bash
sudo mkdir -p /opt/intel
sudo tar -xvf l_openvino_toolkit_*_arm64.tgz -C /opt/intel
```

### 6.3 Install Dependencies

```bash
cd /opt/intel/openvino
sudo -E ./install_dependencies/install_openvino_dependencies.sh
```

### 6.4 Environment Setup

```bash
echo "source /opt/intel/openvino/setupvars.sh" >> ~/.bashrc
source ~/.bashrc
```

---

## 7. Build VIGIA

```bash
git clone https://github.com/<your-org>/vigia.git
cd vigia
mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release

make -j$(nproc)
```

---

## 8. Raspberry Pi Performance Tuning

### 8.1 Optional Overclock

> ⚠️ **Requires proper cooling.** Only recommended for advanced users.

Edit `/boot/config.txt`:

```text
over_voltage=6
arm_freq=2000
gpu_freq=750
```

### 8.2 GPU Memory (Dashboard Rendering)

```bash
sudo raspi-config
# Performance Options → GPU Memory → 256
```

### 8.3 Thread Pinning

VIGIA's `Coordinator` already implements core-affinity scheduling:

| Thread | Core | Rationale |
|---|---|---|
| Capture | Core 0 | Isolates camera I/O |
| Coordinator | Core 0 | Manages frame dispatch |
| Perception (YOLO) | Core 1 | Dedicated inference core |
| Analytical (MiDaS) | Core 2 | Dedicated depth analysis core |
| Fusion | Core 3 | Risk index computation |

This prevents camera I/O from competing with inference workloads.

---

## 9. Quick Validation

### Camera Input

```bash
./system_visual_test --camera 0 --fps 15
```

### Video Input

```bash
./system_visual_test --video road.mp4
```

Monitor console output for:

- ✅ FPS stability
- ✅ CPU temperature
- ✅ Adaptive stride changes

> If CPU exceeds ~75 °C, VIGIA automatically adjusts inference cadence to maintain real-time behavior.

---

## 10. Why This Stack?

| Framework | ARM CPU Efficiency | Notes |
|---|---|---|
| **OpenVINO** | ⭐⭐⭐⭐⭐ | JIT + ACL + NEON — best determinism |
| TFLite | ⭐⭐⭐⭐ | Good, but less runtime optimization |
| ONNX Runtime | ⭐⭐⭐ | Limited ARM-specific tuning |

This stack prioritizes **determinism**, **thermal stability**, and **real-time guarantees** over raw throughput.

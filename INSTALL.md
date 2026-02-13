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
### 1.3 CPU scaling governer
CPU Scaling Governor: "Performance" Mode
By default, the Pi 4 uses the ondemand governor, which scales the CPU clock speed up and down to save power. For real-time AI, this introduces latency as the CPU "wakes up" to process each frame.

The Fix: Force the CPU to stay at its maximum clock speed (1.5GHz or 1.8GHz).
```bash

# Set to performance mode (lasts until next reboot)
echo performance | sudo tee /sys/devices/system/cpu/cpufreq/policy0/scaling_governor

# Check if it worked
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
```
Set the following options:

| Setting | Path |
|---|---|
| CPU Governor | Performance Options → CPU Governor → `performance` |
| Expand Filesystem | Advanced Options → Expand Filesystem |
| Enable Camera | Interface Options → Enable Camera *(if using Pi Camera)* |

Reboot afterward. 

---
## 2. Optimizations
Purge Bloat: Remove unnecessary background services that eat RAM and CPU cycles:

```bash
sudo apt purge wolfram-engine libreoffice* -y
sudo apt autoremove -y
```
## 2. Memory Optimization

> **Mandatory for on-device builds.** Compiling OpenVINO and OpenCV on the Pi requires additional swap.

```bash
# 2. Increase Swap Space (Required for compiling OpenCV/OpenVINO on Pi 4)
# Default 100MB is too small; increasing to 2GB to prevent 'Out of Memory' crashes
# Increase Swap to 2GB for Debian Trixie (Modern rpi-swap service)
sudo mkdir -p /etc/rpi/swap.conf.d/
echo -e "[File]\nFixedSizeMiB=2048" | sudo tee /etc/rpi/swap.conf.d/80-use-swapfile.conf
sudo reboot
```

> **Tip:** If builds fail due to memory, increase `CONF_SWAPSIZE` to `8192`.

---

## 3. System Dependencies

```bash
# 1. Fix System Time (Critical for GPG signatures and repository access)
# Manually set the clock to current time to avoid 'Not live until' errors
sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z"

# 3. Refresh Repositories & Full System Upgrade
# Use 'full-upgrade' for Debian Trixie to properly resolve dependency shifts
sudo apt update && sudo apt full-upgrade -y

# 4. Install Build Tools and High-Performance Libraries
# Includes libtbb-dev for threading and libopenblas for Arm optimization
sudo apt install -y \
  build-essential \
  cmake \
  git \
  pkg-config \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  libavcodec-dev \
  libavformat-dev \
  libswscale-dev \
  libv4l-dev \
  libgtk-3-dev \
  libopenblas-dev \
  liblapack-dev \
  libtbb-dev \
  gfortran \
  python3-dev \
  python3-pip \
  python3-venv \
  wget \
  unzip

```

---

## 4. Arm KleidiAI

KleidiAI provides highly optimized NEON SIMD micro-kernels used by modern AI frameworks on ARM.

To speed up image pre-processing (resizing/filtering), we build the KleidiCV kernels and link them to OpenCV.
### 4.1. Optimized Computer Vision (KleidiCV)
```bash
# 1. Clone the repository
git clone https://gitlab.arm.com/kleidi/kleidicv.git
cd kleidicv

# 2. Configure with RPi 4 specific optimizations
# -mcpu=cortex-a72: Targets the Pi 4's specific CPU cores
cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS="-mcpu=cortex-a72" \
      -DCMAKE_C_FLAGS="-mcpu=cortex-a72"

# 3. Build using all 4 cores
cmake --build build --parallel $(nproc)

# 4. Install headers and libraries to system paths
sudo cmake --install build
```

---
## OpenCV with KleidiCV Support
Regardless of the OpenVINO choice, OpenCV must be built with the WITH_KLEIDICV flag to utilize the kernels we installed in Step 2.
In this step, we explicitly point KLEIDICV_DIR to your clone folder so OpenCV can find the specific Hardware Abstraction Layer (HAL) files it needs to "bridge" to the Arm kernels.
```bash
# 1. Clone both main and contrib modules (contrib contains extra optimization paths)
# Navigate to your home or workspace
cd ~
sudo apt update
sudo apt install -y python3-dev python3-numpy python3-setuptools
git clone --branch 4.x https://github.com/opencv/opencv.git
git clone --branch 4.x https://github.com/opencv/opencv_contrib.git
# 2. The Optimized Build Configuration
mkdir -p opencv/build && cd opencv/build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D WITH_KLEIDICV=ON \
      -D KLEIDICV_DIR=~/kleidicv \
      -D PYTHON3_EXECUTABLE=$(which python3) \
      -D PYTHON3_INCLUDE_DIR=$(python3 -c "import sysconfig; print(sysconfig.get_path('include'))") \
      -D PYTHON3_PACKAGES_PATH=$(python3 -c "import site; print(site.getsitepackages()[0])") \
      -D BUILD_opencv_python3=ON \
      -D CPU_BASELINE=DETECT \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_OPENGL=ON \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_TESTS=OFF \
      -D OPENCV_ENABLE_NONFREE=ON \
      -D CMAKE_CXX_FLAGS="-mcpu=cortex-a72 -O3 -ftree-vectorize" \
      -D CMAKE_C_FLAGS="-mcpu=cortex-a72 -O3 -ftree-vectorize" ..


# 4. Build and Install (This will take 1-2 hours)
#Parallel build using all CPU cores
make -j$(nproc)
sudo make install
sudo ldconfig
```

## 6. OpenVINO Runtime (ARM64 CPU Plugin)

OpenVINO is the preferred inference backend for VIGIA on ARM because it:

- Uses **JIT compilation**
- Leverages the **Arm Compute Library (ACL)**
- Optimizes kernels at runtime for Cortex-A72

Option A: Pre-compiled Archive (Fastest Setup)
Best if you want to get running quickly. Includes standard Arm Compute Library (ACL) optimizations.

Pros: Instant install.

Cons: Misses the latest KleidiAI micro-kernel tweaks for INT8.

### 6.1 Download

```bash
# Download the latest OpenVINO ARM archive from the official GitHub releases
# 1. Download the verified 2025.0 ARM64 archive
wget https://storage.openvinotoolkit.org/repositories/openvino/packages/2025.0/linux/openvino_toolkit_ubuntu20_2025.0.0.17942.1f68be9f594_arm64.tgz -O openvino_2025.tgz

# 2. Extract into the recommended /opt/intel path
sudo mkdir -p /opt/intel
sudo mkdir -p /opt/intel/openvino_2025
sudo tar -xf openvino_2025.tgz -C /opt/intel/openvino_2025 --strip-components=1

# 3. Run the dependency installer for Linux
# 1. Navigate to the dependencies folder
cd /opt/intel/openvino_2025/install_dependencies

# 2. Use sed to swap the OS check from 'ubuntu22.04' to 'debian13' inside the script
sudo sed -i "s/ubuntu22.04/debian13/g" ./install_openvino_dependencies.sh

# 3. Run the script again
sudo -E ./install_openvino_dependencies.sh

# 4. Finalize environment setup
echo "source /opt/intel/openvino_2025/setupvars.sh" >> ~/.bashrc
source ~/.bashrc

```
Option B: Build from Source with KleidiAI (Max Performance)
Highly recommended for INT8 models. This links OpenVINO directly to KleidiAI micro-kernels, which are optimized for quantized matrix multiplication on Cortex-A72.

```bash
# 1. Clone KleidiAI
git clone https://gitlab.arm.com/kleidi/kleidiai.git
cd kleidiai
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel $(nproc)
export KLEIDIAI_DIR=$(pwd)/build

# 2. Build OpenVINO
cd ..
git clone --recursive https://github.com/openvinotoolkit/openvino.git
mkdir openvino/build && cd openvino/build
cmake -DENABLE_KLEIDIAI=ON -DKleidiai_DIR=$KLEIDIAI_DIR \
      -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

---


## 7. Build VIGIA

```bash
git clone https://github.com/<your-org>/vigia.git
cd vigia

# 1. Initialize OpenVINO paths (Critical for CMake to find OpenVINO)
source /opt/intel/openvino_2025/setupvars.sh

# 2. Create and enter build directory
mkdir build && cd build

# 3. Configure the project
# This step checks for your KleidiCV-linked OpenCV and OpenVINO
cmake ..

# 4. Compile using all 4 CPU cores
# Keep an eye on heat; use -j2 if the Pi throttles
make -j$(nproc)

# 5. Run the visual test (assuming hazard.mp4 is present)
./system_visual_test --video hazard.mp4
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

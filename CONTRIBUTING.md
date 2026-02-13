# Contributing to VIGIA

Thank you for your interest in contributing to **VIGIA** — a real-time autonomous pothole detection system designed for edge deployment on Raspberry Pi 4.

This guide covers the development environment setup, build instructions, and performance tuning required to contribute effectively. The stack follows a **hardware-software co-design** philosophy, maximizing throughput from the Pi's ARM Cortex-A72 cores without GPUs, cloud inference, or external accelerators.

> **Before you begin:** Please read the project [README.md](README.md) for an architectural overview and the [LICENSE](LICENSE) for usage terms.

---

## Table of Contents

1. [Target Platform](#1-target-platform)
2. [OS Foundation](#2-os-foundation)
3. [Memory Configuration](#3-memory-configuration)
4. [System Dependencies](#4-system-dependencies)
5. [Arm KleidiAI & KleidiCV](#5-arm-kleidiai--kleidicv)
6. [OpenCV 4.x with KleidiCV](#6-opencv-4x-with-kleidicv)
7. [OpenVINO Runtime (ARM64)](#7-openvino-runtime-arm64)
8. [Building VIGIA](#8-building-vigia)
9. [Performance Tuning](#9-performance-tuning)
10. [Validation](#10-validation)
11. [Stack Rationale](#11-stack-rationale)

---

## 1. Target Platform

| Component      | Specification                              |
|----------------|--------------------------------------------|
| **Board**      | Raspberry Pi 4B (8 GB strongly recommended) |
| **OS**         | Raspberry Pi OS Lite (64-bit)              |
| **CPU**        | ARM Cortex-A72 (ARMv8-A), 4 cores         |
| **Cooling**    | Active cooling / heatsink required         |
| **Camera**     | USB webcam or Pi Camera Module (V2 / V3)   |

---

## 2. OS Foundation

> **Important:** A minimal OS image is critical for real-time performance. A desktop environment introduces background processes that compete for CPU cycles and increase scheduling jitter.

### 2.1 Flash the OS

Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) and select **Raspberry Pi OS Lite (64-bit)** — no desktop environment.

### 2.2 Initial System Configuration

```bash
sudo raspi-config
```

Configure the following:

| Setting             | Path                                                            |
|---------------------|-----------------------------------------------------------------|
| CPU Governor        | Performance Options → CPU Governor → `performance`              |
| Expand Filesystem   | Advanced Options → Expand Filesystem                            |
| Enable Camera       | Interface Options → Enable Camera *(if using Pi Camera)*        |

Reboot after applying changes.

### 2.3 CPU Scaling Governor

By default, the Pi 4 uses the `ondemand` governor, which dynamically scales CPU frequency to conserve power. For real-time inference this introduces latency spikes as the CPU transitions between frequency states.

**Fix:** Lock the CPU at its maximum clock speed (1.5 GHz / 1.8 GHz):

```bash
# Set to performance mode (lasts until next reboot)
echo performance | sudo tee /sys/devices/system/cpu/cpufreq/policy0/scaling_governor

# Check if it worked
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
```

### 2.4 Remove Unnecessary Packages

Strip background services that consume RAM and CPU cycles:

```bash
sudo apt purge wolfram-engine libreoffice* -y
sudo apt autoremove -y
```

---

## 3. Memory Configuration

> **Required for on-device builds.** Compiling OpenCV and OpenVINO on the Pi demands significantly more swap than the default 100 MB.

Increase swap to 2 GB:

```bash
# 2. Increase Swap Space (Required for compiling OpenCV/OpenVINO on Pi 4)
# Default 100MB is too small; increasing to 2GB to prevent 'Out of Memory' crashes
# Increase Swap to 2GB for Debian Trixie (Modern rpi-swap service)
sudo mkdir -p /etc/rpi/swap.conf.d/
echo -e "[File]\nFixedSizeMiB=2048" | sudo tee /etc/rpi/swap.conf.d/80-use-swapfile.conf
sudo reboot
```

> **Tip:** If builds still fail due to OOM, increase `CONF_SWAPSIZE` to `8192`.

---

## 4. System Dependencies

### 4.1 Synchronize System Clock

```bash
# 1. Fix System Time (Critical for GPG signatures and repository access)
# Manually set the clock to current time to avoid 'Not live until' errors
sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z"
```

### 4.2 Update & Install Build Toolchain

```bash
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

## 5. Arm KleidiAI & KleidiCV

KleidiAI provides highly optimized NEON SIMD micro-kernels for AI workloads on ARM. KleidiCV extends this to image preprocessing (resize, filter, color conversion) and is linked into OpenCV as a hardware abstraction layer (HAL).

### 5.1 Build and Install KleidiCV

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

## 6. OpenCV 4.x with KleidiCV

OpenCV must be built from source with the `WITH_KLEIDICV` flag to utilize the NEON-optimized HAL kernels installed above. The `KLEIDICV_DIR` variable points OpenCV to the HAL bridge files.

### 6.1 Clone Repositories

```bash
# 1. Clone both main and contrib modules (contrib contains extra optimization paths)
# Navigate to your home or workspace
cd ~
sudo apt update
sudo apt install -y python3-dev python3-numpy python3-setuptools
git clone --branch 4.x https://github.com/opencv/opencv.git
git clone --branch 4.x https://github.com/opencv/opencv_contrib.git
```

### 6.2 Configure and Build

```bash
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

> **Note:** The build takes approximately 1–2 hours on a Pi 4 with active cooling. Ensure adequate swap space (see [Section 3](#3-memory-configuration)).

---

## 7. OpenVINO Runtime (ARM64)

OpenVINO is the preferred inference backend for VIGIA on ARM. It provides:

- **JIT compilation** of model graphs at load time
- **Arm Compute Library (ACL)** integration for NEON-optimized operators
- **Runtime kernel selection** tuned for Cortex-A72 microarchitecture

Two installation paths are available:

### Option A: Pre-compiled Archive (Recommended for Quick Setup)

Fastest path to a working environment. Includes standard ACL optimizations.

| Pros               | Cons                                        |
|---------------------|---------------------------------------------|
| Instant install     | May lack latest KleidiAI INT8 micro-kernels |

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

### Option B: Build from Source with KleidiAI (Maximum Performance)

Recommended for INT8 quantized models. This links OpenVINO directly to KleidiAI micro-kernels optimized for quantized matrix multiplication on Cortex-A72.

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

## 8. Building VIGIA

```bash
cd ~
git clone https://github.com/BlueWaves-afk/vigia-raspi.git
cd vigia-raspi

# Sourcing OpenVINO 2025 setup script
source /opt/intel/openvino_2025/setupvars.sh

# 2. Create and enter build directory
# Create a dedicated build directory
mkdir -p build && cd build

# Configure the project with CMake
# This will check for your KleidiCV-optimized OpenCV and OpenVINO 2025
cmake ..

# Compile the visual test specifically using all 4 cores
make system_visual_test -j$(nproc)
# Set CPU governor to maximum performance
echo performance | sudo tee /sys/devices/system/cpu/cpufreq/policy0/scaling_governor

# Run the test (assuming hazard.mp4 and models/ are in your project root)
./system_visual_test
```

---

## 9. Performance Tuning

### 9.1 Overclock (Optional)

> ⚠️ **Warning:** Overclocking requires adequate cooling and may void your warranty. Recommended for advanced users only.

Edit `/boot/config.txt`:

```text
over_voltage=6
arm_freq=2000
gpu_freq=750
```

### 9.2 GPU Memory Allocation

Allocate sufficient GPU memory for dashboard rendering:

```bash
sudo raspi-config
# Performance Options → GPU Memory → 256
```

### 9.3 Thread Affinity

VIGIA's `Coordinator` implements core-affinity scheduling to isolate workloads and prevent resource contention:

| Thread               | Core   | Rationale                      |
|----------------------|--------|--------------------------------|
| Capture              | Core 0 | Isolates camera I/O            |
| Coordinator          | Core 0 | Manages frame dispatch         |
| Perception (YOLO)    | Core 1 | Dedicated inference core       |
| Analytical (MiDaS)   | Core 2 | Dedicated depth analysis core  |
| Fusion               | Core 3 | Risk index computation         |

This pinning strategy prevents camera I/O from competing with inference workloads.

---

## 10. Validation

### Camera Input

```bash
./system_visual_test --camera 0 --fps 15
```

### Video Input

```bash
./system_visual_test --video road.mp4
```

**Expected console output indicators:**

- ✅ Stable FPS within target range
- ✅ CPU temperature below thermal throttle threshold
- ✅ Adaptive stride adjustments logged at runtime

> **Thermal note:** If the CPU exceeds ~75 °C, VIGIA automatically adjusts inference cadence to maintain real-time behavior.

---

## 11. Stack Rationale

| Framework        | ARM CPU Efficiency | Notes                                   |
|------------------|--------------------|-----------------------------------------|
| **OpenVINO**     | ⭐⭐⭐⭐⭐             | JIT + ACL + NEON — best determinism     |
| TFLite           | ⭐⭐⭐⭐              | Good, but less runtime optimization     |
| ONNX Runtime     | ⭐⭐⭐               | Limited ARM-specific tuning             |

This stack prioritizes **determinism**, **thermal stability**, and **real-time guarantees** over raw throughput — the defining requirements for safety-critical edge inference on resource-constrained hardware.

---

## Code of Conduct

All contributors are expected to follow respectful, professional communication. Harassment, discrimination, and disruptive behavior will not be tolerated.

## Questions?

If you encounter issues with the build process or have questions about the architecture, please [open an issue](https://github.com/BlueWaves-afk/vigia-raspi/issues) on the repository.
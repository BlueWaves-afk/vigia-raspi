# 🛠️ INSTALL.md: Raspberry Pi 4 Setup Guide

This guide details the steps to deploy the **VIGIA** H-HMAS on a Raspberry Pi 4 (8GB recommended) running **64-bit Raspberry Pi OS**.

## 1. Prerequisites

* **Hardware:** Raspberry Pi 4B (with active cooling/heatsink for MiDaS execution).
* **OS:** Raspberry Pi OS (64-bit) Bullseye or Bookworm.
* **Camera:** USB Webcam or Raspberry Pi Camera Module (V2/V3).

## 2. Install System Dependencies

Open a terminal on your Pi and update the system:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git pkg-config \
    libjpeg-dev libtiff5-dev libjasper-dev libpng-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libgtk-3-dev \
    libatlas-base-dev gfortran python3-dev

```

## 3. Install OpenVINO (ARM64)

Since the standard OpenVINO installer is for x86, we use the **Archive Distribution** for ARM64:

```bash
# Create directory
mkdir ~/openvino && cd ~/openvino

# Download the ARM64 runtime (Example for 2024.x)
wget https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.0/linux/l_openvino_toolkit_debian11_2024.0.0.14509.34caeefd9d8_arm64.tgz

# Extract and set up variables
tar -xf l_openvino_toolkit_*.tgz
source ~/openvino/setupvars.sh

```

> **Pro Tip:** Add `source ~/openvino/setupvars.sh` to your `~/.bashrc` to ensure the environment is ready on every boot.

## 4. Compile VIGIA for ARM

VIGIA uses C++17 and thread pinning optimized for the Pi 4's quad-core Broadcom BCM2711.

```bash
cd ~/vigia
mkdir build && cd build

# Using CMake for robust linking on Pi
cmake .. \
    -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4 \
    -DOpenVINO_DIR=~/openvino/runtime/cmake

make -j4

```

---

## 🚀 Raspberry Pi Performance Tuning

To achieve the best results with the **Adaptive Control** logic in `coordinator.cpp`, apply these Pi-specific optimizations:

### 1. Enable Overclocking (Optional but Recommended)

Edit `/boot/config.txt` to boost the CPU clock for faster MiDaS processing:

```text
over_voltage=6
arm_freq=2000
gpu_freq=750

```

### 2. Thread Pinning Configuration

Your `coordinator.cpp` already pins the Main loop to Core 1 and Capture to Core 0. On the Pi 4, this ensures the heavy YOLO/MiDaS inference doesn't fight the camera driver for CPU cycles.

### 3. Memory Allocation

If using an 8GB Pi, ensure the GPU memory is sufficient for the display dashboard:

```bash
sudo raspi-config # Performance Options -> GPU Memory -> Set to 256

```

---

## 🧪 Quick Test on Pi

Once compiled, run the visual test with the camera:

```bash
./vigia_system --camera 0 --fps 15

```

*Observe the **Insights Dashboard**: If the CPU temperature exceeds 75°C, you will see the `MiDaS stride` automatically increase to maintain a fluid 15 FPS frame rate.*


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
10. [Remote Development with VNC](#10-remote-development-with-vnc)
11. [Validation](#11-validation)
12. [Stack Rationale](#12-stack-rationale)
13. [Known Issues & Solutions](#13-known-issues--solutions)

---

## 1. Target Platform

| Component  | Specification                                           |
|------------|---------------------------------------------------------|
| **Board**  | Raspberry Pi 4B (8 GB strongly recommended)             |
| **OS**     | Raspberry Pi OS Lite (64-bit), Debian 12 (Bookworm)     |
| **CPU**    | ARM Cortex-A72 (ARMv8-A), 4 cores                       |
| **Cooling**| Active cooling / heatsink required                      |
| **Camera** | USB webcam or Pi Camera Module (V2 / V3)                |

<img width="1200" height="1600" alt="image" src="https://github.com/user-attachments/assets/fce7fb6b-8b3d-47c0-95e1-278481f6c9cf" />

---

## 2. OS Foundation

> **Important:** A minimal OS image is critical for real-time performance. A desktop environment introduces background processes that compete for CPU cycles and increase scheduling jitter. Use the 64-bit Debian 12 image, as it is the most suitable base for this project.

### 2.1 Flash the OS

Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) and select **Raspberry Pi OS Lite (64-bit)** — no desktop environment.

### 2.2 Initial System Configuration

```bash
sudo raspi-config
```

Configure the following:

| Setting           | Path                                                       |
|-------------------|------------------------------------------------------------|
| CPU Governor      | Performance Options → CPU Governor → `performance`         |
| Expand Filesystem | Advanced Options → Expand Filesystem                       |
| Enable Camera     | Interface Options → Enable Camera *(if using Pi Camera)*   |

Reboot after applying changes.

### 2.3 CPU Scaling Governor

By default, the Pi 4 uses the `ondemand` governor, which dynamically scales CPU frequency to conserve power. For real-time inference, this introduces latency spikes as the CPU transitions between frequency states.

**Fix:** Lock the CPU at its maximum clock speed (1.5 GHz / 1.8 GHz):

```bash
# Set to performance mode (persists until next reboot)
echo performance | sudo tee /sys/devices/system/cpu/cpufreq/policy0/scaling_governor

# Verify the change
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
# Increase swap to 2 GB (required to prevent out-of-memory crashes during OpenCV/OpenVINO compilation)
# Default 100 MB is insufficient for this workload
sudo mkdir -p /etc/rpi/swap.conf.d/
echo -e "[File]\nFixedSizeMiB=2048" | sudo tee /etc/rpi/swap.conf.d/80-use-swapfile.conf
sudo reboot
```

> **Tip:** If builds still fail due to OOM errors, increase `FixedSizeMiB` to `8192`.

---

## 4. System Dependencies

### 4.1 Synchronize System Clock

```bash
# Manually synchronize the clock to avoid GPG signature errors ("Not live until")
sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z"
```

### 4.2 Update & Install Build Toolchain

```bash
# Refresh repositories and perform a full system upgrade
# Use 'full-upgrade' on Debian Bookworm to properly resolve dependency changes
sudo apt update && sudo apt full-upgrade -y

# Install build tools and high-performance libraries
# Includes libtbb-dev for threading and libopenblas for ARM-optimized math
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
# Clone the repository
git clone https://gitlab.arm.com/kleidi/kleidicv.git
cd kleidicv

# Configure with Raspberry Pi 4-specific optimizations
# -mcpu=cortex-a72 targets the Pi 4's CPU cores directly
cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS="-mcpu=cortex-a72" \
      -DCMAKE_C_FLAGS="-mcpu=cortex-a72"

# Build using all 4 cores
cmake --build build --parallel $(nproc)

# Install headers and libraries to system paths
sudo cmake --install build
```

---

## 6. OpenCV 4.x with KleidiCV

OpenCV must be built from source with the `WITH_KLEIDICV` flag to utilize the NEON-optimized HAL kernels installed above. The `KLEIDICV_DIR` variable points OpenCV to the HAL bridge files.

### 6.1 Clone Repositories

```bash
cd ~
sudo apt update
sudo apt install -y python3-dev python3-numpy python3-setuptools

# Clone both main and contrib modules (contrib contains additional optimization paths)
git clone --branch 4.x https://github.com/opencv/opencv.git
git clone --branch 4.x https://github.com/opencv/opencv_contrib.git
```

### 6.2 Configure and Build

```bash
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

# Build and install — this will take approximately 1–2 hours
make -j$(nproc)
sudo make install
sudo ldconfig
```

> **Note:** The build takes approximately 1–2 hours on a Pi 4 with active cooling. Ensure adequate swap space is configured (see [Section 3](#3-memory-configuration)).

---

## 7. OpenVINO Runtime (ARM64)

OpenVINO is the preferred inference backend for VIGIA on ARM. It provides:

- **JIT compilation** of model graphs at load time
- **KleidiAI integration** for NEON-optimized GEMM kernels on Cortex-A72
- **Runtime kernel selection** tuned for the ARMv8-A microarchitecture
- **Async inference API** with pre-allocated tensor buffers

### Option A: Pre-compiled Archive (Not Recommended)

> **Warning — SIGBUS on Raspberry Pi OS.**
>
> As of early 2026, there is **no pre-compiled OpenVINO archive** whose memory alignment assumptions match Raspberry Pi OS (Debian 12 Bookworm, aarch64). The official archives target Ubuntu 20.04/22.04 ARM64, which uses different `mmap` page alignment and library versioning. On Pi OS this manifests as a **SIGBUS (Bus error)** during model compilation — specifically when the ARM CPU plugin attempts unaligned vector loads on weight tensors that were memory-mapped from the `.bin` file.
>
> This was encountered during development: the pre-compiled plugin crashed deterministically at `ov::Core::compile_model()` with signal 7 (SIGBUS), even on a clean Debian 12 installation. Disabling `mmap` via `ov::enable_mmap(false)` partially mitigated the crash, but inference remained unreliable and significantly slower due to fallback scalar code paths.
>
> **Use Option B (build from source) instead.** It is the only path that produces a working, optimized binary for Raspberry Pi OS.

If you still want to attempt the pre-compiled route (e.g., for a quick feasibility check on Ubuntu ARM64), the commands are:

```bash
# Download an ARM64 archive (may not work on Pi OS — see warning above)
wget https://storage.openvinotoolkit.org/repositories/openvino/packages/2025.0/linux/openvino_toolkit_ubuntu20_2025.0.0.17942.1f68be9f594_arm64.tgz -O openvino_2025.tgz

sudo mkdir -p /opt/intel/openvino_2025
sudo tar -xf openvino_2025.tgz -C /opt/intel/openvino_2025 --strip-components=1

echo "source /opt/intel/openvino_2025/setupvars.sh" >> ~/.bashrc
source ~/.bashrc
```

### Option B: Build from Source with KleidiAI (Recommended)

Building OpenVINO from source is the **only reliable path** on Raspberry Pi OS. This compiles the ARM CPU plugin natively, ensuring correct memory alignment, and links KleidiAI micro-kernels directly into the plugin for optimized GEMM operations on Cortex-A72.

> **Note on KleidiAI:** You do **not** need to clone or build KleidiAI separately. OpenVINO's build system fetches and compiles KleidiAI automatically when `-DENABLE_KLEIDIAI=ON` is set. The KleidiAI source is bundled as a dependency inside the OpenVINO repository.

#### 7.1 Install Build Prerequisites

```bash
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  pkg-config \
  libprotobuf-dev \
  protobuf-compiler \
  libtbb-dev \
  python3-dev \
  python3-pip \
  python3-venv \
  python3-setuptools \
  wget \
  scons
```

#### 7.2 Clone OpenVINO

```bash
cd ~
git clone --recurse-submodules https://github.com/openvinotoolkit/openvino.git
cd openvino

# Check out a known-good release tag
git checkout 2025.4.2
git submodule update --init --recursive
```

#### 7.3 Configure the Build

```bash
mkdir build && cd build

cmake -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_KLEIDIAI=ON \
      -DCMAKE_INSTALL_PREFIX=/opt/intel/openvino_2025 \
      -DENABLE_INTEL_CPU=OFF \
      -DENABLE_INTEL_GPU=OFF \
      -DENABLE_INTEL_NPU=OFF \
      -DENABLE_SAMPLES=OFF \
      -DENABLE_TESTS=OFF \
      -DENABLE_PYTHON=OFF \
      -DENABLE_WHEEL=OFF \
      -DTHREADING=TBB \
      -DCMAKE_CXX_FLAGS="-mcpu=cortex-a72" \
      -DCMAKE_C_FLAGS="-mcpu=cortex-a72" \
      ..
```

**Flag rationale:**

| Flag | Purpose |
|------|---------|
| `ENABLE_KLEIDIAI=ON` | Links KleidiAI NEON micro-kernels for INT8/FP32 GEMM |
| `ENABLE_INTEL_*=OFF` | Disables x86-only plugins (GPU, NPU) that do not exist on ARM |
| `THREADING=TBB` | Uses the system TBB library for multi-core dispatch |
| `-mcpu=cortex-a72` | Generates code tuned specifically for the Pi 4's core |

#### 7.4 Build and Install

```bash
# Build using all 4 cores — takes approximately 2–3 hours with active cooling
# Ensure swap is at least 2 GB (see Section 3)
make -j$(nproc)

# Install to /opt/intel/openvino_2025
sudo make install
```

#### 7.5 Environment Setup

```bash
# Add OpenVINO libraries to the linker search path
echo "/opt/intel/openvino_2025/runtime/lib/aarch64" | sudo tee /etc/ld.so.conf.d/openvino.conf
sudo ldconfig

# Add to your shell profile for CMake discovery
echo 'export OpenVINO_DIR=/opt/intel/openvino_2025/runtime/cmake' >> ~/.bashrc
source ~/.bashrc
```

#### 7.6 Verify the Build

```bash
# Confirm the ARM CPU plugin exists
ls /opt/intel/openvino_2025/runtime/lib/aarch64/libopenvino_arm_cpu_plugin.so

# Verify KleidiAI is linked into the plugin
strings /opt/intel/openvino_2025/runtime/lib/aarch64/libopenvino_arm_cpu_plugin.so | grep -i kleidiai
# Expected output: kleidiai, MatMulKleidiAIExecutor, etc.
```

---

## 8. Building VIGIA

```bash
cd ~
git clone https://github.com/BlueWaves-afk/vigia-raspi.git
cd vigia-raspi

# Create and enter the build directory
mkdir -p build && cd build

# Configure the project with CMake
# If OpenVINO_DIR is set in your shell (see Section 7.5), CMake finds it automatically.
# Otherwise, pass it explicitly:
cmake -DOpenVINO_DIR=/opt/intel/openvino_2025/runtime/cmake ..

# Compile the visual test using all 4 cores
make system_visual_test -j$(nproc)

# Lock the CPU governor to maximum performance
echo performance | sudo tee /sys/devices/system/cpu/cpufreq/policy0/scaling_governor

# Run the test (assumes hazard.mp4 and models/ are in your project root)
./system_visual_test --video ../hazard.mp4
```

---

## 9. Performance Tuning

### 9.1 Overclock (Optional)

> **Warning:** Overclocking requires adequate cooling and may void your warranty. Recommended for advanced users only.

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

VIGIA **automatically pins threads to specific cores** at startup — no manual configuration is required. The pinning is implemented in two locations:

- **`Coordinator::captureLoop()`** and **`Coordinator::processLoop()`** in `src/coordinator.cpp` — each pins itself via `pthread_setaffinity_np` at function entry. `Coordinator::start()` additionally calls `pinThread()` on both threads after launch.
- **`main()`** in `tests/system_visual_test.cpp` — pins the UI/dashboard thread via `pinCurrentThreadToCore(3)`.

All pinning is guarded by `#if defined(__linux__) && defined(__aarch64__)` so it compiles as a no-op on macOS.

#### Core Assignment Map

| Thread                   | Core   | Rationale                                                   |
|--------------------------|--------|-------------------------------------------------------------|
| Capture (`captureLoop`)  | Core 0 | Dedicated to camera I/O and frame decode                    |
| Process (`processLoop`)  | Core 1 | Runs YOLO + MiDaS inference and fusion sequentially         |
| TBB workers (OpenVINO)   | Core 2 | Left unassigned — TBB scheduler uses available cores        |
| UI / Dashboard (`main`)  | Core 3 | Rendering, dashboard compositing, and keyboard input        |

> **Why 4 threads on 4 cores?** The Pi 4 has four identical Cortex-A72 cores. Pinning eliminates cross-core migration, which flushes the L1/L2 cache and adds approximately 1–2 ms of jitter per migration. Core 2 is intentionally left unpinned so that OpenVINO's internal TBB threads can use it for parallel operator execution without competing with the capture or UI threads.

This pinning strategy ensures deterministic scheduling and prevents camera I/O from competing with inference workloads.

---

## 10. Remote Development with VNC

VIGIA's visual tests (`system_visual_test`) require a display. When developing remotely over SSH, a VNC server is needed to view the OpenCV GUI output.

### Why TigerVNC over RealVNC?

| Aspect           | TigerVNC                     | RealVNC                          |
|------------------|------------------------------|----------------------------------|
| Stability        | Excellent                    | Frequent glitches, dropped frames |
| Latency          | Low                          | Variable                         |
| Resource usage   | Lightweight                  | Heavier background services      |
| Configuration    | Simple CLI                   | GUI-centric, complex             |
| Open source      | Yes                          | Proprietary (free tier limited)  |

RealVNC (pre-installed on Raspberry Pi OS) frequently exhibits visual artifacts, connection drops, and high latency when streaming OpenCV windows. TigerVNC provides a more reliable experience for real-time visualization.

### Install TigerVNC (via TightVNC)

```bash
# Remove RealVNC if present
sudo apt remove -y --purge realvnc-vnc-server realvnc-vnc-viewer
sudo apt autoremove -y

# Clean up leftover configuration
rm -rf ~/.vnc

# Install TightVNC
sudo apt update
sudo apt install -y tightvncserver

# Set VNC password (first time only)
vncpasswd
```

### Start VNC Server on Pi

```bash
# Start VNC on display :1 with 720p resolution
vncserver :1 -geometry 1280x720 -depth 24

# Verify the server is running
vncserver -list

# Expected output:
#   TigerVNC server sessions:
#   X DISPLAY #     PROCESS ID
#   :1              12345
```

### Connect from macOS

**Option 1 — Terminal:**
```bash
open vnc://raspi4B.local:5901
```

**Option 2 — Finder:**
1. Press `Cmd + K` (or Finder → Go → Connect to Server)
2. Enter: `vnc://raspi4B.local:5901`
3. Click **Connect**
4. Enter the VNC password set earlier

> **Note:** Display `:1` maps to port `5901`, `:2` to `5902`, and so on.

### Run VIGIA Tests over VNC

Once connected via VNC, open a terminal in the VNC session and run:

```bash
cd ~/vigia-raspi/build

# Standard dashboard view
./system_visual_test --video hazard.mp4

# Fullscreen mode (press Q to quit)
./system_visual_test -F --video hazard.mp4
```

The OpenCV window will appear in the VNC session, not on your local display.

### VNC Management Commands

```bash
# List running VNC sessions
vncserver -list

# Kill a specific session
vncserver -kill :1

# Kill all sessions
vncserver -kill :*

# Check what processes are using VNC ports
ps aux | grep vnc
```

### Complete SSH + VNC Workflow

```bash
# 1. SSH into the Pi
ssh vigiasense@raspi4B.local

# 2. Start the VNC server (if not already running)
vncserver :1 -geometry 1280x720 -depth 24

# 3. Open a new terminal on your Mac and connect via VNC
open vnc://raspi4B.local:5901

# 4. In the VNC session, run visual tests
cd ~/vigia-raspi/build
./system_visual_test -F --video hazard.mp4

# 5. When finished, stop VNC to free resources
vncserver -kill :1
```

### Troubleshooting

| Issue                              | Solution                                                |
|------------------------------------|---------------------------------------------------------|
| "Connection refused" on Mac        | Verify VNC server is running: `vncserver -list`         |
| Black screen in VNC                | Start a window manager: `startlxde &`, or run your test |
| Multiple VNC sessions running      | Kill all: `vncserver -kill :*`, then start fresh        |
| Laggy display                      | Reduce resolution: `-geometry 1024x600`                 |
| Authentication failure             | Re-run `vncpasswd` to reset the password                |

---

## 11. Validation

### Camera Input

```bash
./system_visual_test --camera 0 --fps 15
```

### Video Input

```bash
./system_visual_test --video road.mp4
```

<img width="1440" height="867" alt="image" src="https://github.com/user-attachments/assets/770f6d3d-62bc-4e51-8e25-66b1935a8685" />

**Expected console output indicators:**

- Stable FPS within the target range
- CPU temperature below the thermal throttle threshold
- Adaptive stride adjustments logged at runtime

> **Thermal note:** If the CPU exceeds approximately 75 °C, VIGIA automatically adjusts inference cadence to maintain real-time behavior.

---

## 12. Stack Rationale

| Framework    | ARM CPU Efficiency | Notes                                          |
|--------------|--------------------|------------------------------------------------|
| **OpenVINO** | Excellent          | JIT + KleidiAI + NEON — best determinism       |
| TFLite       | Good               | Good performance, but less runtime optimization |
| ONNX Runtime | Adequate           | Limited ARM-specific tuning                    |

This stack prioritizes **determinism**, **thermal stability**, and **real-time guarantees** over raw throughput — the defining requirements for safety-critical edge inference on resource-constrained hardware.

---

## 13. Known Issues & Solutions

This section documents the engineering challenges encountered during development and the solutions that were implemented.

### Hardware & OS Configuration

**Issue: GPU Disabled in Headless Operation**
The Pi 4 disables its VideoCore GPU when no monitor is attached, forcing the CPU to handle all desktop rendering and causing severe VNC lag.

*Solution:* Force a virtual monitor by appending `video=HDMI-A-1:1280x720@60D` to `/boot/firmware/cmdline.txt`. This keeps the GPU active for hardware-accelerated rendering.

**Issue: VNC Clipboard Synchronization Failure**
Switching from the Wayland compositor to X11 resolved clipboard sync issues between macOS and the Pi.

*Solution:* Use `raspi-config` to set the backend to X11 (System → Boot → Desktop Autologin; Advanced → Wayland → X11), then install and run `autocutsel -fork` to bridge the clipboard.

### Inference Stability

**Issue: SIGBUS Crash During Model Compilation**
The pre-compiled OpenVINO binary crashed at `ov::Core::compile_model()` with signal 7 (SIGBUS) due to memory alignment mismatches between the Ubuntu-targeted archive and Raspberry Pi OS.

*Solution:* Build OpenVINO 2025.4.2 from source with `-DENABLE_KLEIDIAI=ON`. This ensures 16-byte alignment guards are active and all SIMD paths are compiled natively for the Cortex-A72.

**Issue: SD Card Memory Faults (`mmap`)**
Memory-mapping model weights from an SD card caused intermittent hangs and crashes.

*Solution:* Disable memory mapping explicitly via `ov::enable_mmap(false)`, forcing the model weights to be loaded entirely into RAM.

**Issue: Zero Detections (Color Space Mismatch)**
The C++ pipeline produced no detections because OpenCV's capture returns BGR frames, while the model expects RGB input.

*Solution:* Rewrote the preprocessing loop using ARM NEON SIMD de-interleaving to convert BGR to RGB and transpose the tensor to NCHW in a single optimized pass.

### Quantization & Model Accuracy

**Issue: Recall Loss After INT8 Quantization**
Initial INT8 quantization caused the model to miss small or distant potholes that were clearly visible in the FP32 version.

*Solution:* Applied Accuracy-Aware Quantization and Quantization-Aware Training (QAT). A significantly lower confidence threshold (0.008) was used to account for the compressed score range produced by 8-bit arithmetic, with the fusion engine filtering the resulting noise.

**Issue: Bounding Box Merging After Quantization**
Rounding errors caused the model to merge adjacent potholes into a single oversized bounding box.

*Solution:* Implemented Letterbox preprocessing to maintain exact 1:1 aspect ratios, and added a custom post-NMS cleanup step with an IoU threshold of 0.45 for the INT8 pipeline.

**Issue: MiDaS Depth Model Failure at INT8**
Quantizing the MiDaS v2.1 depth estimation model to INT8 produced a uniformly black depth map, rendering it non-functional.

*Solution:* Determined that MiDaS requires higher dynamic range for geometric analysis. A hybrid-precision pipeline was adopted: MiDaS runs in FP32/FP16 while only the YOLO backbone is quantized.

### Throughput & Latency

**Issue: Inference Precision Conflict (2–4 FPS)**
The system was implicitly up-converting the INT8 model back to FP32 at runtime, negating all quantization speed gains.

*Solution:* Set `ov::hint::inference_precision` to dynamic (undefined), allowing the model to remain in its native INT8 execution path.

**Issue: Parallelism Overhead (3 FPS vs. 10 FPS)**
The multi-threaded architecture performed significantly worse than single-threaded benchmarks due to contention on the Pi's shared L2 cache.

*Solution:* Replaced the parallel pipeline with a sequential processing loop to maximize cache locality for the active model. Display sampling was introduced to skip unnecessary UI renders without reducing the background inference rate.

**Issue: UI Back-Pressure Blocking Inference**
The `cv::waitKey(1)` call blocked the entire AI pipeline whenever the display server lagged.

*Solution:* Decoupled visualization into a separate thread, allowing the inference core to continue processing frames regardless of monitor refresh rate.

### Asynchronous Pipeline

**Issue: Pipeline Stall on Skipped MiDaS Frames**
The `InstrumentationBus` stalled indefinitely when waiting for depth data from MiDaS frames that the Adaptive Control logic had deliberately skipped.

*Solution:* Replaced the blocking fusion model with a YOLO-first transparency model. YOLO detections are published immediately; depth and temporal enrichment data are treated as optional layers.

**Issue: Thread State Isolation**
Using `thread_local` variables to track frame indices caused the Perception and Analytical threads to maintain independent, unsynchronized counters, producing mismatched telemetry.

*Solution:* Replaced `thread_local` with `std::atomic<std::uint64_t>` to create a shared, thread-safe global frame index, ensuring all agents report against a consistent timeline.

---

## Code of Conduct

All contributors are expected to maintain respectful, professional communication. Harassment, discrimination, and disruptive behavior will not be tolerated.

## Questions

If you encounter issues with the build process or have questions about the architecture, please [open an issue](https://github.com/BlueWaves-afk/vigia-raspi/issues) on the repository.
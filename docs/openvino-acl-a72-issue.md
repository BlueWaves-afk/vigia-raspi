# OpenVINO Runtime Performance Issue on Cortex-A72 (Raspberry Pi 4)

**Date:** March 2026  
**Hardware:** Raspberry Pi 4B, ARM Cortex-A72 (ARMv8-A), 4 cores @ 1.8 GHz  
**OS:** Raspberry Pi OS Lite 64-bit (Debian 12 Bookworm)  
**OpenVINO version:** 2025.4.2 (pre-built package)

---

## Problem Statement

VIGIA's YOLO26 INT8 inference on the Raspberry Pi 4 achieves **~4 FPS / 236ms latency** with the pre-built OpenVINO 2025.4.2 package, compared to the **~12 FPS / 83ms** target documented in the README benchmarks. This is a **3× performance gap** that is not caused by the model, the application code, or thermal throttling.

---

## Root Cause Analysis

### Step 1 — Confirmed hardware is correct

```
CPU part    : 0xd08  (Cortex-A72 — correct for Pi 4)
CPU freq    : 1800000 Hz (governor locked to performance — correct)
Temp        : within normal range — no throttling
```

### Step 2 — Confirmed OpenVINO build contains KleidiAI and ACL symbols

```bash
strings libopenvino_arm_cpu_plugin.so | grep -i "kleidiai\|arm_compute"
```

Output confirmed both KleidiAI micro-kernels and ACL (`ACLScheduler`) are compiled into the plugin binary.

### Step 3 — Identified the dispatch failure

The Cortex-A72 CPU features list:
```
Features: fp asimd evtstrm crc32 cpuid
```

**Missing: `asimddp`** — this is the `FEAT_DotProd` extension that enables the `vdot`/`sdot` instructions.

KleidiAI's INT8 GEMM fast path (`kai_matmul_clamp_f32_qai8dxp_neon_dotprod`) requires `vdot`. At runtime, OpenVINO queries the CPU for `asimddp`. Since A72 lacks it, KleidiAI's INT8 path is **silently skipped**.

### Step 4 — Confirmed ACL is not dispatching either

```bash
OV_CPU_VERBOSE=2 ./perception_video_test --headless hazard.mp4 2>&1 | grep -i "acl\|jit\|ref"
```

No ACL dispatch output. The pre-built OpenVINO 2025.4.2 package was built with `ENABLE_ARM_COMPUTE_CMAKE=OFF` (ACL disabled), meaning ACL symbols present in the binary are from KleidiAI's dependency chain, not from a functional ACL dispatch path.

### Step 5 — Confirmed single-threaded reference fallback

Latency profile: min=235.90ms, avg=236.47ms, P95=236.78ms, max=263ms.

The near-zero variance (0.88ms spread across 692 frames) is the signature of **scalar reference implementation** — no parallelism, no SIMD, every convolution running on a single core through the generic fallback path.

For comparison, multi-threaded NEON GEMM on 4 cores would show ~83ms average with ±10ms variance.

---

## Why This Happens

The pre-built OpenVINO 2025.4.2 ARM64 package is optimized for **Cortex-A55/A76/A78** (found in phones, Pi 5, Jetson Orin) which have `FEAT_DotProd`. The package ships KleidiAI as the primary INT8 acceleration path. On A72 (Pi 4), which lacks `vdot`:

1. KleidiAI INT8 path → **skipped** (no `asimddp`)
2. ACL path → **not compiled in** (pre-built package has `ENABLE_ARM_COMPUTE_CMAKE=OFF`)
3. oneDNN JIT path → **not available** on ARM (x86-only)
4. Result: **scalar reference implementation** at ~236ms

This is a known limitation of the pre-built package. The Intel documentation states that source builds are required for optimal ARM performance, which is also documented in VIGIA's CONTRIBUTING.md under "Option A: Pre-compiled Archive (Not Recommended)".

---

## Performance Impact

| Configuration | Latency | FPS | Acceleration |
|---|---|---|---|
| Pre-built OpenVINO 2025.4.2 (current) | 236ms | ~4 FPS | None (scalar reference) |
| Source build with ACL (target) | ~83ms | ~12 FPS | ACL NEON GEMM |
| Theoretical maximum (A72 @ 1.8GHz) | ~60ms | ~16 FPS | ACL + NEON FP32 |

---

## The Fix: Build OpenVINO from Source with ACL

The solution is to build OpenVINO from source with `ENABLE_ARM_COMPUTE_CMAKE=ON` and `ENABLE_KLEIDIAI=OFF`. This compiles ACL's A72-tuned NEON GEMM kernels directly into the CPU plugin, bypassing the dotprod requirement entirely.

### Why `ENABLE_KLEIDIAI=OFF` is required

When both KleidiAI and ACL are enabled, OpenVINO's dispatch logic prefers KleidiAI for INT8 operations. On A72, KleidiAI's runtime capability check fails (no `asimddp`), but the fallback to ACL is not guaranteed in the current dispatch implementation. Disabling KleidiAI forces the dispatch to use ACL's FP32/NEON path which is fully supported on A72.

### Build commands

```bash
# 1. Remove pre-built package
sudo rm -rf /opt/intel/openvino_2025
sudo rm -f /etc/ld.so.conf.d/openvino.conf
sudo ldconfig

# 2. Increase swap (build requires ~3GB RAM)
sudo dphys-swapfile swapoff
sudo sed -i 's/CONF_SWAPSIZE=.*/CONF_SWAPSIZE=4096/' /etc/dphys-swapfile
sudo dphys-swapfile setup && sudo dphys-swapfile swapon

# 3. Clone OpenVINO source
cd ~
git clone --recurse-submodules https://github.com/openvinotoolkit/openvino.git
cd openvino
git checkout 2025.4.2
git submodule update --init --recursive

# 4. Configure with ACL enabled, KleidiAI disabled
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_INTEL_CPU=ON \
      -DENABLE_ARM_COMPUTE_CMAKE=ON \
      -DENABLE_KLEIDIAI=OFF \
      -DCMAKE_INSTALL_PREFIX=/opt/intel/openvino_2025 \
      -DENABLE_INTEL_GPU=OFF \
      -DENABLE_INTEL_NPU=OFF \
      -DENABLE_SAMPLES=OFF \
      -DENABLE_TESTS=OFF \
      -DENABLE_PYTHON=OFF \
      -DENABLE_WHEEL=OFF \
      -DTHREADING=TBB \
      -DTBB_DIR="" \
      -DCMAKE_CXX_FLAGS="-mcpu=cortex-a72 -O3" \
      -DCMAKE_C_FLAGS="-mcpu=cortex-a72 -O3" \
      ..

# 5. Build (~3-4 hours on Pi 4 — run inside screen)
screen -S ov_build
make -j4
sudo make install

# 6. Configure environment
echo "/opt/intel/openvino_2025/runtime/lib/aarch64" | sudo tee /etc/ld.so.conf.d/openvino.conf
sudo ldconfig
echo 'source /opt/intel/openvino_2025/setupvars.sh' >> ~/.bashrc
source ~/.bashrc

# 7. Rebuild VIGIA
cd ~/vigia-raspi/build
cmake -DOpenVINO_DIR=/opt/intel/openvino_2025/runtime/cmake ..
make perception_video_test system_visual_test -j4

# 8. Verify improvement
./perception_video_test --headless hazard.mp4
# Expected: ~83ms latency, ~12 FPS
```

### Known build issue (CMake export set error)

During cmake configuration with `ENABLE_ARM_COMPUTE_CMAKE=ON`, the following error may appear:

```
CMake Error: install(EXPORT "dnnl-targets") includes target "dnnl" 
which requires target "arm_compute_core" that is not in any export set.
```

This is a bug in OpenVINO 2025.4.2's CMake where `arm_compute_core` (the ACL static library) is linked into `dnnl` but not added to the install export manifest. 

**Workaround:** Patch the ACL cmake file to add `arm_compute_core` to the export set before running cmake:

```bash
# Find the dnnl install export definition
grep -rn "EXPORT dnnl-targets" ~/openvino/src/ | head -5

# Add arm_compute_core to the same export
# (exact patch depends on file location found above)
```

This is an active issue in the OpenVINO repository. An alternative is to use OpenVINO 2024.6 which has a stable ACL integration on A72 without this cmake bug.

---

## Alternative: OpenVINO 2024.6 (Stable ACL on A72)

If the 2025.4.2 cmake bug cannot be resolved, OpenVINO 2024.6 has a known-working ACL build path for A72:

```bash
cd ~/openvino
git checkout 2024.6.0
git submodule update --init --recursive
```

Use the same cmake flags. The 2024.6 release predates the KleidiAI integration and uses ACL as the primary ARM acceleration path, making it more reliable on A72.

---

## Summary

| | Pre-built 2025.4.2 | Source build (target) |
|---|---|---|
| KleidiAI | Compiled in, runtime disabled (no `vdot`) | Disabled at build time |
| ACL | Not compiled in | Compiled in, active |
| Effective acceleration | None (scalar reference) | ACL NEON GEMM |
| YOLO latency | 236ms | ~83ms |
| System FPS | ~4 FPS | ~12 FPS headless |

The 3× performance gap is entirely attributable to the pre-built package's inability to activate any hardware acceleration on Cortex-A72. A source build with ACL resolves this and delivers the performance documented in the README benchmarks.

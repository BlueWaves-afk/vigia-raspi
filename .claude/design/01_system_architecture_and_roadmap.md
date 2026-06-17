# VIGIA ADAS DePIN Edge Node
## Master System Architecture & Engineering Roadmap
**Document:** `01_system_architecture_and_roadmap.md`  
**Status:** PHASE 1 IN PROGRESS — Node source written, ROS2 building from source on Pi  
**Competition Target:** Samsung Solve for Tomorrow 2026  
**Classification:** Spec-Driven Development — No implementation before node contracts are signed off

## Implementation Status Log

| Date | Action | Notes |
|---|---|---|
| 2026-06-17 | Pi SSH access confirmed via Tailscale (100.114.1.98) | vigiasense@raspberrypi, Debian Trixie, kernel 6.12.75+rpt |
| 2026-06-17 | PREEMPT_RT kernel installed (`linux-image-6.12.73+deb13-rt-arm64`) | **NOT YET BOOTED** — Pi firmware requires manual config; deferred to physical access |
| 2026-06-17 | ROS2 Jazzy binary install failed | Ubuntu Noble `libpython3.12t64` not available on Debian Trixie (Python 3.13 only) |
| 2026-06-17 | ROS2 Jazzy from-source build in progress | `~/ros2_jazzy/` on Pi — `rclcpp`, `sensor_msgs`, `std_msgs` deps targeted |
| 2026-06-17 | All 6 ROS2 nodes written & on Pi | `vigia_ws/src/vigia_edge_node/src/` — branch `claude/great-jepsen-4d776c` pulled to Pi |
| 2026-06-17 | `vigia_msgs` package written | 8 custom message definitions in `vigia_ws/src/vigia_msgs/msg/` |
| 2026-06-17 | ONNX Runtime C++ 1.20.1 installed | `/opt/onnxruntime/` — **NO KleidiAI** (stock CPU EP only) |
| 2026-06-17 | ONNX Runtime Python 1.27.0 installed | pip, `--break-system-packages` — **NO KleidiAI** |
| 2026-06-17 | Eigen3 3.4.0 installed | `sudo apt install libeigen3-dev` |

## Build Completion Matrix (updated 2026-06-17)

| Component | Location | Status | Blocker |
|---|---|---|---|
| `vigia_msgs` (messages) | `vigia_ws/src/vigia_msgs/` | ⏳ Awaiting colcon | ROS2 Jazzy build completing |
| `vigia_edge_node` (6 nodes) | `vigia_ws/src/vigia_edge_node/` | ⏳ Awaiting colcon | ROS2 Jazzy build completing |
| ROS2 Jazzy (`rclcpp`, `sensor_msgs`) | `~/ros2_jazzy/` on Pi | 🔄 Building (build 7 running) | `tracetools` needed `-DTRACETOOLS_DISABLED=ON` |
| ONNX Runtime + **KleidiAI** | — | ❌ NOT BUILT | Needs ORT from source + ARM Compute Library (`-DONNXRUNTIME_USE_KLEIDIAI=ON`) |
| ARM Compute Library (ACL) | — | ❌ NOT BUILT | Prerequisite for KleidiAI EP |
| PREEMPT_RT kernel (active) | `/boot/firmware/` | ❌ NOT ACTIVE | Installed but not booted — requires physical access to Pi |
| STM32 firmware (Phase 2) | `firmware/` | ❌ NOT STARTED | Phase 2 |
| Anti-death MsgPack + MQTT (Phase 5) | `anti_death_node.cpp` | 🔧 STUB | Skeleton written; Paho MQTT + msgpack-c deps needed |
| DePIN ECDSA + Cosmos 3 (Phase 6) | — | ❌ NOT STARTED | Phase 6 |

## KleidiAI Build Plan (Phase 3 prerequisite)

KleidiAI gives **~4× INT8 GEMM throughput** on Cortex-A76 via `asimddp` (UDOT) instructions.
The Pi 5 has `asimddp` confirmed in `/proc/cpuinfo`. Build order:

```bash
# Step 1 — ARM Compute Library (~20 min)
git clone --depth 1 https://github.com/ARM-software/ComputeLibrary.git /opt/acl
cd /opt/acl && scons Werror=0 debug=0 asserts=0 neon=1 opencl=0 os=linux arch=arm64-v8.2-a \
  build=native -j4

# Step 2 — ONNX Runtime with KleidiAI (~45 min)
git clone --depth 1 --branch v1.20.1 https://github.com/microsoft/onnxruntime /tmp/ort_src
cd /tmp/ort_src && ./build.sh --config Release --arm --use_acl --acl_home /opt/acl \
  --acl_libs /opt/acl/build --parallel --cmake_extra_defines \
  DONNXRUNTIME_USE_KLEIDIAI=ON

# Step 3 — install to /opt/onnxruntime-kleidi/
```

Expected result: YOLO INT8 latency drops from ~28 ms → ~7 ms per frame (4× on GEMM layers).

---

## 1. Executive Summary

VIGIA is migrating from a Bharat AI-SoC Hackathon prototype (C++17, CFS scheduler, OpenVINO vision-only) into a commercial-grade **ADAS DePIN (Decentralized Physical Infrastructure Network) Edge Node**. The node collects multimodal sensor data, produces cryptographically attested road hazard events, and contributes to a decentralized world model (NVIDIA Cosmos 3) in a Sybil-resistant manner.

The migration is structured as **six engineering phases**, each with discrete hardware and software deliverables and acceptance criteria. Phase 1 is constrained entirely to **hardware currently on hand** (see §4). Phases 2–6 are sequentially dependent.

---

## 2. Baseline Architecture Audit

### 2.1 Current Code State (Phase 0 — Confirmed by Static Analysis)

| Component | File | Current Behavior | Target Behavior |
|---|---|---|---|
| Thread orchestration | `src/coordinator.cpp` | 3x `std::thread`, `SCHED_OTHER` (CFS), `pthread_setaffinity_np` | ROS 2 nodes, `SCHED_FIFO`, `StaticSingleThreadedExecutor` |
| Inter-thread transport | `include/safe_queue.hpp` | `std::mutex` + `std::condition_variable` + `frame.clone()` heap copy | `rclcpp::intra_process_comm` zero-copy `std::unique_ptr<sensor_msgs::msg::Image>` |
| Vision inference | `src/perception.cpp` | OpenVINO 2025, YOLO26 INT8 (320×320), forced FP32 precision hint | ONNX Runtime + KleidiAI INT8 UDOT micro-kernels, true INT8 execution |
| Depth inference | `src/analytical.cpp` | OpenVINO 2025, MiDaS v2.1 FP32 (256×256), `.clone()` on depth output | ONNX Runtime, MiDaS INT8 (pending quant evaluation), zero-copy tensor wrap |
| Fusion | `src/fusion.cpp` | Weighted blend: YOLO 40% + geometry 35% + temporal 25% | Adds ISS (velocity-normalized, gravity-compensated) as 4th input |
| Telemetry | `src/coordinator.cpp:311–326` | `stdout` debug prints (suppressed in release) | ROS 2 topics → MQTT → SIM7600 LTE |
| Hardware sensors | (none) | Vision-only | IMU + GPS + Secure Element + LTE |
| Storage | (none) | No persistent storage | `/dev/shm` volatile RAM disk (rolling 10s buffer) |
| Security | (none) | No signing or authentication | ATECC608A ECDSA (Pico 2 side) signs kinematic context `E_t` |

### 2.2 Known Performance Baseline (Pi 5, INT8 YOLO, PREEMPT_RT pending)

| Metric | Current (Phase 0) | Target (Phase 1+) |
|---|---|---|
| YOLO26 INT8 throughput | ~32 FPS / 28 ms (OpenVINO) | ≥30 FPS / ≤33 ms (ONNX Runtime + KleidiAI) |
| MiDaS FP32 latency | ~525 ms (stride-5 adaptive) | ≤200 ms (INT8 eval required) |
| End-to-end pipeline P95 | ~139 ms | ≤80 ms (zero-copy eliminates clone overhead) |
| MiDaS clone cost | ~0.9 ms/frame × stride | 0 ms (intra-process zero-copy) |

---

## 3. Target Architecture Overview

### 3.1 Compute Matrix

```
┌─────────────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5 (HPCU)                        │
│  OS: PREEMPT_RT Linux 6.6+   Middleware: ROS 2 Jazzy            │
│                                                                  │
│  Core 0 ── [Camera Capture Node]   SCHED_FIFO priority 80       │
│  Core 1 ── [Vision Inference Node] SCHED_FIFO priority 75       │
│  Core 2 ── [Depth Analysis Node]   SCHED_FIFO priority 75       │
│  Core 3 ── [Fusion Node]           SCHED_FIFO priority 70       │
│            [Sensor Bridge Node]    SCHED_FIFO priority 85       │
│            [Anti-Death Handler]    SCHED_FIFO priority 99       │
│                                                                  │
│  /dev/shm ── Rolling 10s frame buffer (seqlock-protected)       │
│  /dev/ttyACM0 ── USB-CDC from Pico 2 (COBS binary protocol)    │
└────────────────────────┬────────────────────────────────────────┘
                         │ USB-CDC (COBS binary packets, 921600 baud)
┌────────────────────────▼────────────────────────────────────────┐
│              Raspberry Pi Pico 2 / RP2350 (RPU)                  │
│  OS: Bare-metal (Pico SDK — no RTOS)                             │
│                                                                  │
│  SPI0 ──► BNO085 IMU (4D quaternion + linear accel, 100 Hz)     │
│  UART1 ──► NEO-M8N GPS (UBX binary, 10 Hz)                      │
│  I2C1 ──► ATECC608A Secure Element (ECDSA secp256r1 signing)     │
│                                                                  │
│  Responsibility: Hash + sign kinematic context E_t               │
│                  before forwarding to Pi via USB-CDC             │
└─────────────────────────────────────────────────────────────────┘
         │                              │
    BNO085 IMU                    NEO-M8N GPS
    (SPI @ 3 MHz)                 (UART @ 9600 baud)
```

### 3.2 Software Stack

| Layer | Technology |
|---|---|
| OS | PREEMPT_RT Linux 6.6 (custom Pi 5 kernel build) |
| Middleware | ROS 2 Jazzy Jalopy (C++ nodes, `rclcpp`) |
| IPC Transport | `rclcpp::intra_process_comm` — zero-copy `std::unique_ptr` handoffs |
| Vision Runtime | ONNX Runtime 1.18+ with KleidiAI backend (INT8 UDOT NEON assembly) |
| Vision Models | YOLOv26 Nano (ONNX, INT8, 320×320) + MiDaS v2.1 small (ONNX, INT8 eval) |
| Sensor Protocol | COBS-encoded binary packets over USB-CDC (`/dev/ttyACM0`) |
| Storage | `/dev/shm` volatile RAM disk — **no NVMe, no SD writes** |
| Telemetry Uplink | MQTT over SIM7600 LTE module |
| Security | ATECC608A (Pico 2) ECDSA secp256r1 — signs `E_t` kinematic context |
| DePIN Target | NVIDIA Cosmos 3 world model (server-side attestation) |

---

## 4. Phase 1 Hardware Constraints (ENFORCED)

> **These constraints are non-negotiable for the Phase 1 prototype sprint. All architectural decisions in Phase 1 must respect these boundaries.**

| Constraint | Enforced Rule |
|---|---|
| **No NVMe HAT/SSD** | All video buffer storage MUST use `/dev/shm` (volatile RAM disk). No `fwrite()`, no SQLite, no file-based ring buffers that touch the SD card. |
| **No 12V DC-DC buck** | Emergency shutdown trigger MUST come exclusively from the 18650 UPS GPIO status pin. No ignition-sense line. No automotive CAN. |
| **ATECC608A on Pico 2 only** | The Pi 5 has no direct access to the secure element. The Pico 2 is the sole signing authority for `E_t`. The Pi's spatial latent vector `S_t` is **unsigned in Phase 1**. |
| **5V bench power via 18650 UPS** | Power budget: ~5A sustained at 5V = 25W max. Thermal constraint: stay below 80°C sustained (Pi 5 throttle threshold). |

---

## 5. Three Industry-Grade Improvements

These improvements address gaps in the original specification that would cause correctness failures, safety violations, or priority inversions in a real-time embedded context.

---

### Improvement 1 — PREEMPT_RT + ROS 2 Executor Anti-Pattern *(Critical — Correctness)*

**The Gap:** The original spec says "ROS 2 with `rclcpp::intra_process_comm` + `SCHED_FIFO`." The critical omission is executor selection. ROS 2's default executors (`SingleThreadedExecutor`, `MultiThreadedExecutor`) use `std::mutex` constructs internally that are **not priority-inheritance-aware**. Under PREEMPT_RT, a high-priority RT thread blocked on a mutex held by a low-priority executor thread causes **unbounded priority inversion** — the exact failure mode PREEMPT_RT is designed to eliminate.

**Industry Standard (Apex.AI, `ros2_realtime_examples`):**
- Use `rclcpp::executors::StaticSingleThreadedExecutor` — eliminates dynamic memory allocation during spin, uses a pre-allocated callback queue.
- Each RT node runs on a dedicated OS thread manually created with `std::thread`, then promoted with `pthread_setschedparam(SCHED_FIFO, priority)` and `pthread_setaffinity_np`.
- Never call `rclcpp::spin()` on the main thread for any RT node.

**SCHED_FIFO Priority Ladder for Phase 1:**

| Thread | Node | Priority | Core | Rationale |
|---|---|---|---|---|
| `anti_death_handler` | AntiDeathNode | **99** | 3 | Must preempt everything — 15s power window |
| `sensor_bridge` | SensorBridgeNode | **85** | 3 | IMU/GPS data must not be dropped |
| `capture` | CameraNode | **80** | 0 | Frame acquisition is time-source |
| `vision` | VisionNode | **75** | 1 | YOLO inference, soft real-time |
| `depth` | DepthNode | **75** | 2 | MiDaS inference, soft real-time |
| `fusion` | FusionNode | **70** | 3 (shared) | Data processing, lowest RT priority |

**Action:** Every phase spec that defines a ROS 2 node MUST include its executor type, OS thread launch pattern, SCHED_FIFO priority, and core affinity in the node contract.

---

### Improvement 2 — IMU Gravity Compensation Before ISS Computation *(Algorithmically Critical — Correctness)*

**The Gap:** The spec defines ISS (Impact Severity Score) as:
```
ISS = Z_accel_spike / v_GPS
```
This is **physically incorrect**. The BNO085 IMU outputs raw linear accelerometer data that includes the **1g gravity vector**. On any road incline (even 5°), the gravity component projects onto the device's Z-axis, causing false ISS spikes proportional to incline angle — completely independent of actual road impact. The formula as written will misclassify every hill as a pothole.

**Industry Standard (Comma.ai Openpilot, Mobileye kinematic pipeline):**

The BNO085 simultaneously outputs a **4D unit quaternion** `q = (w, x, y, z)` representing orientation relative to the Earth frame. The correct pipeline is:

```
1. Rotate raw accel vector into world frame:
   a_world = q ⊗ a_body ⊗ q*          (quaternion sandwich product)

2. Subtract gravity:
   a_detrended = a_world - [0, 0, 9.81]  (remove Earth gravity component)

3. Compute ISS with dead-zone guard:
   ISS = |a_detrended.z| / max(v_GPS, v_min)
   where v_min = 2.0 m/s  (tunable — prevents divide-by-zero in parking lots)
```

**Action:** The Sensor Fusion Node (Phase 4) contract MUST specify the quaternion rotation step as a mandatory pre-processing stage. `v_min = 2.0 m/s` is a tunable parameter to be validated in the Phase 4 field test. The BNO085 MUST be configured in `NDOF` fusion mode (not raw IMU mode) to output calibrated quaternions.

---

### Improvement 3 — Seqlock on `/dev/shm` Ring Buffer *(Safety Critical — Liveness)*

**The Gap:** The anti-death handler (SCHED_FIFO 99) must snapshot the `/dev/shm` rolling frame buffer within 15 seconds of UPS GPIO assertion. The concurrent writer is the Camera Capture Node (SCHED_FIFO 80). The naive fix — a `std::mutex` guarding the ring buffer — is dangerous:

Under PREEMPT_RT, if the capture thread (priority 80) holds the mutex and the anti-death handler (priority 99) context-switches in, the handler will block on a mutex held by a lower-priority thread. While PREEMPT_RT's PI futex partially mitigates this via priority inheritance, the snapshot path becomes unbounded if the camera driver ISR also runs at priority 80 and re-enters the capture thread context. This is the **priority ceiling problem** in embedded RT systems.

**Industry Standard (Linux RT kernel, VxWorks, FreeRTOS seqlock pattern):**

Use a **seqlock** (sequence lock) — a lock-free, single-writer / multi-reader synchronization primitive:

```cpp
// Writer (CameraNode capture thread, SCHED_FIFO 80):
seq_.fetch_add(1, std::memory_order_release);  // mark odd = writing
ring_buffer_[write_idx_] = frame;              // write frame
seq_.fetch_add(1, std::memory_order_release);  // mark even = stable

// Snapshot reader (AntiDeathNode, SCHED_FIFO 99):
uint32_t seq1, seq2;
do {
    seq1 = seq_.load(std::memory_order_acquire);
    if (seq1 & 1) continue;                    // spin if writer active
    snapshot = ring_buffer_;                   // copy entire buffer
    std::atomic_thread_fence(std::memory_order_acquire);
    seq2 = seq_.load(std::memory_order_relaxed);
} while (seq1 != seq2);                        // retry if write occurred mid-read
```

This gives the anti-death handler a **wait-free snapshot path** — it never blocks, never deadlocks, and is guaranteed to complete in bounded time regardless of the capture thread's state.

**Action:** The `/dev/shm` ring buffer data structure in Phase 5 MUST use a `std::atomic<uint32_t>` seqlock. The buffer struct definition and seqlock wrapper MUST appear in the Phase 5 node contract before any implementation begins.

---

## 6. Engineering Phases

### Phase 1 — OS & Zero-Copy ROS 2 Middleware
**Goal:** Replace the CFS/pthread foundation with PREEMPT_RT + ROS 2 without regressing vision performance.

**Deliverables:**
- [ ] PREEMPT_RT Linux 6.6 kernel built and booted on Pi 5 (verify with `uname -a` showing `PREEMPT_RT`)
- [ ] ROS 2 Jazzy installed (Debian arm64 or source build)
- [ ] `CameraNode` — wraps existing `captureLoop()` logic, publishes `sensor_msgs/msg/Image` intra-process
- [ ] `VisionNode` — wraps existing `processLoop()` + `perception.cpp`, subscribes zero-copy
- [ ] `DepthNode` — wraps existing `midasLoop()` + `analytical.cpp`, subscribes zero-copy
- [ ] `FusionNode` — wraps existing `fusion.cpp`, publishes `vigia_msgs/msg/HazardEvent`
- [ ] `StaticSingleThreadedExecutor` per node, each on dedicated `std::thread` with `SCHED_FIFO` pinning per §5 priority table
- [ ] `/dev/shm` ring buffer with seqlock (struct definition only — no anti-death logic yet)
- [ ] CMakeLists.txt updated: `ament_cmake`, `rclcpp`, `sensor_msgs` dependencies
- [ ] Existing `safe_queue.hpp` DEPRECATED — replaced by ROS 2 intra-process transport

**Acceptance Criteria:**
- `ros2 topic hz /vigia/detections` reports ≥28 FPS sustained for 60 seconds
- `cyclictest -p99 -t4 -n` shows max latency ≤100 µs (confirms RT kernel)
- No `frame.clone()` calls remain in CameraNode or VisionNode hot paths
- `valgrind --tool=massif` shows flat heap profile during 60s run (no allocation growth)

**Hardware Constraints Active:** No Pico 2, no sensors. Pi 5 + camera only.

---

### Phase 2 — Pico 2 Sensor Bridge
**Goal:** Bring the Raspberry Pi Pico 2 online as a bare-metal sensor aggregation and signing RPU.

**Deliverables:**
- [ ] Pico 2 firmware (separate repo or `firmware/` subdirectory):
  - BNO085 driver over SPI0 (3 MHz, DMA-driven) — NDOF fusion mode, 100 Hz quaternion + linear accel output
  - NEO-M8N driver over UART1 (9600 baud) — UBX binary parser, `NAV-PVT` message at 10 Hz (lat, lon, alt, speed, course, fix quality, HDOP)
  - ATECC608A driver over I2C1 (400 kHz) — Microchip `cryptoauthlib` (RP2040/RP2350 HAL)
  - COBS encoder — frames `E_t` payload as COBS packet before USB-CDC transmit (TinyUSB)
- [ ] COBS Packet Format (on wire):

```
[COBS_START][version:1B][type:1B][seq:4B][timestamp_us:8B][payload:NB][ecdsa_sig:64B][COBS_END]
```

- `type = 0x01` → `IMU_QUATERNION` (q_w, q_x, q_y, q_z : f32×4, a_x, a_y, a_z : f32×3) = 28 bytes payload
- `type = 0x02` → `GPS_PVT` (lat, lon : f64×2, alt, speed, course : f32×3, fix:u8, hdop:f32, sats:u8) = 34 bytes payload
- `type = 0x03` → `SIGNED_ET` (IMU + GPS combined, ECDSA signature over SHA-256 hash of payload)

- [ ] `SensorBridgeNode` (Pi 5, ROS 2) — reads `/dev/ttyACM0` at 921600 baud, COBS-decodes, validates ECDSA signature, publishes:
  - `vigia_msgs/msg/ImuSample` → `/vigia/imu` (100 Hz)
  - `vigia_msgs/msg/GpsPvt` → `/vigia/gps` (10 Hz)
  - `vigia_msgs/msg/SignedEt` → `/vigia/signed_et` (10 Hz, carries ECDSA signature)
- [ ] Custom message definitions in `vigia_msgs/` package

**Acceptance Criteria:**
- `ros2 topic hz /vigia/imu` reports ≥95 Hz sustained
- `ros2 topic hz /vigia/gps` reports ≥9 Hz sustained
- ECDSA signature validates on Pi side for 1000 consecutive `SIGNED_ET` packets (zero failures)
- Packet loss rate ≤0.1% over 10-minute run (measured by seq number gaps)

---

### Phase 3 — ONNX Runtime Vision Engine + Spatial Latent Vector
**Goal:** Replace OpenVINO with ONNX Runtime + KleidiAI backend and extract the spatial latent vector `S_t`.

**Deliverables:**
- [ ] ONNX Runtime 1.18+ built for arm64 with KleidiAI backend (`-DONNXRUNTIME_USE_KLEIDIAI=ON`)
- [ ] YOLOv26 Nano exported to ONNX (from Ultralytics), INT8 quantized via ONNX Runtime quantization tool
- [ ] MiDaS v2.1 small exported to ONNX — evaluate INT8 quantization impact on depth accuracy (accept if mean relative error ≤5% vs FP32 baseline)
- [ ] `VisionNode` updated: `Ort::Session` replaces `ov::CompiledModel`
- [ ] **Spatial Latent Vector extraction:** Hook into the penultimate feature map layer of YOLOv26 (the layer immediately before the detection head). Extract as a flattened `std::vector<float>` — this is `S_t`, the semantic scene descriptor.
  - Verify layer name in ONNX graph via `Netron` visualization
  - Add second `Ort::Session::Run()` output node for the penultimate feature map
  - Publish `S_t` as `vigia_msgs/msg/SpatialLatent` (float32 array) on `/vigia/spatial_latent`
- [ ] `DepthNode` updated: `Ort::Session` for MiDaS
- [ ] OpenVINO dependency removed from `CMakeLists.txt`

**Acceptance Criteria:**
- YOLO26 INT8 (ONNX Runtime + KleidiAI) achieves ≥28 FPS on Pi 5 (parity with OpenVINO baseline)
- `S_t` vector published at camera framerate (≥28 Hz)
- `S_t` L2-norm is non-trivially different between "pothole present" and "clear road" frames (qualitative sanity check — ≥0.15 cosine distance between scene classes)

---

### Phase 4 — Algorithmic Fusion & ISS
**Goal:** Implement the Speed-Normalized Impact Severity Score with gravity compensation and integrate all sensor streams into the fusion pipeline.

**Deliverables:**
- [ ] `FusionNode` updated to subscribe to `/vigia/imu`, `/vigia/gps`, `/vigia/detections`, `/vigia/depth`
- [ ] **Gravity Compensation pipeline** (see §5, Improvement 2):
  - BNO085 quaternion → rotate accel vector to world frame
  - Subtract [0, 0, 9.81] m/s²
  - Output: `a_detrended` (gravity-free linear acceleration in world frame)
- [ ] **ISS computation:**
  ```
  ISS = |a_detrended.z| / max(v_GPS, 2.0)   // v_min = 2.0 m/s
  ```
- [ ] **Kalman Filter** — fuse GPS velocity with IMU-integrated velocity for dead-reckoning when GPS fix quality < 3 or HDOP > 2.5:
  - State: `[v_x, v_y, v_z, a_x, a_y, a_z]` (6-DOF)
  - Measurement: GPS `NAV-PVT` speed (when available)
  - Process noise: IMU linear accel (100 Hz input)
- [ ] Updated `FusionEngine` scoring:
  ```
  RRI = 0.35 × YOLO_conf + 0.25 × geometry_conf + 0.15 × temporal_conf + 0.25 × ISS_normalized
  ```
- [ ] Tunable parameters externalized to ROS 2 parameters YAML (`config/fusion_params.yaml`)

**Acceptance Criteria:**
- ISS reads ≈0.0 on flat road at constant 30 km/h (gravity compensation working)
- ISS reads elevated (>1.5) on speed bump crossing at 20 km/h (confirmed in field test)
- Kalman filter maintains velocity estimate within ±0.5 m/s of GPS during 10-second GPS outage simulation

---

### Phase 5 — Anti-Death Storage & MQTT Uplink
**Goal:** Implement the emergency event capture and LTE transmission pipeline within the 15-second UPS power window.

**Deliverables:**
- [ ] `/dev/shm` rolling frame buffer:
  - Size: 300 frames × (320×320 RGB) = ~92 MB — verify available RAM (Pi 5 has 8GB LPDDR4X)
  - Write path: CameraNode writes frames via seqlock (see §5, Improvement 3)
  - Metadata ring: per-frame `[timestamp_us, imu_sample, gps_pvt, yolo_detections, midas_depth_hash]`
- [ ] **UPS GPIO handler** (`AntiDeathNode`, SCHED_FIFO 99):
  - Monitors GPIO pin (sysfs or libgpiod) for UPS `POWER_FAIL` assertion
  - On assertion: seqlock snapshot → serialize 300-frame buffer to in-memory `msgpack` blob
  - Attach `vigia_msgs/msg/SignedEt` from latest Pico 2 packet (E_t already ECDSA-signed)
  - Attach `S_t` spatial latent from latest VisionNode output (unsigned in Phase 1)
  - Publish via MQTT to `vigia/events/{device_id}/hazard`
- [ ] **SIM7600 LTE MQTT integration:**
  - Driver: AT command interface over `/dev/ttyUSB2` (or equivalent SIM7600 CDC ACM port)
  - MQTT library: Eclipse Paho C++ async client
  - TLS 1.2 (TLS 1.3 if SIM7600 firmware supports it — check AT+CSSLCFG)
  - QoS 1 (at-least-once) for hazard events
- [ ] **State machine** for 15-second window:
  ```
  RUNNING → [UPS_GPIO_ASSERT] → CAPTURING_SNAPSHOT (≤2s)
           → SERIALIZING (≤3s)
           → MQTT_CONNECTING (≤5s, with retry)
           → MQTT_TRANSMITTING (≤4s)
           → SAFE_SHUTDOWN (remaining time)
  ```
- [ ] Systemd unit `vigia-edge.service` with `WantedBy=multi-user.target` and `Restart=always`

**Acceptance Criteria:**
- End-to-end from GPIO assert to MQTT `PUBACK` received in ≤13 seconds (2s margin)
- MQTT payload received and parsed by test server with correct `device_id`, `event_timestamp`, and ≥290 frames in buffer
- System survives 10 simulated power-fail events without data corruption in `/dev/shm`

---

### Phase 6 — DePIN Security & Attestation
**Goal:** Complete the cryptographic attestation pipeline and integrate with the NVIDIA Cosmos 3 world model endpoint.

**Deliverables:**
- [ ] **Pico 2 signing pipeline** (completes Phase 2 stub):
  - ATECC608A provisioned with device private key (secp256r1) and X.509 certificate via Microchip Trust Platform
  - Pico 2 computes `E_t` = `SHA-256(IMU_quaternion ∥ GPS_PVT ∥ timestamp_us ∥ device_id)`
  - ATECC608A performs ECDSA sign of `E_t` hash → 64-byte signature
  - Included in `SIGNED_ET` COBS packet to Pi
- [ ] **Pi-side attestation payload assembly:**
  ```json
  {
    "device_id": "<uuid>",
    "timestamp_us": <u64>,
    "S_t": [<float32 array>],          // spatial latent (unsigned, Phase 1)
    "E_t_hash": "<hex>",               // SHA-256 of kinematic context
    "E_t_signature": "<base64>",       // ECDSA sig from ATECC608A
    "E_t_cert": "<PEM>",               // device certificate chain
    "frames_count": <u32>,
    "rri_score": <f32>
  }
  ```
- [ ] MQTT topic: `vigia/attest/{device_id}` — consumed by server-side Cosmos 3 pipeline
- [ ] **Anti-Sybil validation** (server side, documented only): Server verifies ECDSA signature against device certificate chain anchored to Microchip Trust Platform root CA

**Acceptance Criteria:**
- Server-side ECDSA verification passes for 100 consecutive events (zero signature failures)
- Replayed event (same `E_t_hash`) rejected by server monotonic counter check
- `device_id` in payload matches provisioned certificate CN

---

## 7. Cross-Cutting Constraints

| Constraint | Enforcement |
|---|---|
| **No SD card writes in hot path** | All runtime data to `/dev/shm` or ROS 2 intra-process. Logs to `journald` only (ramoops). |
| **No NVMe** | Zero persistent storage architecture. Buffer is volatile by design — loss on unclean shutdown is acceptable (UPS provides clean-shutdown path). |
| **No 12V / automotive power** | UPS GPIO is the only shutdown trigger. No `SIGPWR`, no CAN bus, no ignition-sense. |
| **ATECC608A on Pico 2 only** | Pi never calls cryptoauthlib. Pi receives pre-signed `E_t` packets and forwards them. `S_t` is unsigned in Phase 1. |
| **PREEMPT_RT kernel mandatory from Phase 1** | CFS scheduler is insufficient for deterministic SCHED_FIFO priority enforcement. |
| **StaticSingleThreadedExecutor mandatory** | Default ROS 2 executors cause priority inversion under PREEMPT_RT — never use them for RT nodes. |

---

## 8. Phase Dependencies & Critical Path

```
Phase 1 (OS + ROS2 Middleware)
    │
    ├──► Phase 2 (Pico 2 Bridge) ───────────────────────────────────┐
    │                                                              │
    └──► Phase 3 (ONNX Runtime + S_t) ──► Phase 4 (ISS Fusion) ──┤
                                                                   │
                                               Phase 5 (Anti-Death + MQTT)
                                                                   │
                                               Phase 6 (DePIN Security + Attestation)
```

**Critical path:** Phase 1 → Phase 3 → Phase 4 → Phase 5 → Phase 6  
Phase 2 (Pico 2) can be developed in parallel with Phase 3 and integrated at Phase 4.

---

## 9. Open Questions (Resolve Before Node Contracts)

1. **ONNX Runtime vs OpenVINO benchmark:** Has KleidiAI INT8 ONNX Runtime been benchmarked against OpenVINO INT8 on Pi 5? If ONNX Runtime underperforms by >15%, reconsider the runtime choice for Phase 3.

2. **MiDaS INT8 accuracy:** MiDaS depth estimation is sensitive to quantization. The Phase 3 acceptance criterion of ≤5% mean relative error needs empirical validation on Pi 5 road imagery before committing to INT8.

3. **SIM7600 TLS version:** Confirm SIM7600 firmware supports TLS 1.2 with client certificate authentication (`AT+CSSLCFG`). If not, MQTT will fall back to TLS 1.2 server-auth only (reduced security for Phase 5).

4. **`S_t` layer selection in YOLOv26:** The penultimate feature map of YOLOv26 Nano needs to be identified in the ONNX graph (Netron). Its dimensionality determines the `SpatialLatent` message size. Confirm the layer name and output shape before Phase 3 node contract is written.

5. **BNO085 SPI wiring on Pico 2:** Confirm BNO085 breakout is wired to SPI0 (GP18/GP19/GP16/GP17). SPI is mandatory for 100 Hz — I2C is not supported in Phase 2 firmware spec.

---

*Next document: `.claude/design/02_ros2_node_contracts.md` — defines the ROS 2 node interface for each Phase 1 component (topics, message types, executor config, thread parameters). Await approval before authoring.*

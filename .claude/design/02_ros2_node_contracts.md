# VIGIA ADAS DePIN Edge Node
## ROS 2 Node Interface Contracts
**Document:** `02_ros2_node_contracts.md`  
**Depends on:** `01_system_architecture_and_roadmap.md` (APPROVED)  
**Status:** AWAITING APPROVAL — No implementation until sign-off  
**Scope:** Phase 1 (OS & Zero-Copy Middleware) + Phase 4 (Algorithmic Fusion & ISS)

---

## 0. Architectural Decisions Integrated (Resolved Open Questions)

| Question | Resolution | Enforcement in Contracts |
|---|---|---|
| Vision runtime | **ONNX Runtime 1.18+ + KleidiAI EP** — mandatory, OpenVINO fully removed | `VisionNode` and `DepthNode` use `Ort::Session` only |
| Mixed precision | **YOLOv26 → INT8. MiDaS → FP32.** INT8 MiDaS is prohibited | `VisionNode` session allows INT8; `DepthNode` session locks FP32 |
| LTE security | **TLS 1.2 mutual auth assumed** (SIM7600 firmware confirmed capable) | Phase 5 MQTT spec will mandate `AT+CSSLCFG` client cert load |
| `SpatialLatent` type | **`float32[]` dynamic array** — layer-dimension agnostic | `vigia_msgs/msg/SpatialLatent.msg` uses unbounded array |
| BNO085 bus | **SPI1 on STM32 Black Pill (PA5/PA6/PA7/PA4), 4 MHz, DMA** — mandatory | STM32 Phase 2 firmware spec will enforce SPI1 with DMA channel 2 |

---

## 1. Process Architecture

All six nodes run **in a single OS process** (`vigia_edge_node`). This is mandatory for `rclcpp::intra_process_comm` to function — zero-copy intra-process transfers only operate within a shared address space.

```
vigia_edge_node (single process)
├── CameraNode      → std::thread T0 → Core 0, SCHED_FIFO 80
├── VisionNode      → std::thread T1 → Core 1, SCHED_FIFO 75
├── DepthNode       → std::thread T2 → Core 2, SCHED_FIFO 75
├── FusionNode      → std::thread T3 → Core 3, SCHED_FIFO 70
├── SensorBridgeNode→ std::thread T4 → Core 3, SCHED_FIFO 85
└── AntiDeathNode   → std::thread T5 → Core 3, SCHED_FIFO 99
```

Cores 0–2 are dedicated (one node each). Core 3 hosts three nodes at different SCHED_FIFO priorities; the RT kernel preempts correctly by priority ladder.

---

## 2. Intra-Process Zero-Copy Mandate

Every node MUST be constructed with `use_intra_process_comms(true)`. This is **not optional**.

```cpp
// Canonical node construction pattern — required for ALL 6 nodes:
rclcpp::NodeOptions options;
options.use_intra_process_comms(true);
auto node = std::make_shared<CameraNode>(options);  // example
```

### Zero-Copy Ownership Semantics

| Publisher → Subscriber(s) | Transfer Type | Rationale |
|---|---|---|
| `CameraNode` → `VisionNode` + `DepthNode` (2 subscribers) | `std::shared_ptr<const sensor_msgs::msg::Image>` | ROS 2 intra-process promotes `unique_ptr` to `shared_ptr` when fan-out > 1. No data copy — reference-counted, zero-serialization. |
| `VisionNode` → `FusionNode` (1 subscriber) | `std::unique_ptr<vigia_msgs::msg::DetectionArray>` | Single consumer → true ownership transfer, zero-copy. |
| `VisionNode` → `AntiDeathNode` (1 subscriber) | `std::unique_ptr<vigia_msgs::msg::SpatialLatent>` | Single consumer → ownership transfer. |
| `DepthNode` → `FusionNode` (1 subscriber) | `std::unique_ptr<vigia_msgs::msg::DepthMap>` | Single consumer → ownership transfer. |
| `SensorBridgeNode` → `FusionNode` + `AntiDeathNode` (2 subscribers) | `std::shared_ptr<const vigia_msgs::msg::SignedEt>` | Fan-out 2 → shared_ptr intra-process. |
| `FusionNode` → `AntiDeathNode` (1 subscriber) | `std::unique_ptr<vigia_msgs::msg::HazardEvent>` | Single consumer → ownership transfer. |

**Publisher call-site rule:** All publishers in the hot path MUST call the `unique_ptr` overload:
```cpp
// CORRECT — transfers ownership, zero-copy intra-process:
auto msg = std::make_unique<sensor_msgs::msg::Image>();
// ... fill msg ...
publisher_->publish(std::move(msg));

// PROHIBITED — triggers unnecessary copy:
sensor_msgs::msg::Image msg;
publisher_->publish(msg);
```

---

## 3. OS Thread Launch Pattern (Mandatory Boilerplate)

Every node uses this **identical thread launch pattern**. Differences are only in `kPriority`, `kCore`, and the node pointer.

```cpp
// vigia_edge_node/src/rt_thread.hpp
#pragma once
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <string>

struct RtThreadConfig {
    int  sched_priority;   // SCHED_FIFO priority (1–99)
    int  cpu_core;         // CPU affinity (0–3)
    std::string name;      // pthread name (max 15 chars)
};

inline std::thread launch_rt_node(
    std::shared_ptr<rclcpp::Node> node,
    RtThreadConfig cfg)
{
    return std::thread([node, cfg]() {
        // 1. Set thread name (visible in htop / ps)
        pthread_setname_np(pthread_self(), cfg.name.c_str());

        // 2. Pin to CPU core
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cfg.cpu_core, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            throw std::runtime_error("pthread_setaffinity_np failed for " + cfg.name);
        }

        // 3. Elevate to SCHED_FIFO (requires CAP_SYS_NICE or root)
        sched_param sp{};
        sp.sched_priority = cfg.sched_priority;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
            throw std::runtime_error("pthread_setschedparam SCHED_FIFO failed for " + cfg.name);
        }

        // 4. Spin executor — StaticSingleThreadedExecutor only
        rclcpp::executors::StaticSingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    });
}
```

**`main()` entry point pattern:**
```cpp
rclcpp::init(argc, argv);

// Construct all nodes (intra-process enabled on all)
auto camera_node  = std::make_shared<CameraNode>(make_ipc_options());
auto vision_node  = std::make_shared<VisionNode>(make_ipc_options());
auto depth_node   = std::make_shared<DepthNode>(make_ipc_options());
auto fusion_node  = std::make_shared<FusionNode>(make_ipc_options());
auto bridge_node  = std::make_shared<SensorBridgeNode>(make_ipc_options());
auto antideath_node = std::make_shared<AntiDeathNode>(make_ipc_options());

// Launch RT threads — order matters: highest priority first
auto t5 = launch_rt_node(antideath_node, {99, 3, "vigia_antideath"});
auto t4 = launch_rt_node(bridge_node,    {85, 3, "vigia_bridge"});
auto t0 = launch_rt_node(camera_node,    {80, 0, "vigia_camera"});
auto t1 = launch_rt_node(vision_node,    {75, 1, "vigia_vision"});
auto t2 = launch_rt_node(depth_node,     {75, 2, "vigia_depth"});
auto t3 = launch_rt_node(fusion_node,    {70, 3, "vigia_fusion"});

t0.join(); t1.join(); t2.join(); t3.join(); t4.join(); t5.join();
rclcpp::shutdown();
```

---

## 4. Custom Message Definitions (`vigia_msgs` package)

All custom messages live in `vigia_msgs/msg/`. They are defined before any node implementation.

### 4.1 `vigia_msgs/msg/Detection.msg`
```
# Single object detection result from YOLO inference
uint32          class_id
string          class_label
float32         confidence
sensor_msgs/RegionOfInterest  bbox    # pixel coordinates in original frame space
```

### 4.2 `vigia_msgs/msg/DetectionArray.msg`
```
# All detections for one frame
std_msgs/Header         header      # stamp = inference completion time
uint32                  frame_id    # monotonic frame counter from CameraNode
vigia_msgs/Detection[]  detections  # empty array = no detections (not null)
float32                 inference_latency_ms
```

### 4.3 `vigia_msgs/msg/SpatialLatent.msg`
```
# Penultimate feature map extracted from YOLOv26 — semantic scene descriptor S_t
# Array length is runtime-determined by ONNX graph; unbounded to remain layer-agnostic.
std_msgs/Header  header
uint32           frame_id
float32[]        latent_vector       # flattened, row-major. Length = product of feature map dims.
string           source_layer_name   # ONNX node name of extracted layer (for provenance)
```

### 4.4 `vigia_msgs/msg/DepthMap.msg`
```
# MiDaS FP32 depth output — normalized inverse depth in [0.0, 1.0]
# Higher values = closer to camera. Row-major, width x height floats.
std_msgs/Header  header
uint32           frame_id
uint32           width               # 256 (MiDaS v2.1 small native output)
uint32           height              # 256
float32[]        data                # width * height elements, row-major
float32          inference_latency_ms
```

### 4.5 `vigia_msgs/msg/ImuSample.msg`
```
# BNO085 NDOF fusion mode output — calibrated quaternion + gravity-free linear accel
# All values forwarded verbatim from STM32 COBS packet. Gravity compensation done Pi-side.
std_msgs/Header  header
float32          q_w                 # Unit quaternion (world←body orientation)
float32          q_x
float32          q_y
float32          q_z
float32          lin_accel_x         # Body-frame linear accel m/s² (gravity-included)
float32          lin_accel_y
float32          lin_accel_z
uint8            calibration_status  # BNO085 cal status: 0=uncal, 3=fully cal
```

### 4.6 `vigia_msgs/msg/GpsPvt.msg`
```
# NEO-M8N UBX NAV-PVT parsed fields
std_msgs/Header  header
float64          latitude            # degrees (WGS-84)
float64          longitude           # degrees (WGS-84)
float32          altitude_m          # meters above ellipsoid
float32          speed_ms            # ground speed m/s
float32          course_deg          # heading degrees (0=North, clockwise)
uint8            fix_type            # 0=none 1=dead_reckoning 2=2D 3=3D 4=GNSS+DR
float32          hdop                # Horizontal dilution of precision
uint8            satellites_used
bool             valid_fix           # fix_type >= 2 AND hdop <= 2.5
```

### 4.7 `vigia_msgs/msg/SignedEt.msg`
```
# Kinematic context E_t signed by STM32 ATECC608A (secp256r1 ECDSA)
# The Pi forwards this payload verbatim — it does NOT re-sign or modify.
std_msgs/Header  header
uint32           sequence            # Monotonic counter from STM32 (anti-replay)
uint64           stm32_timestamp_us  # STM32 hardware timer at signing time
uint8[32]        et_hash             # SHA-256( IMU_quaternion ∥ GPS_PVT ∥ timestamp ∥ device_id )
uint8[64]        ecdsa_signature     # secp256r1 DER-encoded signature from ATECC608A
uint8[]          device_cert_der     # X.509 DER certificate chain (device + intermediate)
```

### 4.8 `vigia_msgs/msg/HazardEvent.msg`
```
# Complete fused hazard detection event. Published when RRI >= threshold.
std_msgs/Header             header
string                      device_id           # UUID provisioned at manufacture
float32                     rri_score           # Road Risk Index [0.0, 1.0]
float32                     iss_score           # Impact Severity Score (gravity-compensated)
float32                     yolo_confidence     # Best detection confidence
float32                     geometry_confidence
float32                     temporal_confidence
vigia_msgs/DetectionArray   detections
vigia_msgs/DepthMap         depth_map
vigia_msgs/ImuSample        imu_sample          # Sample at event time
vigia_msgs/GpsPvt           gps_pvt             # Fix at event time
vigia_msgs/SignedEt         signed_et           # Pre-signed E_t from STM32
float32[]                   spatial_latent      # S_t — unsigned in Phase 1
```

---

## 5. QoS Profile Definitions

Define these profiles once in `vigia_edge_node/include/vigia_qos.hpp` and reference by name in all node implementations.

```cpp
// vigia_edge_node/include/vigia_qos.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>

namespace vigia::qos {

// Raw sensor streams — tolerate drops, never block the pipeline.
// Best effort + volatile + keep_last(1): only the latest sample matters.
inline rclcpp::QoS sensor_stream() {
    return rclcpp::QoS(rclcpp::KeepLast(1))
        .best_effort()
        .durability_volatile();
}

// Camera frames — tolerate drops at high rate, keep a small queue for burst smoothing.
inline rclcpp::QoS camera_frames() {
    return rclcpp::QoS(rclcpp::KeepLast(4))
        .best_effort()
        .durability_volatile();
}

// Inference results — reliable delivery within the process, small queue.
inline rclcpp::QoS inference_results() {
    return rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .durability_volatile();
}

// Critical events — reliable + transient_local: late-joining subscribers receive last event.
inline rclcpp::QoS hazard_events() {
    return rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .transient_local();
}

// Signed kinematic context — reliable + small queue (10 Hz data, must not drop).
inline rclcpp::QoS signed_et() {
    return rclcpp::QoS(rclcpp::KeepLast(5))
        .reliable()
        .durability_volatile();
}

}  // namespace vigia::qos
```

---

## 6. Node Contracts

---

### 6.1 `CameraNode`

**Responsibility:** Acquire frames from the camera at the target FPS. Write each frame to the `/dev/shm` seqlock ring buffer. Publish frames intra-process for zero-copy handoff to `VisionNode` and `DepthNode`.

**File:** `vigia_edge_node/src/camera_node.cpp` + `camera_node.hpp`

#### Published Topics

| Topic | Message Type | QoS Profile | Transfer Semantics |
|---|---|---|---|
| `/vigia/camera/image_raw` | `sensor_msgs/msg/Image` | `vigia::qos::camera_frames()` | `std::unique_ptr` publish → promoted to `shared_ptr` intra-process (fan-out = 2) |

#### Subscribed Topics
*None.*

#### Parameters (ROS 2 Params, loaded from `config/camera_params.yaml`)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `camera_index` | `int` | `0` | V4L2 device index |
| `target_fps` | `double` | `30.0` | Frame capture rate |
| `frame_width` | `int` | `1280` | Capture resolution width |
| `frame_height` | `int` | `720` | Capture resolution height |
| `shm_ring_size` | `int` | `300` | `/dev/shm` ring buffer depth (frames) |

#### Executor & Thread Configuration

| Property | Value |
|---|---|
| Executor | `rclcpp::executors::StaticSingleThreadedExecutor` |
| OS Thread | Dedicated `std::thread` via `launch_rt_node()` |
| `SCHED_FIFO` Priority | **80** |
| CPU Core Affinity | **Core 0** |
| `pthread` Name | `vigia_camera` |
| `use_intra_process_comms` | **`true`** (mandatory) |

#### Key Implementation Constraints

- The `cv::VideoCapture::read()` call MUST NOT hold any mutex during blocking wait.
- Frame image data MUST be placed into a pre-allocated `sensor_msgs::msg::Image` buffer (pre-allocated in constructor, re-used each frame via `std::move` into `unique_ptr`) to eliminate per-frame heap allocation.
- After publishing, the node MUST also write the raw `cv::Mat` data into the `/dev/shm` seqlock ring buffer (see §7 for ring buffer spec). These are two independent write paths.
- Encoding: `bgr8` (OpenCV native — conversion to RGB happens in `VisionNode`).

---

### 6.2 `VisionNode`

**Responsibility:** Run YOLOv26 INT8 inference via ONNX Runtime + KleidiAI EP on every received frame. Extract the penultimate feature map as `S_t`. Publish detections and spatial latent vector.

**File:** `vigia_edge_node/src/vision_node.cpp` + `vision_node.hpp`

#### Published Topics

| Topic | Message Type | QoS Profile | Transfer Semantics |
|---|---|---|---|
| `/vigia/detections` | `vigia_msgs/msg/DetectionArray` | `vigia::qos::inference_results()` | `std::unique_ptr` → single consumer (`FusionNode`) |
| `/vigia/spatial_latent` | `vigia_msgs/msg/SpatialLatent` | `vigia::qos::inference_results()` | `std::unique_ptr` → single consumer (`AntiDeathNode`) |

#### Subscribed Topics

| Topic | Message Type | QoS Profile | Callback Arg Type |
|---|---|---|---|
| `/vigia/camera/image_raw` | `sensor_msgs/msg/Image` | `vigia::qos::camera_frames()` | `std::shared_ptr<const sensor_msgs::msg::Image>` (shared — fan-in from CameraNode, shared with DepthNode) |

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `model_path` | `string` | `models/yolov26/yolov26_nano_int8.onnx` | Path to INT8 ONNX model |
| `latent_layer_name` | `string` | `""` | ONNX node name for S_t extraction — populated after Netron inspection |
| `conf_threshold` | `double` | `0.25` | NMS confidence threshold |
| `nms_iou_threshold` | `double` | `0.45` | NMS IoU threshold |
| `input_width` | `int` | `320` | Model input width (px) |
| `input_height` | `int` | `320` | Model input height (px) |

#### Executor & Thread Configuration

| Property | Value |
|---|---|
| Executor | `rclcpp::executors::StaticSingleThreadedExecutor` |
| OS Thread | Dedicated `std::thread` via `launch_rt_node()` |
| `SCHED_FIFO` Priority | **75** |
| CPU Core Affinity | **Core 1** |
| `pthread` Name | `vigia_vision` |
| `use_intra_process_comms` | **`true`** (mandatory) |

#### ONNX Runtime Session Configuration (mandatory)

```cpp
// VisionNode constructor — ONNX Runtime session setup
Ort::SessionOptions session_opts;

// INT8 MANDATORY — prohibit FP32 fallback
session_opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
session_opts.AddConfigEntry("session.set_denormal_as_zero", "1");

// KleidiAI EP — ARM64 INT8 UDOT micro-kernels via ACL
// Requires ONNX Runtime built with -DONNXRUNTIME_USE_KLEIDIAI=ON
OrtArenaCfg arena_cfg{0, -1, -1, -1};  // default arena
Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_ACL(
    session_opts, /*enable_fast_math=*/1));

// Thread count: 1 (node is pinned to Core 1 — ACL uses its own NEON scheduler internally)
session_opts.SetIntraOpNumThreads(1);
session_opts.SetInterOpNumThreads(1);

// Disable memory patterns to prevent dynamic arena growth in RT context
session_opts.DisableMemPattern();

yolo_session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_opts);
```

#### S_t Extraction Contract

- At session initialization, verify `latent_layer_name` exists as an output node in the ONNX graph.
- Run inference with TWO output nodes: `[output_detection_head, latent_layer_name]`.
- Flatten the latent tensor to `std::vector<float>` and populate `SpatialLatent::latent_vector`.
- Populate `SpatialLatent::source_layer_name` with the exact ONNX node name for traceability.
- If `latent_layer_name` is empty (not yet configured), publish `SpatialLatent` with empty `latent_vector` — do not crash.

#### Key Implementation Constraints

- Preprocessing MUST reuse pre-allocated `cv::Mat` buffers (resize target, letterbox pad, CHW blob) — no per-frame heap allocation.
- BGR→RGB conversion MUST occur before resize (smaller buffer to convert).
- CHW transposition MUST use NEON intrinsics (`vld3q_u8`) for INT8 data — do not use `cv::dnn::blobFromImage` (allocates internally).
- YOLO output postprocessing (NMS) MUST reuse the existing `nmsBoxes()` from `src/perception.cpp` — do not rewrite.

---

### 6.3 `DepthNode`

**Responsibility:** Run MiDaS v2.1 small FP32 inference via ONNX Runtime on a stride-gated subset of frames. Publish normalized inverse depth maps for use in geometry confidence scoring.

**File:** `vigia_edge_node/src/depth_node.cpp` + `depth_node.hpp`

#### Published Topics

| Topic | Message Type | QoS Profile | Transfer Semantics |
|---|---|---|---|
| `/vigia/depth` | `vigia_msgs/msg/DepthMap` | `vigia::qos::inference_results()` | `std::unique_ptr` → single consumer (`FusionNode`) |

#### Subscribed Topics

| Topic | Message Type | QoS Profile | Callback Arg Type |
|---|---|---|---|
| `/vigia/camera/image_raw` | `sensor_msgs/msg/Image` | `vigia::qos::camera_frames()` | `std::shared_ptr<const sensor_msgs::msg::Image>` (shared with VisionNode) |

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `model_path` | `string` | `models/midasv21/midas_v21_small_256.onnx` | Path to FP32 ONNX model |
| `midas_stride` | `int` | `3` | Run inference every N frames (adaptive — overridden by thermal monitor) |
| `temp_warn_threshold_c` | `double` | `75.0` | Increase stride to 4 at this temperature |
| `temp_critical_threshold_c` | `double` | `85.0` | Increase stride to 6 at this temperature |

#### Executor & Thread Configuration

| Property | Value |
|---|---|
| Executor | `rclcpp::executors::StaticSingleThreadedExecutor` |
| OS Thread | Dedicated `std::thread` via `launch_rt_node()` |
| `SCHED_FIFO` Priority | **75** |
| CPU Core Affinity | **Core 2** |
| `pthread` Name | `vigia_depth` |
| `use_intra_process_comms` | **`true`** (mandatory) |

#### ONNX Runtime Session Configuration (mandatory)

```cpp
// DepthNode constructor — ONNX Runtime session setup
Ort::SessionOptions session_opts;

// FP32 MANDATORY — INT8 MiDaS is prohibited (accuracy degradation)
// Do NOT add KleidiAI EP here. CPU EP only.
session_opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

// Permit FP32 precision only — explicitly block INT8/FP16 graph transformations
session_opts.AddConfigEntry("session.disable_quant_qdq_cleanup", "1");

// 2 threads: Core 2 is dedicated, but MiDaS benefits from limited parallelism
// across its depthwise separable conv layers
session_opts.SetIntraOpNumThreads(2);
session_opts.SetInterOpNumThreads(1);
session_opts.DisableMemPattern();

midas_session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_opts);
```

#### Key Implementation Constraints

- Frame stride gating: Maintain `uint64_t frame_counter_` incremented on each callback. Only invoke inference when `frame_counter_ % midas_stride_ == 0`. On skipped frames, do NOT publish (FusionNode must handle missing depth gracefully).
- Thermal monitoring: Read `/sys/class/thermal/thermal_zone0/temp` at most once per second (cached). Adjust `midas_stride_` dynamically per parameter thresholds.
- Preprocessing MUST reuse pre-allocated buffers from `src/analytical.cpp` — `midasResized_` and `midasBlob_` — see `latency-deep-dive.md` Priority 4 fix.
- Depth output MUST wrap the ONNX tensor data as a `cv::Mat` header (zero-copy via `Ort::Value::GetTensorMutableData<float>()`) before populating `DepthMap::data`. Do not `.clone()` the depth output.

---

### 6.4 `FusionNode`

**Responsibility:** Receive all inference outputs and sensor data. Perform gravity compensation, ISS computation, Kalman filter velocity fusion, and weighted RRI scoring. Publish `HazardEvent` when `RRI >= rri_threshold`.

**File:** `vigia_edge_node/src/fusion_node.cpp` + `fusion_node.hpp`

#### Published Topics

| Topic | Message Type | QoS Profile | Transfer Semantics |
|---|---|---|---|
| `/vigia/hazard_event` | `vigia_msgs/msg/HazardEvent` | `vigia::qos::hazard_events()` | `std::unique_ptr` → single consumer (`AntiDeathNode`) |

#### Subscribed Topics

| Topic | Message Type | QoS Profile | Callback Arg Type |
|---|---|---|---|
| `/vigia/detections` | `vigia_msgs/msg/DetectionArray` | `vigia::qos::inference_results()` | `std::unique_ptr<vigia_msgs::msg::DetectionArray>` |
| `/vigia/depth` | `vigia_msgs/msg/DepthMap` | `vigia::qos::inference_results()` | `std::unique_ptr<vigia_msgs::msg::DepthMap>` |
| `/vigia/imu` | `vigia_msgs/msg/ImuSample` | `vigia::qos::sensor_stream()` | `std::shared_ptr<const vigia_msgs::msg::ImuSample>` |
| `/vigia/gps` | `vigia_msgs/msg/GpsPvt` | `vigia::qos::sensor_stream()` | `std::shared_ptr<const vigia_msgs::msg::GpsPvt>` |
| `/vigia/signed_et` | `vigia_msgs/msg/SignedEt` | `vigia::qos::signed_et()` | `std::shared_ptr<const vigia_msgs::msg::SignedEt>` |

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `rri_threshold` | `double` | `0.75` | Minimum RRI to publish HazardEvent |
| `w_yolo` | `double` | `0.35` | YOLO confidence weight in RRI |
| `w_geometry` | `double` | `0.25` | Geometry confidence weight |
| `w_temporal` | `double` | `0.15` | Temporal confidence weight |
| `w_iss` | `double` | `0.25` | ISS weight in RRI |
| `v_min_ms` | `double` | `2.0` | Minimum velocity for ISS denominator (m/s) — prevents divide-by-zero |
| `kalman_process_noise` | `double` | `0.01` | Kalman filter process noise covariance |
| `kalman_meas_noise` | `double` | `0.5` | Kalman filter measurement noise covariance (GPS) |
| `gps_fallback_hdop_max` | `double` | `2.5` | Maximum HDOP before switching to IMU dead-reckoning |

#### Executor & Thread Configuration

| Property | Value |
|---|---|
| Executor | `rclcpp::executors::StaticSingleThreadedExecutor` |
| OS Thread | Dedicated `std::thread` via `launch_rt_node()` |
| `SCHED_FIFO` Priority | **70** |
| CPU Core Affinity | **Core 3** |
| `pthread` Name | `vigia_fusion` |
| `use_intra_process_comms` | **`true`** (mandatory) |

#### Gravity Compensation Pipeline (Improvement 2 — mandatory)

```
Input:  ImuSample { q_w, q_x, q_y, q_z, lin_accel_x, lin_accel_y, lin_accel_z }

Step 1 — Quaternion sandwich product (body → world frame rotation):
    q       = Eigen::Quaternionf(q_w, q_x, q_y, q_z).normalized()
    a_body  = Eigen::Vector3f(lin_accel_x, lin_accel_y, lin_accel_z)
    a_world = q * a_body   // Eigen handles q*v*q^-1 via Quaternion::operator*(Vector3)

Step 2 — Gravity subtraction:
    a_detrended = a_world - Eigen::Vector3f(0.0f, 0.0f, 9.81f)

Step 3 — ISS computation:
    float v_gps = std::max(gps_pvt.speed_ms, static_cast<float>(v_min_ms_))
    float iss   = std::abs(a_detrended.z()) / v_gps
```

**Dependency:** Add `Eigen3` to `vigia_msgs` package dependencies. No dynamic allocation — `Eigen::Quaternionf` and `Eigen::Vector3f` are stack-allocated.

#### Kalman Filter State (velocity dead-reckoning)

- **State vector:** `x = [v_x, v_y]` (2D ground velocity, m/s, world frame)
- **Predict step** (triggered by each `ImuSample` callback at 100 Hz): Integrate `a_detrended.x()` and `a_detrended.y()` over `dt`.
- **Update step** (triggered by each valid `GpsPvt` callback): Fuse GPS speed components when `gps_pvt.valid_fix == true && gps_pvt.hdop <= gps_fallback_hdop_max`.
- **ISS uses `v_gps`** from GPS directly when GPS valid; falls back to `||x||` (Kalman estimated speed) when GPS invalid.
- Implementation: Use Eigen-based 2×2 Kalman (plain matrices, no dynamic allocation). Do not use `robot_localization` package — overkill and introduces non-RT dependencies.

#### Message Synchronization Policy

- Use `message_filters::ApproximateTimeSynchronizer` on `detections` + `depth` (sync tolerance: 100 ms — MiDaS runs at stride, so exact sync is impossible).
- IMU and GPS are consumed asynchronously via separate callbacks — always store latest sample in `latest_imu_` and `latest_gps_` class members.
- When `detections` arrive: immediately compute RRI using latest cached IMU/GPS/depth state. Do not wait for all streams to align beyond the detection+depth sync pair.

---

### 6.5 `SensorBridgeNode`

**Responsibility:** Read COBS-framed binary packets from the STM32 over `/dev/ttyACM0`. Decode and validate each packet. Verify ECDSA signature on `SIGNED_ET` packets. Publish IMU, GPS, and signed kinematic context to the ROS 2 graph.

**File:** `vigia_edge_node/src/sensor_bridge_node.cpp` + `sensor_bridge_node.hpp`

#### Published Topics

| Topic | Message Type | QoS Profile | Transfer Semantics |
|---|---|---|---|
| `/vigia/imu` | `vigia_msgs/msg/ImuSample` | `vigia::qos::sensor_stream()` | `std::unique_ptr` → shared_ptr (fan-out: FusionNode + AntiDeathNode) |
| `/vigia/gps` | `vigia_msgs/msg/GpsPvt` | `vigia::qos::sensor_stream()` | `std::unique_ptr` → shared_ptr (fan-out: FusionNode + AntiDeathNode) |
| `/vigia/signed_et` | `vigia_msgs/msg/SignedEt` | `vigia::qos::signed_et()` | `std::unique_ptr` → shared_ptr (fan-out: FusionNode + AntiDeathNode) |

#### Subscribed Topics
*None.* Input comes from `/dev/ttyACM0` via a non-ROS blocking read loop.

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `serial_port` | `string` | `/dev/ttyACM0` | STM32 USB-CDC device path |
| `baud_rate` | `int` | `921600` | Serial line speed |
| `ecdsa_verify_enabled` | `bool` | `true` | Verify ECDSA sig on SIGNED_ET packets; drop packet if invalid |
| `device_cert_path` | `string` | `/etc/vigia/device_cert.pem` | Expected device certificate for ECDSA verification |

#### Executor & Thread Configuration

| Property | Value |
|---|---|
| Executor | `rclcpp::executors::StaticSingleThreadedExecutor` |
| OS Thread | Dedicated `std::thread` via `launch_rt_node()` |
| `SCHED_FIFO` Priority | **85** |
| CPU Core Affinity | **Core 3** |
| `pthread` Name | `vigia_bridge` |
| `use_intra_process_comms` | **`true`** (mandatory) |

#### Key Implementation Constraints

- The serial read loop runs as a `rclcpp::TimerBase` callback at 1 ms period (1000 Hz poll). At 921600 baud, a 34-byte GPS packet takes ~0.37 ms to transmit — 1 ms polling ensures no packet is held in the UART buffer for more than one timer tick.
- COBS decoder MUST be implemented as a pure state machine with no dynamic allocation. Packet buffer is pre-allocated at max packet size (512 bytes).
- ECDSA verification: Use mbedTLS `mbedtls_pk_verify()` with `MBEDTLS_PK_ECDSA` and `MBEDTLS_MD_SHA256`. Add `mbedtls` to CMakeLists.txt dependencies.
- On ECDSA failure: increment `sig_fail_counter_`, log `RCLCPP_WARN`, and **drop the packet** — do not publish an unverified `SignedEt`.
- Monotonic sequence check: If `packet.sequence <= last_sequence_`, log `RCLCPP_WARN` (replay attempt) and drop.

---

### 6.6 `AntiDeathNode`

**Responsibility:** Monitor the UPS GPIO `POWER_FAIL` pin. On assertion, execute the emergency snapshot-serialize-transmit sequence within the 15-second power window. This node has the highest SCHED_FIFO priority in the system (99) and must never be blocked.

**File:** `vigia_edge_node/src/anti_death_node.cpp` + `anti_death_node.hpp`

#### Published Topics
*None.* Emergency output goes directly to MQTT (SIM7600 LTE), bypassing the ROS 2 graph.

#### Subscribed Topics

| Topic | Message Type | QoS Profile | Callback Arg Type |
|---|---|---|---|
| `/vigia/spatial_latent` | `vigia_msgs/msg/SpatialLatent` | `vigia::qos::inference_results()` | `std::unique_ptr<vigia_msgs::msg::SpatialLatent>` |
| `/vigia/signed_et` | `vigia_msgs/msg/SignedEt` | `vigia::qos::signed_et()` | `std::shared_ptr<const vigia_msgs::msg::SignedEt>` |
| `/vigia/hazard_event` | `vigia_msgs/msg/HazardEvent` | `vigia::qos::hazard_events()` | `std::unique_ptr<vigia_msgs::msg::HazardEvent>` |

All subscriber callbacks do nothing except update `latest_*_` cached members under a seqlock. No business logic runs in the subscriber callbacks.

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ups_gpio_chip` | `string` | `/dev/gpiochip4` | libgpiod GPIO chip (Pi 5 uses gpiochip4) |
| `ups_gpio_line` | `int` | `17` | GPIO line number for UPS POWER_FAIL signal |
| `ups_gpio_active_low` | `bool` | `true` | UPS POWER_FAIL asserted low (active-low signal) |
| `power_window_seconds` | `double` | `15.0` | Total available time from GPIO assert to power loss |
| `mqtt_broker_host` | `string` | — | MQTT broker hostname (required) |
| `mqtt_broker_port` | `int` | `8883` | MQTT broker TLS port |
| `mqtt_topic_prefix` | `string` | `vigia/events` | Topic: `{prefix}/{device_id}/hazard` |

#### Executor & Thread Configuration

| Property | Value |
|---|---|
| Executor | `rclcpp::executors::StaticSingleThreadedExecutor` |
| OS Thread | Dedicated `std::thread` via `launch_rt_node()` |
| `SCHED_FIFO` Priority | **99** |
| CPU Core Affinity | **Core 3** |
| `pthread` Name | `vigia_antideath` |
| `use_intra_process_comms` | **`true`** (mandatory) |

#### GPIO Monitoring

```cpp
// Anti-death GPIO setup — libgpiod (no sysfs polling, no busy-loop)
// Runs as a dedicated blocking thread WITHIN the AntiDeathNode executor loop.
gpiod::chip chip(ups_gpio_chip_);
gpiod::line line = chip.get_line(ups_gpio_line_);
line.request({
    "vigia_antideath",
    gpiod::line_request::EVENT_FALLING_EDGE,  // active-low UPS signal
    /*flags=*/0
});

// Blocking edge-wait (≤1ms latency on PREEMPT_RT, replaces polling):
if (line.event_wait(std::chrono::milliseconds(100))) {
    auto event = line.event_read();
    if (event.event_type == gpiod::line_event::FALLING_EDGE) {
        execute_emergency_sequence();
    }
}
```

#### Emergency Sequence State Machine

```
State: RUNNING
  │ [UPS GPIO FALLING EDGE detected — T=0.000s]
  ▼
State: CAPTURING_SNAPSHOT  (budget: ≤2.0s)
  │  seqlock_snapshot(shm_ring_buffer_)  → raw_frames[300]
  │  capture latest_signed_et_, latest_spatial_latent_, latest_hazard_event_
  ▼
State: SERIALIZING          (budget: ≤3.0s)
  │  msgpack::pack(payload)   → std::vector<uint8_t> blob
  │  Attach: E_t (pre-signed by STM32), S_t (unsigned Phase 1)
  ▼
State: MQTT_CONNECTING      (budget: ≤5.0s, with 3 retries × 1.5s backoff)
  │  mqtt::async_client connect to broker:8883 (TLS 1.2 mutual auth)
  ▼
State: MQTT_TRANSMITTING    (budget: ≤4.0s)
  │  mqtt::async_client::publish(topic, blob, QoS=1)
  │  wait for PUBACK
  ▼
State: SAFE_SHUTDOWN        (remaining time)
     rclcpp::shutdown()
     sync()   // flush any journald writes
     // Power loss occurs here — /dev/shm contents are lost (expected)
```

**Budget enforcement:** Each state transition checks `elapsed_time > state_budget`. If exceeded, skip to `SAFE_SHUTDOWN` immediately. Never exceed the 15-second window.

#### Key Implementation Constraints

- `execute_emergency_sequence()` MUST run entirely on the `vigia_antideath` thread (SCHED_FIFO 99). It MUST NOT `co_await`, call `rclcpp::spin_some()`, or yield back to the executor.
- The seqlock snapshot (§7) MUST be the first operation — before any serialization or network I/O — to maximize the captured frame count.
- Eclipse Paho C++ async MQTT client MUST be initialized at node startup (not during emergency sequence). TLS context, certificates, and connection parameters loaded at startup. Only `publish()` is called during the emergency.
- `sync()` call before shutdown ensures `journald` ring buffer is flushed to the ramoops kernel crash log (available after reboot for post-mortem).

---

## 7. `/dev/shm` Seqlock Ring Buffer Specification

Defined in `vigia_edge_node/include/shm_ring_buffer.hpp`. Shared between `CameraNode` (writer) and `AntiDeathNode` (snapshot reader).

```cpp
// vigia_edge_node/include/shm_ring_buffer.hpp
#pragma once
#include <atomic>
#include <array>
#include <cstdint>
#include <cstring>

constexpr int    SHM_RING_DEPTH    = 300;      // ~10s at 30 FPS
constexpr size_t SHM_FRAME_BYTES   = 1280 * 720 * 3;  // BGR8 raw frame

struct alignas(64) ShmFrame {
    uint64_t  timestamp_us;
    uint32_t  frame_id;
    uint8_t   data[SHM_FRAME_BYTES];
};

struct ShmRingBuffer {
    // Seqlock counter: even = stable, odd = write in progress
    // Cache-line aligned to prevent false sharing with frame data
    alignas(64) std::atomic<uint32_t> seq{0};

    uint32_t  write_idx{0};              // Next write position (not atomic — single writer)
    ShmFrame  frames[SHM_RING_DEPTH];   // Allocated in /dev/shm via mmap

    // Writer (CameraNode, SCHED_FIFO 80):
    void write_frame(uint32_t frame_id, uint64_t ts_us, const uint8_t* bgr_data) {
        seq.fetch_add(1, std::memory_order_release);   // mark: write starting (odd)
        auto& f = frames[write_idx % SHM_RING_DEPTH];
        f.frame_id     = frame_id;
        f.timestamp_us = ts_us;
        std::memcpy(f.data, bgr_data, SHM_FRAME_BYTES);
        write_idx++;
        seq.fetch_add(1, std::memory_order_release);   // mark: write complete (even)
    }

    // Snapshot reader (AntiDeathNode, SCHED_FIFO 99) — wait-free:
    void snapshot(std::array<ShmFrame, SHM_RING_DEPTH>& out) {
        uint32_t s1, s2;
        do {
            s1 = seq.load(std::memory_order_acquire);
            if (s1 & 1u) { continue; }                 // spin if write in progress
            std::memcpy(out.data(), frames, sizeof(frames));
            std::atomic_thread_fence(std::memory_order_acquire);
            s2 = seq.load(std::memory_order_relaxed);
        } while (s1 != s2);
    }
};
```

**Allocation:** `ShmRingBuffer` is allocated via `mmap(MAP_SHARED | MAP_ANONYMOUS)` backed by `/dev/shm/vigia_ring.buf`. `mlock()` MUST be called on the allocation to prevent page faults during the emergency sequence (`mlockall(MCL_CURRENT | MCL_FUTURE)` called at process start).

**Size:** `300 × (1280×720×3) ≈ 829 MB`. Pi 5 has 8GB LPDDR4X. RAM disk usage is acceptable. Verify available `/dev/shm` space at startup; abort with clear error if < 900 MB available.

---

## 8. ROS 2 Topic Summary

| Topic | Publisher | Subscriber(s) | Message Type | QoS |
|---|---|---|---|---|
| `/vigia/camera/image_raw` | CameraNode | VisionNode, DepthNode | `sensor_msgs/msg/Image` | `camera_frames` |
| `/vigia/detections` | VisionNode | FusionNode | `vigia_msgs/msg/DetectionArray` | `inference_results` |
| `/vigia/spatial_latent` | VisionNode | AntiDeathNode | `vigia_msgs/msg/SpatialLatent` | `inference_results` |
| `/vigia/depth` | DepthNode | FusionNode | `vigia_msgs/msg/DepthMap` | `inference_results` |
| `/vigia/imu` | SensorBridgeNode | FusionNode, AntiDeathNode | `vigia_msgs/msg/ImuSample` | `sensor_stream` |
| `/vigia/gps` | SensorBridgeNode | FusionNode, AntiDeathNode | `vigia_msgs/msg/GpsPvt` | `sensor_stream` |
| `/vigia/signed_et` | SensorBridgeNode | FusionNode, AntiDeathNode | `vigia_msgs/msg/SignedEt` | `signed_et` |
| `/vigia/hazard_event` | FusionNode | AntiDeathNode | `vigia_msgs/msg/HazardEvent` | `hazard_events` |

---

## 9. Package Structure

```
vigia_edge_node/
├── CMakeLists.txt              # ament_cmake, rclcpp, sensor_msgs, vigia_msgs, Eigen3, mbedtls, gpiod
├── package.xml
├── config/
│   ├── camera_params.yaml
│   ├── vision_params.yaml
│   ├── depth_params.yaml
│   ├── fusion_params.yaml
│   ├── sensor_bridge_params.yaml
│   └── anti_death_params.yaml
├── include/vigia_edge_node/
│   ├── rt_thread.hpp           # launch_rt_node() boilerplate
│   ├── vigia_qos.hpp           # QoS profile definitions
│   └── shm_ring_buffer.hpp     # Seqlock ring buffer
├── src/
│   ├── main.cpp                # Process entry point
│   ├── camera_node.cpp/.hpp
│   ├── vision_node.cpp/.hpp
│   ├── depth_node.cpp/.hpp
│   ├── fusion_node.cpp/.hpp
│   ├── sensor_bridge_node.cpp/.hpp
│   └── anti_death_node.cpp/.hpp

vigia_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── Detection.msg
    ├── DetectionArray.msg
    ├── SpatialLatent.msg
    ├── DepthMap.msg
    ├── ImuSample.msg
    ├── GpsPvt.msg
    ├── SignedEt.msg
    └── HazardEvent.msg
```

---

## 10. Acceptance Criteria (Phase 1 Complete)

| Test | Command | Pass Condition |
|---|---|---|
| PREEMPT_RT verified | `uname -a` | Contains `PREEMPT_RT` |
| SCHED_FIFO verified | `chrt -p $(pgrep vigia_camera)` | `scheduling policy: SCHED_FIFO`, `priority: 80` |
| Intra-process zero-copy | `ros2 run rqt_graph rqt_graph` + custom allocator tracer | No heap allocation in CameraNode → VisionNode frame path after warmup |
| Vision throughput | `ros2 topic hz /vigia/detections` | ≥28 Hz sustained for 60 seconds |
| Spatial latent live | `ros2 topic hz /vigia/spatial_latent` | ≥28 Hz; `latent_vector` length > 0 |
| Depth throughput | `ros2 topic hz /vigia/depth` | ≥8 Hz at stride-4 (28 FPS ÷ 4 × 1.15 tolerance) |
| RT latency | `sudo cyclictest -p99 -t4 -n -D 60` | Max latency ≤100 µs |
| No clone in hot path | `grep -r "\.clone()" src/` | Zero results in CameraNode, VisionNode, DepthNode |
| Seqlock ring buffer | Unit test: 60s write + snapshot under load | Zero torn snapshots (all snapshots have consistent seq counter) |

---

*Next document: `.claude/design/03_stm32_firmware_contracts.md` — STM32 Black Pill firmware spec: BNO085 SPI1 DMA driver, NEO-M8N UBX parser, ATECC608A signing pipeline, COBS packet encoder, and USB-CDC transmit loop.*

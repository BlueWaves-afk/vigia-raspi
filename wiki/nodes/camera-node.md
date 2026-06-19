---
title: "CameraNode"
type: ros2-node
tags: [ros2-node, capture, realtime]
source: vigia_ws/src/vigia_edge_node/src/camera_node.cpp
related: ["[[vision-node]]", "[[depth-node]]", "[[shm-ring-buffer]]", "[[anti-death-node]]", "[[vigia-qos]]", "[[rt-thread]]", "[[params-yaml]]", "[[raspberry-pi-5]]", "[[camera]]", "[[flow-capture-to-uplink]]"]
updated: 2026-06-19
---

# CameraNode

**File:** `vigia_ws/src/vigia_edge_node/src/camera_node.cpp` / `camera_node.hpp`

Acquires frames from the CSI camera (V4L2) at the target FPS. Writes each raw BGR frame to the [[shm-ring-buffer]] via seqlock. Publishes frames intra-process for zero-copy handoff to [[vision-node]] and [[depth-node]].

## Thread Configuration

| Property | Value |
|---|---|
| Executor | `rclcpp::executors::StaticSingleThreadedExecutor` |
| OS Thread | `launch_rt_node()` from [[rt-thread]] |
| SCHED_FIFO Priority | **80** |
| CPU Core Affinity | **Core 0** |
| pthread Name | `vigia_camera` |
| use_intra_process_comms | `true` (mandatory) |

## Published Topics

| Topic | Message Type | QoS | Transfer |
|---|---|---|---|
| `/vigia/camera/image_raw` | `sensor_msgs/msg/Image` | `camera_frames` (KeepLast 4, best-effort) | `unique_ptr` → `shared_ptr` (fan-out=2 to VisionNode + DepthNode) |

## Subscribed Topics
None. Input comes from V4L2 hardware.

## Key Behaviors
- Pre-allocates `sensor_msgs::msg::Image` buffer in constructor, re-uses each frame via `std::move` into `unique_ptr` — no per-frame heap allocation.
- `cv::VideoCapture::read()` must NOT hold any mutex during blocking wait.
- After ROS2 publish, also writes raw `cv::Mat` data into [[shm-ring-buffer]] via `write_frame(frame_id, ts_us, bgr_data)` — two independent write paths.
- Encoding: `bgr8` (OpenCV native; BGR→RGB conversion happens in [[vision-node]]).

## Parameters (from [[params-yaml]])
| Parameter | Default |
|---|---|
| `camera_index` | `0` |
| `target_fps` | `30.0` |
| `frame_width` | `1280` |
| `frame_height` | `720` |
| `shm_ring_size` | `300` |

## Links
- Writes to: [[shm-ring-buffer]] (seqlock writer, SCHED_FIFO 80)
- Publishes to: [[vision-node]] (shared_ptr intra-process), [[depth-node]] (shared_ptr intra-process)
- Physical hardware: [[camera]] (CSI V4L2), [[raspberry-pi-5]] (Core 0)
- Snapshot reader: [[anti-death-node]] (SCHED_FIFO 99)
- Config: [[params-yaml]] `camera_node` section
- Flow: [[flow-capture-to-uplink]]

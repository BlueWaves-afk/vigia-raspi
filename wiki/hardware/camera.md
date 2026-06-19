---
title: "CSI Camera Module"
type: hardware
tags: [hardware, sensor]
source: .claude/design/02_ros2_node_contracts.md
related: ["[[raspberry-pi-5]]", "[[camera-node]]", "[[vision-node]]", "[[depth-node]]"]
updated: 2026-06-19
---

# CSI Camera Module

**Interface:** CSI-2 on Raspberry Pi 5  
**V4L2 device:** `/dev/video0` (camera_index=0)  
**Capture resolution:** 1280×720 px, bgr8 encoding  
**Target rate:** 30 FPS  
**Status:** Hardware interface pending (CSI cable connection required per Ben's task list)

## Role
Source of all video frames in the VIGIA pipeline. Frame data flows:
1. [[camera-node]] reads via `cv::VideoCapture` (V4L2)
2. Publishes as `sensor_msgs/msg/Image` (bgr8, 1280×720)
3. Writes raw BGR to [[shm-ring-buffer]] (seqlock)
4. [[vision-node]] + [[depth-node]] subscribe intra-process (zero-copy shared_ptr)

## Preprocessing Chain
- [[vision-node]]: letterbox resize 1280×720 → 320×320, BGR→RGB, HWC→CHW (NEON `vld3q_u8`)
- [[depth-node]]: resize 1280×720 → 256×256, normalize to ImageNet FP32 (NEON `vld3_u8` + `vmlaq_f32`)

## Links
- Connected to: [[raspberry-pi-5]] (CSI-2)
- Read by: [[camera-node]] (V4L2)
- Consumers: [[vision-node]], [[depth-node]]

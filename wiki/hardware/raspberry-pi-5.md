---
title: "Raspberry Pi 5"
type: hardware
tags: [hardware, compute]
source: .claude/design/01_system_architecture_and_roadmap.md
related: ["[[pico-2]]", "[[camera]]", "[[18650-ups]]", "[[sim7600]]", "[[camera-node]]", "[[vision-node]]", "[[depth-node]]", "[[fusion-node]]", "[[sensor-bridge-node]]", "[[anti-death-node]]", "[[ble-gatt-node]]", "[[shm-ring-buffer]]", "[[kleidiai-acl]]"]
updated: 2026-06-19
---

# Raspberry Pi 5 (HPCU — High Performance Compute Unit)

**SoC:** BCM2712  
**CPU:** Cortex-A76 quad-core @ 2.4 GHz  
**RAM:** 8 GB LPDDR4X (~50 GB/s sustained bandwidth)  
**OS:** PREEMPT_RT Linux 6.12.73+rpt-arm64 (kernel `linux-image-6.12.73+deb13-rt-arm64`, installed but NOT YET BOOTED — requires physical access)  
**Middleware:** ROS2 Jazzy (from-source build in progress, `~/ros2_jazzy/`)  
**SSH:** Tailscale IP `100.114.1.98`, user `vigiasense@raspberrypi`, Debian Trixie

## Core-to-Node Mapping

| Core | Node | SCHED_FIFO | pthread Name |
|---|---|---|---|
| Core 0 | [[camera-node]] | 80 | `vigia_camera` |
| Core 1 | [[vision-node]] | 75 | `vigia_vision` |
| Core 2 | [[depth-node]] | 75 | `vigia_depth` |
| Core 3 | [[fusion-node]] | 70 | `vigia_fusion` |
| Core 3 | [[sensor-bridge-node]] | 85 | `vigia_bridge` |
| Core 3 | [[anti-death-node]] | 99 | `vigia_antideath` |

## /dev/shm (RAM Disk)
- [[shm-ring-buffer]]: `/dev/shm/vigia_frame_ring` — 829 MB, 300 frames
- [[frame-metadata-ring]]: `/dev/shm/vigia_meta_ring.buf` — 37 KB
- `mlockall(MCL_CURRENT|MCL_FUTURE)` at process start

## Installed Software
- ONNX Runtime 1.20.1 C++ at `/opt/onnxruntime/` (stock CPU EP only)
- Eigen3 3.4.0 (`/usr/include/eigen3`)
- libgpiod, libcurl, libsodium
- Pico SDK for cross-compilation

## Connectivity
- USB-C: power (5V from [[18650-ups]])
- USB-A: [[pico-2]] USB-CDC (`/dev/ttyACM0`)
- USB-A: [[sim7600]] LTE (ECM `usb0` + AT port `/dev/ttyUSB2`)
- CSI: [[camera]] (V4L2 `/dev/video0`)
- GPIO: [[18650-ups]] POWER_FAIL on gpiochip4 line 17
- Bluetooth: onboard BT radio for [[ble-gatt-node]] (BlueZ D-Bus)

## CPU Capability
- `asimddp` (ARM dot product / UDOT) confirmed in `/proc/cpuinfo` — required for [[kleidiai-acl]]
- L1D: 64 KB per core (private), L2: 512 KB per core (private), L3: 8 MB shared

## Links
- Hosts: all 6 ROS2 nodes, [[shm-ring-buffer]], [[frame-metadata-ring]]
- Connected to: [[pico-2]] (USB-CDC), [[sim7600]] (LTE), [[camera]] (CSI)
- Powered by: [[18650-ups]]

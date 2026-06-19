# ROS 2 Edge Node Graph

The production edge runtime (`vigia_ws/src/vigia_edge_node`) is a multi-node ROS 2
graph that superseded the monolithic `coordinator.cpp` described in
[[04_Coordinator_And_Threads]]. Each node is launched on a dedicated thread with an
explicit scheduling class via `launch_rt_node()` (`rt_thread.hpp`).

## Node map

```
                 ┌──────────────┐   /vigia/frames (shm)   ┌──────────────┐
   CSI camera ──▶│ camera_node  │────────────────────────▶│ vision_node  │ YOLO INT8 (ONNX)
                 └──────┬───────┘                          └──────┬───────┘
                        │ ShmMetaRing                              │ /vigia/detections + latent S_t
                        ▼                                          ▼
                 ┌──────────────┐                          ┌──────────────┐
                 │ depth_node   │ MiDaS v2.1 ──────────────▶│ fusion_node  │ RRI / ISS / Kalman
                 └──────────────┘                          └──────┬───────┘
   STM32 Pico ──▶┌──────────────┐ COBS@921600  IMU/GPS/E_t        │ /vigia/hazard_events
   (USB-CDC)     │sensor_bridge │────────────────────────────────┤
                 └──────────────┘                                  ├──────────────┬─────────────┐
                                                                   ▼              ▼             ▼
                                                           ┌─────────────┐ ┌────────────┐ ┌──────────┐
                                                           │hazard_uplink│ │ ble_gatt   │ │anti_death│
                                                           │ MQTT→IoT    │ │ phone link │ │ UPS ISR  │
                                                           └─────────────┘ └────────────┘ └──────────┘
```

## Nodes & scheduling

| Node | File | Sched / prio | Core | Role |
|------|------|--------------|------|------|
| `camera_node` | `src/camera_node.cpp` | SCHED_FIFO 80 | 0 | CSI capture → `/dev/shm` frame ring + `ShmMetaRing` |
| `vision_node` | `src/vision_node.cpp` | SCHED_FIFO 75 | 1 | YOLO26 INT8 via ONNX Runtime + `Ort::IoBinding`; emits spatial latent `S_t` |
| `depth_node` | `src/depth_node.cpp` | SCHED_FIFO 75 | 2 | MiDaS v2.1 depth |
| `fusion_node` | `src/fusion_node.cpp` | SCHED_FIFO 70 | — | Gravity-compensated ISS, RRI, Kalman dead-reckoning → `/vigia/hazard_events` |
| `sensor_bridge_node` | `src/sensor_bridge_node.cpp` | SCHED_FIFO 85 | — | COBS@921600 from Pico; parses IMU/GPS/`SignedEtPacketPi` (173 B); ECDSA verify |
| `anti_death_node` | `src/anti_death_node.cpp` | SCHED_FIFO 99 | — | UPS GPIO edge (libgpiod v2); seqlock snapshot; emergency uplink in 15 s budget |
| `ble_gatt_node` | `src/ble_gatt_node.cpp` | SCHED_OTHER 40 | — | sdbus-c++ v2 GATT server; ECDH handshake; phone telemetry (spec: `.claude/design/07_ble_transport_spec.md`) |
| `hazard_uplink_node` | `src/hazard_uplink_node.cpp` | SCHED_OTHER 30 | — | MsgPack → MQTT QoS-1 mTLS to AWS IoT Core → [[09_Cloud_Pipeline]] |

> [!NOTE]
> Priorities are real (`pthread_setschedparam`) but only take effect under a
> PREEMPT_RT kernel. The RT kernel is **installed but not booted** (GAP_TRACKER 1.1)
> — until then everything runs under CFS.

## Config

All node parameters are path-scoped YAML under `config/` (`camera_params.yaml`,
`vision_params.yaml`, `depth_params.yaml`, `fusion_params.yaml`,
`sensor_bridge_params.yaml`, `ble_gatt_params.yaml`, `anti_death_params.yaml`).
The combined `params.yaml` is loaded with `--params-file`; param keys are namespaced
by node name (`hazard_uplink_node:` ≠ `anti_death_node:`).

## Related
[[00_Index]] · [[04_Coordinator_And_Threads]] · [[06_Fusion_Engine]] · [[07_Firmware_Bridge]] · [[09_Cloud_Pipeline]] · [[10_Cloud_Security_Model]]

#tags: #ros2 #edge-compute #real-time #vigia-stack

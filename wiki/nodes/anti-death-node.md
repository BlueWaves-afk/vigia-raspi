---
title: "AntiDeathNode"
type: ros2-node
tags: [ros2-node, security, realtime]
source: vigia_ws/src/vigia_edge_node/src/anti_death_node.cpp
related: ["[[camera-node]]", "[[vision-node]]", "[[sensor-bridge-node]]", "[[fusion-node]]", "[[shm-ring-buffer]]", "[[frame-metadata-ring]]", "[[18650-ups]]", "[[sim7600]]", "[[https-rest]]", "[[aws-iot-mqtt]]", "[[ed25519-signing]]", "[[vigia-qos]]", "[[rt-thread]]", "[[params-yaml]]", "[[flow-anti-death]]", "[[adr-seqlock-ring]]"]
updated: 2026-06-19
---

# AntiDeathNode

**File:** `vigia_ws/src/vigia_edge_node/src/anti_death_node.cpp` / `anti_death_node.hpp`

Highest-priority node (SCHED_FIFO 99). Monitors UPS GPIO `POWER_FAIL` via libgpiod. On assertion executes a deterministic 15-second state machine: seqlock snapshot → serialize → HTTPS POST to AWS API Gateway. Never blocks on any mutex during the emergency sequence.

## Thread Configuration

| Property | Value |
|---|---|
| SCHED_FIFO Priority | **99** |
| CPU Core Affinity | **Core 3** |
| pthread Name | `vigia_antideath` |
| Stack Size | 512 KB (explicit — SnapshotData is 37 KB on stack) |

## Published Topics
None. Emergency output goes directly to AWS API Gateway via libcurl HTTPS POST (bypasses ROS2 graph).

## Subscribed Topics

| Topic | Callback Behavior |
|---|---|
| `/vigia/spatial_latent` | Double-buffer swap into `latest_latent_a_/b_` (atomic pointer) |
| `/vigia/signed_et` | Value copy into `latest_signed_et_` |
| `/vigia/hazard_event` | `unique_ptr` move into `latest_hazard_event_` |

All subscriber callbacks ONLY update cached state — no logic.

## GPIO Monitoring
- `libgpiod` `FALLING_EDGE` event on `/dev/gpiochip4` line 17 (active-low UPS signal)
- 1 ms `wall_timer` callback (`gpio_poll_callback`) — non-blocking `gpiod_line_event_wait`

## Emergency State Machine (budget from T=0)
| State | Budget | Action |
|---|---|---|
| `CAPTURING_SNAPSHOT` | T+0 → T+1.0s | Seqlock snapshot of `FrameMetadataRing` (~36 KB memcpy); freeze latent/SignedEt/HazardEvent pointers |
| `SERIALIZING` | T+1.0s → T+3.0s | MsgPack encode into 500 KB pre-reserved `sbuffer` |
| `HTTPS_TRANSMIT` | T+3.0s → T+13.0s | libcurl POST to AWS API Gateway `/telemetry`, Ed25519-signed JSON body |
| `SAFE_SHUTDOWN` | T+13.0s → T+15.0s | `::sync()` flush journald + `rclcpp::shutdown()` |

Note: Phase 5 target adds MQTT transmission via [[aws-iot-mqtt]] + [[sim7600]]. Phase 1 uses HTTPS REST directly via [[https-rest]].

## Startup Initialization (before emergency)
1. Pre-allocate latent double-buffers with `reserve(kMaxLatentElems)`
2. Read device_id from `/etc/vigia/device_id`
3. Initialize [[sim7600]] LTE (ECM mode, `usb0` interface)
4. Pre-connect Paho [[aws-iot-mqtt]] client (CONNACK blocking)
5. mmap [[shm-ring-buffer]] and [[frame-metadata-ring]]
6. Initialize libgpiod for [[18650-ups]] GPIO
7. Load Ed25519 key for [[ed25519-signing]] (libsodium)

## Signing (Ed25519, Phase 1)
- libsodium `crypto_sign_detached()` over JSON telemetry payload
- Key: 32-byte seed at `/etc/vigia/device_ed25519.key`
- See [[ed25519-signing]]

## Links
- Monitors: [[18650-ups]] (GPIO gpiochip4 line 17)
- Reads: [[shm-ring-buffer]] (wait-free seqlock snapshot)
- Reads: [[frame-metadata-ring]] (seqlock snapshot)
- Subscribes to: [[vision-node]] (spatial latent), [[sensor-bridge-node]] (SignedEt), [[fusion-node]] (HazardEvent)
- Transmits via: [[https-rest]] (Phase 1), [[aws-iot-mqtt]] (Phase 5)
- LTE via: [[sim7600]]
- Signs with: [[ed25519-signing]]
- Config: [[params-yaml]] `anti_death_node` section
- ADR: [[adr-seqlock-ring]], [[adr-preempt-rt]]
- Flow: [[flow-anti-death]]

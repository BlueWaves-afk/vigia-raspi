---
title: "VIGIA Edge Node — Map of Content"
type: index
tags: [decision, index]
source: .claude/design/01_system_architecture_and_roadmap.md
related: []
updated: 2026-06-19
---

# VIGIA ADAS DePIN Edge Node

Samsung Solve for Tomorrow 2026 — cryptographically attested road hazard detector running on Raspberry Pi 5 + Pico 2.

## ROS2 Nodes
- [[camera-node]] — Frame capture, seqlock ring buffer writer, Core 0 SCHED_FIFO 80
- [[vision-node]] — YOLO INT8 + spatial latent S_t, Core 1 SCHED_FIFO 75
- [[depth-node]] — MiDaS FP32 depth, Core 2 SCHED_FIFO 75
- [[fusion-node]] — Gravity-compensated ISS, Kalman, RRI scoring, Core 3 SCHED_FIFO 70
- [[sensor-bridge-node]] — Pico 2 COBS decoder, IMU/GPS/SignedEt publisher, Core 3 SCHED_FIFO 85
- [[anti-death-node]] — UPS GPIO monitor, 15s emergency sequence, Core 3 SCHED_FIFO 99
- [[ble-gatt-node]] — BlueZ D-Bus GATT peripheral for Android app link

## C++ Classes & Data Structures
- [[shm-ring-buffer]] — Seqlock-protected /dev/shm frame ring, 300 frames × 829 MB
- [[frame-metadata-ring]] — Compact per-frame metadata sidecar (124 B/frame × 300)
- [[safe-queue]] — Legacy std::mutex queue (deprecated, replaced by intra-process IPC)
- [[rt-thread]] — launch_rt_node() SCHED_FIFO thread launcher helper
- [[vigia-qos]] — QoS profile definitions (sensor_stream, camera_frames, hazard_events, etc.)
- [[vigia-rri]] — Shared RRI computation with graceful degradation (header-only)
- [[ble-frame-codec]] — BLE telemetry wire-format encoder/decoder (header-only)
- [[ble-gatt-constants]] — GATT service/characteristic UUIDs and protocol bytes

## Inference Engine
- [[onnx-runtime]] — ONNX Runtime 1.20.1 C++ sessions for YOLO + MiDaS
- [[yolo-int8]] — YOLOv26 Nano INT8 QUInt8 ONNX model (320×320, 84×2100 output)
- [[midas-fp32]] — MiDaS v2.1 small FP32 ONNX model (256×256 depth)
- [[kleidiai-acl]] — ARM Compute Library + KleidiAI UDOT EP (planned, not yet built)
- [[io-binding]] — Ort::IoBinding zero-copy pre-bound tensor hot path

## Firmware (Pico 2 / RP2350)
- [[bno085-driver]] — SPI0 DMA SHTP driver, 100 Hz quaternion + linear accel
- [[neo-m8n-driver]] — UART1 ring-buffer UBX NAV-PVT parser, 1-10 Hz GPS
- [[atecc608a-driver]] — I2C1 cryptoauthlib wrappers: atcab_sha + atcab_sign
- [[cobs-tx-driver]] — No-alloc COBS encoder, USB-CDC TinyUSB transmission
- [[no-heap]] — operator new/delete ban via static_assert at link time

## Hardware
- [[raspberry-pi-5]] — BCM2712, Cortex-A76 quad-core, 8 GB LPDDR4X
- [[pico-2]] — RP2350, Cortex-M33 @ 150 MHz, bare-metal firmware
- [[atecc608a]] — Secure element, I2C1, secp256r1 ECDSA signing
- [[bno085]] — 9-DOF IMU, SPI0 @ 3 MHz, NDOF fusion mode
- [[neo-m8n]] — GPS module, UART1 @ 9600 baud, UBX NAV-PVT
- [[sim7600]] — LTE CAT-4 modem, ECM USB network interface, AT command port
- [[18650-ups]] — UPS HAT, GPIO POWER_FAIL signal on gpiochip4 line 17
- [[camera]] — CSI camera module for Pi 5, V4L2 /dev/video0

## Transport
- [[aws-iot-mqtt]] — Paho async_client, TLS 1.2 mTLS, QoS 1, pre-connected
- [[https-rest]] — libcurl HTTPS POST to AWS API Gateway /telemetry (anti-death uplink)
- [[ble-gatt]] — BlueZ D-Bus GATT peripheral: TELEMETRY_CHAR notify, 5 Hz stream
- [[cobs-usb-cdc]] — COBS-framed binary packets over USB-CDC /dev/ttyACM0, 921600 baud
- [[tls-certs]] — mTLS certs: device_cert.pem, device_key.pem, ca_chain.pem

## Security
- [[ecdsa-signing]] — ATECC608A secp256r1 ECDSA-SHA256 over EtHashInput (96 bytes)
- [[ed25519-signing]] — libsodium Ed25519 device signing in AntiDeathNode
- [[et-hash-input]] — 96-byte packed struct: device_id + timestamp + IMU + GPS
- [[mbedtls-verify]] — Pi-side ECDSA signature verification (stub until SE wired)

## Config
- [[params-yaml]] — Unified ROS2 params: all 6 nodes in vigia_edge_node/config/params.yaml

## Flows
- [[flow-capture-to-uplink]] — Camera→Vision→Fusion→AntiDeath→AWS full pipeline
- [[flow-anti-death]] — UPS GPIO→Snapshot→Serialize→MQTT in 15-second window
- [[flow-sensor-bridge]] — Pico2→COBS→SensorBridgeNode→FusionNode signed E_t

## Architecture Decisions (ADRs)
- [[adr-preempt-rt]] — PREEMPT_RT + StaticSingleThreadedExecutor anti-priority-inversion
- [[adr-seqlock-ring]] — Seqlock /dev/shm ring buffer over std::mutex
- [[adr-onnx-vs-openvino]] — ONNX Runtime + KleidiAI replacing OpenVINO
- [[adr-gravity-compensated-iss]] — Quaternion gravity subtraction before ISS computation

---
title: "Flow: Capture → Vision → Fusion → Uplink"
type: flow
tags: [flow]
source: vigia_ws/src/vigia_edge_node/src
related: ["[[camera-node]]", "[[shm-ring-buffer]]", "[[vision-node]]", "[[yolo-int8]]", "[[io-binding]]", "[[depth-node]]", "[[midas-fp32]]", "[[fusion-node]]", "[[vigia-rri]]", "[[aws-iot-mqtt]]"]
updated: 2026-06-19
---

# Flow: Capture → Vision → Fusion → Uplink

The main perception pipeline, fully pinned and prioritized under SCHED_FIFO.

1. [[camera-node]] (Core 0, prio 80) grabs a frame from [[camera]], writes it into the
   [[shm-ring-buffer]] under the global seqlock, and stamps [[frame-metadata-ring]].
2. [[vision-node]] (Core 1, prio 75) reads the latest slot, runs [[yolo-int8]] via the
   pre-bound [[io-binding]] hot path, and extracts the spatial latent `S_t`.
3. [[depth-node]] (Core 2, prio 75) runs [[midas-fp32]] for per-pixel depth.
4. [[fusion-node]] (Core 3, prio 70) fuses detections + depth + IMU/GPS, computes the
   gravity-compensated ISS and the [[vigia-rri]] road-risk score, and emits a hazard event.
5. The hazard event is published QoS-1 over [[aws-iot-mqtt]] (mTLS via [[tls-certs]]) to AWS
   IoT Core for cloud attestation.

## Links
Nodes: [[camera-node]] → [[vision-node]] → [[depth-node]] → [[fusion-node]] → [[aws-iot-mqtt]]. Buffer: [[shm-ring-buffer]]. See ADRs [[adr-seqlock-ring]], [[adr-onnx-vs-openvino]], [[adr-gravity-compensated-iss]].

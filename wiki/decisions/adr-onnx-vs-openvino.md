---
title: "ADR: ONNX Runtime + KleidiAI over OpenVINO"
type: decision
tags: [decision]
source: vigia_ws/src/vigia_edge_node/src/vision_node.cpp
related: ["[[onnx-runtime]]", "[[yolo-int8]]", "[[midas-fp32]]", "[[kleidiai-acl]]", "[[io-binding]]", "[[vision-node]]", "[[depth-node]]"]
updated: 2026-06-19
---

# ADR: ONNX Runtime + KleidiAI over OpenVINO

**Status:** accepted (ONNX in production; KleidiAI ACL EP build pending on Pi).

**Context:** The hackathon baseline used OpenVINO 2025 with a forced FP32 precision hint —
not optimal for the Cortex-A76 and awkward to quantize for INT8 UDOT acceleration.

**Decision:** Standardize on [[onnx-runtime]] (1.20.1 C++) for both [[yolo-int8]] and
[[midas-fp32]], with the ARM Compute Library + [[kleidiai-acl|KleidiAI UDOT]] execution
provider as the INT8 fast path, and [[io-binding]] for zero-copy pre-bound tensors on the
hot loop.

**Consequence:** Portable ARM-native INT8 inference; the ACL EP is gated behind a
`/proc/cpuinfo asimddp` CPUID check before use.

## Links
Powers [[vision-node]] and [[depth-node]]; see [[flow-capture-to-uplink]].

---
title: "ADR: Gravity-Compensated ISS"
type: decision
tags: [decision]
source: vigia_ws/src/vigia_edge_node/src/fusion_node.cpp
related: ["[[fusion-node]]", "[[bno085]]", "[[bno085-driver]]", "[[neo-m8n]]", "[[vigia-rri]]", "[[flow-sensor-bridge]]"]
updated: 2026-06-19
---

# ADR: Gravity-Compensated ISS

**Status:** accepted (implemented in [[fusion-node]]).

**Context:** Computing Impact Severity Score as `ISS = Z_accel / v_GPS` is physically wrong:
the [[bno085]] accelerometer includes the 1g gravity vector, so on any incline gravity
projects onto Z and produces false ISS spikes even at constant velocity (highway expansion
joints false-trigger).

**Decision:** Use the BNO085 unit quaternion to rotate the accel vector into the world frame
(quaternion sandwich), subtract `[0,0,9.81]`, then compute
`ISS = |a_world.z_detrended| / max(v_GPS, v_min_ms)` with `v_min_ms` preventing divide-by-zero
at rest. The same world-frame accel feeds the Kalman predict step.

**Consequence:** ISS reflects real road impacts, feeding a trustworthy [[vigia-rri]] score.

## Links
Implemented in [[fusion-node]]; inputs via [[flow-sensor-bridge]]; tunable `v_min_ms` in [[params-yaml]].

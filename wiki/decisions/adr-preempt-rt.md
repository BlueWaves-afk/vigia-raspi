---
title: "ADR: PREEMPT_RT + StaticSingleThreadedExecutor"
type: decision
tags: [decision]
source: .claude/design/01_system_architecture_and_roadmap.md
related: ["[[rt-thread]]", "[[camera-node]]", "[[vision-node]]", "[[fusion-node]]", "[[anti-death-node]]", "[[sensor-bridge-node]]"]
updated: 2026-06-19
---

# ADR: PREEMPT_RT + StaticSingleThreadedExecutor

**Status:** accepted (kernel installed; boot flag pending hardware access).

**Context:** ROS2's default executors use internal `std::mutex` constructs that are not
priority-inheritance-aware, causing priority inversion under PREEMPT_RT — the exact failure
mode RT systems exist to avoid.

**Decision:** Run a PREEMPT_RT kernel and launch each node on a dedicated `std::thread` with
`pthread_setschedparam(SCHED_FIFO, prio)` + `pthread_setaffinity_np` via the [[rt-thread]]
helper, using `StaticSingleThreadedExecutor` (no dynamic allocation on spin). Never
`rclcpp::spin()` an RT node on the main thread.

**Priority ladder:** [[anti-death-node]] 99 > [[sensor-bridge-node]] 85 > [[camera-node]] 80 >
[[vision-node]]/[[depth-node]] 75 > [[fusion-node]] 70.

## Links
Implemented by [[rt-thread]]; governs all [[index|ROS2 nodes]].

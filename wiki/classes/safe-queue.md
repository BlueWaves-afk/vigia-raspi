---
title: "SafeQueue (Deprecated)"
type: cpp-class
tags: [cpp-class]
source: include/safe_queue.hpp
related: ["[[camera-node]]", "[[vision-node]]", "[[adr-preempt-rt]]"]
updated: 2026-06-19
---

# SafeQueue (Deprecated)

**File:** `include/safe_queue.hpp`

Legacy Phase 0 inter-thread queue using `std::mutex` + `std::condition_variable` + `frame.clone()` heap copy. Used in the original CFS/pthread coordinator (`src/coordinator.cpp`).

**Status: DEPRECATED** — replaced entirely by `rclcpp::intra_process_comm` zero-copy `std::unique_ptr` handoffs in Phase 1. See [[adr-preempt-rt]].

## Why Deprecated
- `std::mutex` inside a ROS2 executor causes **priority inversion** under PREEMPT_RT (documented in `.claude/design/01_system_architecture_and_roadmap.md §5 Improvement 1`)
- `frame.clone()` causes per-frame heap allocation — eliminated by zero-copy intra-process

## Links
- Replaced by: [[vigia-qos]] intra-process transport
- ADR: [[adr-preempt-rt]]

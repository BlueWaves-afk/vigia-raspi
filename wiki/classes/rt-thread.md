---
title: "RtThread (launch_rt_node)"
type: cpp-class
tags: [cpp-class, realtime]
source: vigia_ws/src/vigia_edge_node/include/vigia_edge_node/rt_thread.hpp
related: ["[[camera-node]]", "[[vision-node]]", "[[depth-node]]", "[[fusion-node]]", "[[sensor-bridge-node]]", "[[anti-death-node]]", "[[adr-preempt-rt]]"]
updated: 2026-06-19
---

# RtThread — `launch_rt_node()`

**File:** `vigia_ws/src/vigia_edge_node/include/vigia_edge_node/rt_thread.hpp`

Header-only helper that launches a ROS2 node on a dedicated `std::thread` with SCHED_FIFO scheduling and CPU core pinning.

## `RtThreadConfig` struct
```cpp
struct RtThreadConfig {
    int sched_priority;   // SCHED_FIFO priority 1–99
    int cpu_core;         // CPU affinity 0–3
    std::string name;     // pthread name (max 15 chars)
};
```

## `launch_rt_node(node, cfg)` → `std::thread`
1. `pthread_setname_np(pthread_self(), cfg.name.c_str())`
2. `pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)` — pin to `cfg.cpu_core`
3. `pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)` — elevate priority (requires CAP_SYS_NICE or root); logs RCLCPP_WARN and continues on failure (dev environment)
4. `StaticSingleThreadedExecutor::add_node(node) + spin()`

## `make_ipc_options()` helper
Returns `rclcpp::NodeOptions` with `use_intra_process_comms(true)` — required for all 6 nodes.

## Priority Ladder (main.cpp launch order: highest first)
| Node | Priority | Core | Name |
|---|---|---|---|
| [[anti-death-node]] | 99 | 3 | `vigia_antideath` |
| [[sensor-bridge-node]] | 85 | 3 | `vigia_bridge` |
| [[camera-node]] | 80 | 0 | `vigia_camera` |
| [[vision-node]] | 75 | 1 | `vigia_vision` |
| [[depth-node]] | 75 | 2 | `vigia_depth` |
| [[fusion-node]] | 70 | 3 | `vigia_fusion` |

## Links
- Used by all 6 nodes
- ADR: [[adr-preempt-rt]]

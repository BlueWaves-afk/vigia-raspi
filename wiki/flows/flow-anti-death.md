---
title: "Flow: Anti-Death (power-loss 15s window)"
type: flow
tags: [flow, anti-death]
source: vigia_ws/src/vigia_edge_node/src/anti_death_node.cpp
related: ["[[18650-ups]]", "[[anti-death-node]]", "[[shm-ring-buffer]]", "[[ed25519-signing]]", "[[aws-iot-mqtt]]", "[[https-rest]]", "[[adr-seqlock-ring]]"]
updated: 2026-06-19
---

# Flow: Anti-Death

The safety-critical emergency uplink triggered when main power is lost.

1. [[18650-ups]] asserts the `POWER_FAIL` signal on `gpiochip4` line 17.
2. [[anti-death-node]] (Core 3, prio 99 — highest) catches the libgpiod v2 edge event.
3. It takes a wait-free bulk snapshot of the [[shm-ring-buffer]] via the global seqlock
   (`snapshot_all()`) — no mutex, safe to call from the high-priority handler (see
   [[adr-seqlock-ring]]).
4. The snapshot + last hazard state is serialized and [[ed25519-signing|signed]].
5. Transmitted within the ~15-second UPS hold window: QoS-1 via [[aws-iot-mqtt]] (preferred,
   mTLS) or [[https-rest]] fallback.

## Links
Trigger: [[18650-ups]] → handler: [[anti-death-node]] → reads [[shm-ring-buffer]] → [[ed25519-signing]] → [[aws-iot-mqtt]] / [[https-rest]].

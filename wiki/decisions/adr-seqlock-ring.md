---
title: "ADR: Seqlock /dev/shm Ring Buffer"
type: decision
tags: [decision]
source: vigia_ws/src/vigia_edge_node/include/vigia_edge_node/shm_ring_buffer.hpp
related: ["[[shm-ring-buffer]]", "[[camera-node]]", "[[anti-death-node]]", "[[flow-anti-death]]", "[[adr-preempt-rt]]"]
updated: 2026-06-19
---

# ADR: Seqlock /dev/shm Ring Buffer

**Status:** accepted (implemented, incl. global seqlock bulk snapshot).

**Context:** The frame ring has a concurrent writer ([[camera-node]], prio 80) and a
snapshot reader ([[anti-death-node]], prio 99). Taking a `std::mutex` from the high-priority
handler risks deadlock/priority inversion under [[adr-preempt-rt|PREEMPT_RT]].

**Decision:** Use a seqlock. The writer brackets each write with an atomic counter
(odd = writing, even = stable). A **global** seqlock additionally wraps the whole ring so
`snapshot_all()` captures all slots as one consistent bulk copy rather than N independent
per-slot reads. The reader spins until the counter is even, copies, then re-checks — fully
wait-free.

**Consequence:** [[anti-death-node]] gets a lock-free snapshot path; see [[flow-anti-death]].

## Links
Realized in [[shm-ring-buffer]]; consumed by [[flow-anti-death]].

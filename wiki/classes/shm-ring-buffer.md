---
title: "ShmRingBuffer"
type: cpp-class
tags: [cpp-class, realtime, memory]
source: vigia_ws/src/vigia_edge_node/include/vigia_edge_node/shm_ring_buffer.hpp
related: ["[[camera-node]]", "[[anti-death-node]]", "[[frame-metadata-ring]]", "[[adr-seqlock-ring]]"]
updated: 2026-06-19
---

# ShmRingBuffer

**File:** `vigia_ws/src/vigia_edge_node/include/vigia_edge_node/shm_ring_buffer.hpp`

Seqlock-protected rolling frame ring buffer backed by POSIX shared memory (`/dev/shm/vigia_frame_ring`). Single-writer (CameraNode SCHED_FIFO 80), single wait-free snapshot reader (AntiDeathNode SCHED_FIFO 99).

## Key Types

### `ShmFrameSlot`
```cpp
struct ShmFrameSlot {
    std::atomic<uint32_t> seq{0};   // seqlock: even=stable, odd=writing
    uint64_t timestamp_us{0};
    uint32_t frame_id{0};
    uint32_t width{0}, height{0}, channels{3};
    // pixel data follows in-place
};
```

### `ShmRingBuffer` class
- Constructor: `shm_open("/vigia_frame_ring", O_CREAT|O_RDWR)` + `ftruncate` + `mmap(MAP_SHARED)`
- Total size: 300 × (sizeof(ShmFrameSlot) + 1280×720×3) ≈ **829 MB**
- `mlockall(MCL_CURRENT|MCL_FUTURE)` called at process start — prevents page faults during emergency

## Key Methods

### `write_frame(frame_id, ts_us, bgr_data, data_bytes)` — CameraNode writer
```
seq.fetch_add(1, release)  // mark odd (writing)
copy fields + pixel data
seq.fetch_add(1, release)  // mark even (stable)
```

### `snapshot(slot_idx, out_meta, out_pixels, pixels_cap) → bool` — AntiDeathNode reader
```
seq1 = load(acquire)
if (seq1 & 1) return false  // writer in progress
copy fields + pixels
seq2 = load(acquire)
return (seq1 == seq2)       // false → concurrent write → caller retries
```

## Seqlock Invariant
- `seq` even → buffer slot stable → reader may copy
- `seq` odd → writer mid-write → reader must retry
- No mutex; no blocking; O(1) wait-free read path for SCHED_FIFO 99 AntiDeathNode

## Links
- Writer: [[camera-node]] (SCHED_FIFO 80)
- Snapshot reader: [[anti-death-node]] (SCHED_FIFO 99)
- Related: [[frame-metadata-ring]] (parallel sidecar ring)
- ADR: [[adr-seqlock-ring]]

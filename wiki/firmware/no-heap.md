---
title: "no_heap — Dynamic Allocation Ban"
type: firmware
tags: [firmware]
source: firmware/src/no_heap.cpp
related: ["[[pico-2]]", "[[atecc608a-driver]]", "[[cobs-tx-driver]]"]
updated: 2026-06-19
---

# no_heap.cpp — Link-Time Allocation Ban

**File:** `firmware/src/no_heap.cpp`  
**Status:** Written

Overrides `operator new` and `operator delete` with `static_assert(false)` to produce a **compile-time or link-time error** if any C++ code in the firmware attempts to use dynamic allocation.

## Invariant
From `firmware/src/03_pico2_firmware_contracts.md §0`:
> `new`, `delete`, `malloc`, `free`, `std::vector`, `std::string` are **banned**. All buffers are `static` or stack-allocated with known, bounded sizes.

## Why
Bare-metal embedded system (RP2350, 520 KiB SRAM). No heap allocator configured. Dynamic allocation would cause undefined behavior or silent corruption at runtime. The link-time ban makes violations a build error.

## Verification
```bash
nm build/vigia_pico2.elf | grep ' malloc\b'   # must be zero results
```

## Links
- Enforced on: [[pico-2]] firmware
- Related: [[atecc608a-driver]] (`ATCA_NO_HEAP`), [[cobs-tx-driver]] (static buffers)

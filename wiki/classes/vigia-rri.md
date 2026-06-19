---
title: "vigia_rri — RRI Computation"
type: cpp-class
tags: [cpp-class, fusion]
source: vigia_ws/src/vigia_edge_node/include/vigia_edge_node/vigia_rri.hpp
related: ["[[fusion-node]]", "[[ble-gatt-node]]", "[[adr-gravity-compensated-iss]]"]
updated: 2026-06-19
---

# vigia_rri — Road Risk Index Computation

**File:** `vigia_ws/src/vigia_edge_node/include/vigia_edge_node/vigia_rri.hpp`

Header-only, no ROS2 dependency — unit-testable on host. Provides `compute_rri()` with graceful degradation: absent input terms are excluded and remaining weights renormalized to sum to 1.

## `RriWeights` struct
```cpp
struct RriWeights {
    float w_yolo     {0.35f};
    float w_geometry {0.25f};
    float w_temporal {0.15f};
    float w_iss      {0.25f};
};
```

## `RriInputs` struct
Each term has a `have_*` flag. Absent terms (flag=false) are excluded from sum.
```cpp
struct RriInputs {
    bool have_yolo; float yolo_conf;    // best YOLO detection confidence [0,1]
    bool have_geometry; float geo_conf; // plane-fit depression confidence [0,1]
    bool have_temporal; float temp_conf;// persistence/stability [0,1]
    bool have_iss; float iss_norm;      // normalized gravity-compensated ISS [0,1]
};
```

## `RriTier` enum
`kNone / kVisionOnly / kVisionDepth / kFull` — surfaced on HazardEvent so downstream knows degradation level.

## `compute_rri(weights, inputs) → float`
```
RRI = renormalized_weighted_sum(present_terms)
    = Σ(w_i × clamp01(term_i)) / Σ(w_i)   for all present terms
```

## Graceful Degradation
- Vision-only (no IMU/GPS): RRI = yolo_conf weighted over yolo weight
- Full system: all 4 terms weighted as above
- Prevents FusionNode from producing zero RRI when Pico 2 is in stub/bringup mode

## Links
- Used by: [[fusion-node]] (primary consumer), [[ble-gatt-node]] (continuous stream)
- ADR: [[adr-gravity-compensated-iss]]

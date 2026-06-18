#pragma once
//
// vigia_rri.hpp — shared Road-Risk-Index (RRI) computation.
//
// One implementation, two consumers (design spec §6.3):
//   * FusionNode      — full fusion, threshold-gated -> AWS DePIN hazard uplink
//   * BleGattNode     — continuous stream RRI -> Android companion app
//
// The key property is GRACEFUL DEGRADATION: each term is optional. When an
// input is missing (e.g. no IMU because the Pico is in stub mode) its weight
// is dropped and the remaining weights are renormalized to sum to 1. So a
// vision-only system still produces a meaningful RRI = yolo_conf, instead of
// FusionNode going dead the moment the IMU is absent (the old G5 bug).
//
// Header-only, no ROS2 dependency: host-unit-testable. See
// test/test_vigia_rri.cpp.
//

#include <cstdint>

namespace vigia {

// Default fusion weights (match FusionNode parameter defaults).
struct RriWeights {
    float w_yolo     {0.35f};
    float w_geometry {0.25f};
    float w_temporal {0.15f};
    float w_iss      {0.25f};
};

// Each term carries a presence flag; absent terms are excluded from the
// weighted sum and trigger renormalization over the present weights.
struct RriInputs {
    bool  have_yolo     {false}; float yolo_conf {0.0f};  // best detection confidence [0,1]
    bool  have_geometry {false}; float geo_conf  {0.0f};  // plane-fit depression confidence [0,1]
    bool  have_temporal {false}; float temp_conf {0.0f};  // persistence/stability [0,1]
    bool  have_iss      {false}; float iss_norm  {0.0f};  // normalized impact-shock score [0,1]
};

// Which sensing tier produced an RRI — surfaced on HazardEvent so downstream
// consumers know whether the value is hardware-complete or degraded.
enum class RriTier : uint8_t {
    kNone        = 0,  // nothing available -> rri == 0
    kVisionOnly  = 1,  // yolo only
    kVisionDepth = 2,  // yolo + geometry/temporal, no IMU/GPS
    kFull        = 3,  // all terms present
};

inline float clamp01(float v) {
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

// Weighted, renormalized RRI over the present terms. Returns 0 if nothing present.
inline float compute_rri(const RriWeights & w, const RriInputs & in) {
    float num = 0.0f;
    float den = 0.0f;
    if (in.have_yolo)     { num += w.w_yolo     * clamp01(in.yolo_conf); den += w.w_yolo;     }
    if (in.have_geometry) { num += w.w_geometry * clamp01(in.geo_conf);  den += w.w_geometry; }
    if (in.have_temporal) { num += w.w_temporal * clamp01(in.temp_conf); den += w.w_temporal; }
    if (in.have_iss)      { num += w.w_iss      * clamp01(in.iss_norm);  den += w.w_iss;      }
    if (den <= 0.0f) return 0.0f;
    return clamp01(num / den);
}

// Classify the sensing tier from which inputs were present.
inline RriTier classify_tier(const RriInputs & in) {
    const bool depth = in.have_geometry || in.have_temporal;
    if (in.have_iss && in.have_yolo)       return RriTier::kFull;
    if (in.have_yolo && depth)             return RriTier::kVisionDepth;
    if (in.have_yolo)                      return RriTier::kVisionOnly;
    return RriTier::kNone;
}

}  // namespace vigia

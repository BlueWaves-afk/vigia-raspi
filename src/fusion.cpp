#include "fusion.hpp"

#include <algorithm>
#include <cmath>

namespace vigia {

/* ===================== Helpers ===================== */

static inline float clamp01(float v) {
    return std::clamp(v, 0.0f, 1.0f);
}

/* ===================== Constructor ===================== */

FusionEngine::FusionEngine() = default;

/* ===================== Geometry Confidence ===================== */
/*
High when:
- Depression is strong
- Surface is NOT noisy (low roughness)
*/

float FusionEngine::computeGeometryConfidence(
    float depression,
    float roughness
) const {
    // Normalize depression into [0,1]
    const float dep = clamp01(depression);

    // Penalize rough surfaces (noise ≠ pothole)
    const float roughPenalty = std::exp(-roughness * 10.0f);

    return clamp01(dep * roughPenalty);
}

/* ===================== Temporal Confidence ===================== */
/*
High when:
- Signal persists
- Signal is stable
*/

float FusionEngine::computeTemporalConfidence(
    float persistence,
    float stability
) const {
    // Persistence already mean/std normalized → squash
    const float p = std::tanh(persistence * 0.1f);

    // Stability is inverse variance → squash
    const float s = std::tanh(stability * 0.01f);

    return clamp01(0.5f * p + 0.5f * s);
}

/* ===================== Public Fusion ===================== */

FusionOutput FusionEngine::fuse(
    const FusionInput& in
) const {
    FusionOutput out{};

    out.geometryConfidence =
        computeGeometryConfidence(
            in.depressionScore,
            in.roughness
        );

    out.temporalConfidence =
        computeTemporalConfidence(
            in.persistence,
            in.stability
        );

    // Tunable weights (EXPLICIT BY DESIGN)
    constexpr float W_DET = 0.40f;
    constexpr float W_GEO = 0.35f;
    constexpr float W_TMP = 0.25f;

    out.finalConfidence = clamp01(
        W_DET * clamp01(in.yoloConfidence) +
        W_GEO * out.geometryConfidence +
        W_TMP * out.temporalConfidence
    );

    return out;
}

} // namespace vigia

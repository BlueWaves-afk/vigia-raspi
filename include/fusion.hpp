#pragma once

namespace vigia {

/* ===================== Inputs ===================== */

struct FusionInput {
    float yoloConfidence{0.0f};     // detector confidence [0,1]
    float depressionScore{0.0f};    // from analytical
    float roughness{0.0f};          // from analytical
    float persistence{0.0f};        // from temporal
    float stability{0.0f};          // from temporal
};

/* ===================== Outputs ===================== */

struct FusionOutput {
    float geometryConfidence{0.0f};
    float temporalConfidence{0.0f};
    float finalConfidence{0.0f};
};

/* ===================== Fusion Engine ===================== */

class FusionEngine {
public:
    FusionEngine();
    virtual ~FusionEngine() = default;

    virtual FusionOutput fuse(const FusionInput& in) const;

private:
    float computeGeometryConfidence(
        float depression,
        float roughness
    ) const;

    float computeTemporalConfidence(
        float persistence,
        float stability
    ) const;
};

} // namespace vigia

#pragma once

namespace vigia {

/* ===================== Inputs ===================== */

struct FusionInput {
    float yoloConfidence{0.0f};     // detector confidence [0,1]
    float depressionScore{0.0f};    // from analytical
    float roughness{0.0f};          // from analytical
    float persistence{0.0f};        // from temporal
    float stability{0.0f};          // from temporal

    // M6: IMU-derived impact severity, already normalized to [0,1] by coordinator
    float imuIss{0.0f};
    // M6: GPS speed in m/s — used for ISS motion gating inside FusionEngine
    float speedMs{0.0f};
    // M6: pre-validated GPS position (caller checks fix_type >= 2, hdop <= 2.5)
    double gpsLat{0.0};
    double gpsLon{0.0};
    bool gpsValid{false};
};

/* ===================== Outputs ===================== */

struct FusionOutput {
    float geometryConfidence{0.0f};
    float temporalConfidence{0.0f};
    float finalConfidence{0.0f};

    // M6: geo-tag — populated only when gpsValid was true in the input
    double latitude{0.0};
    double longitude{0.0};
    float speedMs{0.0f};
    bool gpsValid{false};
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

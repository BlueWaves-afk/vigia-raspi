#pragma once

#include "iss_filter.hpp"
#include "sensor_state.hpp"

namespace vigia {

/*
 * SensorProcessor — ISS pipeline + GPS gate for one fusion frame.
 *
 * Separated from Coordinator so the same logic drives:
 *   - live fusion (Coordinator::querySensors)
 *   - bench diagnostics (sensor_fusion_live_test)
 *   - future replay harness
 */
class SensorProcessor {
public:
    struct Config {
        float gpsMaxHdop{2.5f};
        uint8_t gpsMinFixType{2};
        IssFilter::Config filter{};
    };

    struct Snapshot {
        float imuIss{0.0f};       // value fed to FusionEngine (0 unless genuine impact)
        float speedMs{0.0f};
        double gpsLat{0.0};
        double gpsLon{0.0};
        bool gpsValid{false};

        // Diagnostics — useful for bench / field logging
        bool imuValid{false};
        float rawVerticalMs2{0.0f};
        float normalizedIss{0.0f};
        IssFilter::Result filter{};
    };

    explicit SensorProcessor(Config cfg = Config{});

    Snapshot process(const SensorState& state) const;
    void reset();

    const Config& config() const { return cfg_; }

private:
    Config cfg_;
    mutable IssFilter filter_;
};

} // namespace vigia

#pragma once

#include "sensor_packet.hpp"

namespace vigia {

/* Tunable ISS constants — shared by coordinator, tests, and live diagnostics */
struct IssParams {
    static constexpr float kIssMax = 3.0f;  // ~3 g vertical impulse saturates normalized ISS
    static constexpr float kVMin   = 2.0f;  // m/s speed floor when GPS unavailable (~7 km/h)
};

/*
 * Rotate body-frame linear acceleration into world frame and return |a_world.z|.
 *
 * The BNO085 Linear Acceleration report already removes gravity in firmware;
 * do NOT subtract 9.81 again here.
 */
float computeVerticalImpulse(const ImuSample& imu);

// ISS = |awz| / max(v_gps, V_MIN), clamped to [0, 1] via ISS_MAX
float normalizeIss(float verticalImpulseMs2, float speedMs);

// GPS quality gate used before geo-tagging and speed for ISS normalization
bool isGpsFixUsable(const GpsFix& fix, float maxHdop = 2.5f, uint8_t minFixType = 2);

} // namespace vigia

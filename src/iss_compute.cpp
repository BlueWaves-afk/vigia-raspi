#include "iss_compute.hpp"

#include <algorithm>
#include <cmath>

namespace vigia {

float computeVerticalImpulse(const ImuSample& imu)
{
    const float qw = imu.qw, qx = imu.qx, qy = imu.qy, qz = imu.qz;
    const float ax = imu.ax, ay = imu.ay, az = imu.az;

    // Rodrigues formula — compute only world-Z component
    const float tx = qy * az - qz * ay;
    const float ty = qz * ax - qx * az;
    const float tz = qx * ay - qy * ax;

    const float awz = az + 2.0f * qw * tz + 2.0f * (qx * ty - qy * tx);
    return std::fabsf(awz);
}

float normalizeIss(float verticalImpulseMs2, float speedMs)
{
    const float iss = verticalImpulseMs2 / std::max(speedMs, IssParams::kVMin);
    return std::min(iss / IssParams::kIssMax, 1.0f);
}

bool isGpsFixUsable(const GpsFix& fix, float maxHdop, uint8_t minFixType)
{
    return fix.valid && fix.fix_type >= minFixType && fix.hdop <= maxHdop;
}

} // namespace vigia

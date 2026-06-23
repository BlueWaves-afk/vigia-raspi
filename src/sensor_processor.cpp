#include "sensor_processor.hpp"

#include "iss_compute.hpp"

namespace vigia {

SensorProcessor::SensorProcessor()
    : SensorProcessor(Config{})
{}

SensorProcessor::SensorProcessor(Config cfg)
    : cfg_(std::move(cfg)),
      filter_(cfg_.filter)
{}

SensorProcessor::Snapshot SensorProcessor::process(const SensorState& state) const
{
    Snapshot snap{};

    const auto gps = state.getLatestGps();
    if (gps && isGpsFixUsable(*gps, cfg_.gpsMaxHdop, cfg_.gpsMinFixType)) {
        snap.speedMs  = gps->speed_ms;
        snap.gpsLat   = gps->latitude;
        snap.gpsLon   = gps->longitude;
        snap.gpsValid = true;
    }

    const auto imu = state.getLatestImu();
    if (!imu || !imu->valid)
        return snap;

    snap.imuValid = true;
    snap.rawVerticalMs2 = computeVerticalImpulse(*imu);
    snap.normalizedIss  = normalizeIss(snap.rawVerticalMs2, snap.speedMs);

    snap.filter = filter_.update(snap.normalizedIss, snap.speedMs);
    snap.imuIss = snap.filter.isGenuineImpact ? snap.filter.detrendedIss : 0.0f;

    return snap;
}

void SensorProcessor::reset()
{
    filter_.reset();
}

} // namespace vigia

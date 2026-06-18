#pragma once

#include "sensor_packet.hpp"
#include "signed_et_packet.hpp"

#include <array>
#include <cstddef>
#include <mutex>
#include <optional>

namespace vigia {

class SensorState {
public:
    static constexpr std::size_t kImuHistorySize = 200;

    void updateImu(const ImuSample& sample);
    void updateGps(const GpsFix& fix);
    void updateSignedEt(const SignedEtSample& sample);
    void updateHealth(const SensorHealth& health);

    std::optional<ImuSample> getLatestImu() const;
    std::optional<GpsFix> getLatestGps() const;
    std::optional<SignedEtSample> getLatestSignedEt() const;
    SensorHealth getHealth() const;

    std::optional<ImuSample> getSampleAtOrBefore(uint64_t timestamp_us) const;

private:
    mutable std::mutex mutex_;

    std::optional<ImuSample> latest_imu_;
    std::optional<GpsFix> latest_gps_;
    std::optional<SignedEtSample> latest_signed_et_;
    SensorHealth health_{};

    std::array<ImuSample, kImuHistorySize> imu_history_{};
    std::size_t imu_history_head_{0};
    std::size_t imu_history_count_{0};
};

} // namespace vigia

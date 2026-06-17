#include "sensor_state.hpp"

namespace vigia {

void SensorState::updateImu(const ImuSample& sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_imu_ = sample;
    imu_history_[imu_history_head_] = sample;
    imu_history_head_ = (imu_history_head_ + 1) % kImuHistorySize;
    if (imu_history_count_ < kImuHistorySize)
        ++imu_history_count_;
}

void SensorState::updateGps(const GpsFix& fix) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_gps_ = fix;
}

void SensorState::updateHealth(const SensorHealth& health) {
    std::lock_guard<std::mutex> lock(mutex_);
    health_ = health;
}

std::optional<ImuSample> SensorState::getLatestImu() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_imu_;
}

std::optional<GpsFix> SensorState::getLatestGps() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_gps_;
}

SensorHealth SensorState::getHealth() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return health_;
}

std::optional<ImuSample> SensorState::getSampleAtOrBefore(uint64_t timestamp_us) const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (imu_history_count_ == 0)
        return std::nullopt;

    for (std::size_t age = 0; age < imu_history_count_; ++age) {
        const std::size_t idx =
            (imu_history_head_ + kImuHistorySize - 1 - age) % kImuHistorySize;
        const ImuSample& sample = imu_history_[idx];
        if (sample.timestamp_us <= timestamp_us)
            return sample;
    }

    return std::nullopt;
}

} // namespace vigia

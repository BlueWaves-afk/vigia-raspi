#include "event_promoter.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <thread>

namespace vigia {

namespace {

constexpr double kEarthRadiusM = 6371000.0;
constexpr uint8_t kMinGpsFixType = 2;

double haversineMeters(double lat1, double lon1, double lat2, double lon2)
{
    const double lat1r = lat1 * M_PI / 180.0;
    const double lat2r = lat2 * M_PI / 180.0;
    const double dlat  = (lat2 - lat1) * M_PI / 180.0;
    const double dlon  = (lon2 - lon1) * M_PI / 180.0;

    const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                     std::cos(lat1r) * std::cos(lat2r) *
                     std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return kEarthRadiusM * c;
}

} // namespace

EventPromoter::EventPromoter(Config cfg)
    : cfg_(cfg),
      ring_capacity_(std::min(std::max(cfg.ring_capacity, std::size_t{2}),
                              ring_.size())),
      start_time_(std::chrono::steady_clock::now())
{
    // ring_capacity_ must be at least 2 (head != next prevents a 1-slot ring
    // from ever accepting any item). Enforce this regardless of caller input so
    // enqueue() can never produce a modulo-zero fault.
    std::strncpy(cfg_.device_id, cfg.device_id, sizeof(cfg_.device_id) - 1);
    cfg_.device_id[sizeof(cfg_.device_id) - 1] = '\0';
}

bool EventPromoter::submit(const HazardObservation& candidate)
{
    if (!passesGates(candidate))
        return false;

    if (isDuplicate(candidate))
        return false;

    HazardObservation obs = candidate;
    obs.timestamp_us = monotonicNowUs();
    obs.device_seq = device_seq_ + 1;
    generateEventId(obs.event_id);
    std::strncpy(obs.device_id, cfg_.device_id, sizeof(obs.device_id) - 1);
    obs.device_id[sizeof(obs.device_id) - 1] = '\0';

    if (!enqueue(obs))
        return false;

    ++device_seq_;
    recordDedup(obs);
    return true;
}

bool EventPromoter::passesGates(const HazardObservation& obs) const
{
    if (obs.rri < cfg_.rri_threshold)
        return false;

    if (obs.geometry_conf <= 0.0f)
        return false;

    if (!cfg_.require_gps)
        return true;

    if (!obs.gps_valid)
        return false;

    if (obs.gps_fix_type < kMinGpsFixType)
        return false;

    if (obs.hdop > cfg_.max_hdop)
        return false;

    return true;
}

bool EventPromoter::isDuplicate(const HazardObservation& obs) const
{
    pruneDedup(obs.timestamp_us ? obs.timestamp_us : monotonicNowUs());

    for (std::size_t i = 0; i < dedup_count_; ++i) {
        const auto& entry = dedup_[i];
        const double dist = haversineMeters(obs.lat, obs.lon, entry.lat, entry.lon);
        if (dist <= static_cast<double>(cfg_.dedup_radius_m))
            return true;
    }
    return false;
}

void EventPromoter::recordDedup(const HazardObservation& obs)
{
    pruneDedup(obs.timestamp_us);

    if (dedup_count_ < kDedupHistory) {
        dedup_[dedup_count_++] = DedupEntry{obs.lat, obs.lon, obs.timestamp_us};
        return;
    }

    for (std::size_t i = 1; i < dedup_count_; ++i)
        dedup_[i - 1] = dedup_[i];
    dedup_[dedup_count_ - 1] = DedupEntry{obs.lat, obs.lon, obs.timestamp_us};
}

void EventPromoter::pruneDedup(uint64_t now_us) const
{
    const uint64_t window_us =
        static_cast<uint64_t>(cfg_.dedup_window_s * 1'000'000.0f);

    std::size_t write = 0;
    for (std::size_t i = 0; i < dedup_count_; ++i) {
        if (now_us - dedup_[i].timestamp_us <= window_us)
            dedup_[write++] = dedup_[i];
    }
    dedup_count_ = write;
}

bool EventPromoter::enqueue(HazardObservation obs)
{
    const std::size_t head = head_.load(std::memory_order_acquire);
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    const std::size_t next = (tail + 1) % ring_capacity_;

    if (next == head)
        return false;

    ring_[tail] = obs;
    tail_.store(next, std::memory_order_release);
    return true;
}

bool EventPromoter::try_dequeue(HazardObservation& out)
{
    std::lock_guard<std::mutex> lock(consumer_mutex_);

    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t tail = tail_.load(std::memory_order_acquire);

    if (head == tail)
        return false;

    out = ring_[head];
    head_.store((head + 1) % ring_capacity_, std::memory_order_release);
    return true;
}

bool EventPromoter::wait_dequeue(
    HazardObservation& out,
    std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < deadline) {
        if (try_dequeue(out))
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

std::size_t EventPromoter::pending_count() const
{
    const std::size_t head = head_.load(std::memory_order_acquire);
    const std::size_t tail = tail_.load(std::memory_order_acquire);
    if (tail >= head)
        return tail - head;
    return ring_capacity_ - head + tail;
}

void EventPromoter::generateEventId(uint8_t out[16]) const
{
    const auto now = std::chrono::system_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();

    out[0] = static_cast<uint8_t>((ms >> 40) & 0xFF);
    out[1] = static_cast<uint8_t>((ms >> 32) & 0xFF);
    out[2] = static_cast<uint8_t>((ms >> 24) & 0xFF);
    out[3] = static_cast<uint8_t>((ms >> 16) & 0xFF);
    out[4] = static_cast<uint8_t>((ms >> 8) & 0xFF);
    out[5] = static_cast<uint8_t>(ms & 0xFF);

    out[6] = 0x70 | static_cast<uint8_t>((ms >> 8) & 0x0F);
    out[7] = static_cast<uint8_t>((ms >> 16) & 0xFF);

    thread_local std::mt19937_64 rng{std::random_device{}()};
    const uint64_t rand = rng();
    out[8]  = static_cast<uint8_t>((rand >> 56) & 0xFF);
    out[9]  = static_cast<uint8_t>((rand >> 48) & 0xFF);
    out[10] = static_cast<uint8_t>((rand >> 40) & 0xFF);
    out[11] = static_cast<uint8_t>((rand >> 32) & 0xFF);
    out[12] = static_cast<uint8_t>((rand >> 24) & 0xFF);
    out[13] = static_cast<uint8_t>((rand >> 16) & 0xFF);
    out[14] = static_cast<uint8_t>((rand >> 8) & 0xFF);
    out[15] = static_cast<uint8_t>(rand & 0xFF);

    out[8] = static_cast<uint8_t>((out[8] & 0x3F) | 0x80);
}

uint64_t EventPromoter::monotonicNowUs() const
{
    const auto elapsed = std::chrono::steady_clock::now() - start_time_;
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
}

} // namespace vigia

#pragma once

#include "hazard_event.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>

namespace vigia {

class EventPromoter {
public:
    struct Config {
        char device_id[32];
        float rri_threshold{0.75f};
        float dedup_radius_m{5.0f};
        float dedup_window_s{30.0f};
        bool require_gps{true};
        float max_hdop{2.5f};
        std::size_t ring_capacity{512};

        Config() {
            std::strncpy(device_id, "vigia-dev-001", sizeof(device_id) - 1);
            device_id[sizeof(device_id) - 1] = '\0';
        }
    };

    explicit EventPromoter(Config cfg = Config{});

    EventPromoter(const EventPromoter&) = delete;
    EventPromoter& operator=(const EventPromoter&) = delete;

    /* Hot path — stack copy into ring; no heap allocation. */
    bool submit(const HazardObservation& candidate);

    /* Consumer API — mutex on consumer side only. */
    bool tryDequeue(HazardObservation& out);
    bool waitDequeue(HazardObservation& out, std::chrono::milliseconds timeout);
    std::size_t pendingCount() const;

    const Config& config() const { return cfg_; }
    uint64_t deviceSeq() const { return device_seq_; }

private:
    struct DedupEntry {
        double lat{0.0};
        double lon{0.0};
        uint64_t timestamp_us{0};
    };

    static constexpr std::size_t kDedupHistory = 128;

    bool passesGates(const HazardObservation& obs) const;
    bool isDuplicate(const HazardObservation& obs) const;
    void recordDedup(const HazardObservation& obs);
    void pruneDedup(uint64_t now_us) const;
    bool enqueue(HazardObservation obs);
    void generateEventId(uint8_t out[16]) const;
    uint64_t monotonicNowUs() const;

    Config cfg_;
    std::array<HazardObservation, 512> ring_{};
    std::size_t ring_capacity_{512};

    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};

    mutable std::array<DedupEntry, kDedupHistory> dedup_{};
    mutable std::size_t dedup_count_{0};  // pruned from const gate checks

    uint64_t device_seq_{0};
    std::chrono::steady_clock::time_point start_time_{};

    mutable std::mutex consumer_mutex_;
};

} // namespace vigia

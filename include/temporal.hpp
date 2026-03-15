#pragma once

#include <array>
#include <cstddef>

namespace vigia {

struct TemporalMetrics {
    float persistence{0.0f};
    float stability{0.0f};
};

class TemporalAnalyzer {
public:
    static constexpr std::size_t kHistorySize = 10;

    explicit TemporalAnalyzer(std::size_t /*historySize*/ = kHistorySize);
    virtual ~TemporalAnalyzer() = default;

    virtual TemporalMetrics update(float depressionScore, float roughness);
    void reset();

private:
    float computePersistence() const;
    float computeStability() const;

    // Fixed-size circular buffers — contiguous memory, no heap allocation
    std::array<float, kHistorySize> depressionBuf_{};
    std::array<float, kHistorySize> roughnessBuf_{};
    std::size_t head_{0};
    std::size_t count_{0};
};

} // namespace vigia

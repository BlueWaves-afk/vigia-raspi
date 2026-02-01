#pragma once

#include <deque>
#include <cstddef>

namespace vigia {

/* ===================== Output ===================== */

struct TemporalMetrics {
    float persistence{0.0f};
    float stability{0.0f};
};

/* ===================== Analyzer ===================== */

class TemporalAnalyzer {
public:
    explicit TemporalAnalyzer(std::size_t historySize = 10);

    TemporalMetrics update(float depressionScore, float roughness);
    void reset();

private:
    float computePersistence() const;
    float computeStability() const;

private:
    std::size_t historySize_;
    std::deque<float> depressionHistory_;
    std::deque<float> roughnessHistory_;
};

} // namespace vigia

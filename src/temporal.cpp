#include "temporal.hpp"
#include <algorithm>
#include <cmath>

namespace vigia {

TemporalAnalyzer::TemporalAnalyzer(std::size_t /*historySize*/) {}

TemporalMetrics TemporalAnalyzer::update(float depressionScore, float roughness) {
    depressionBuf_[head_] = depressionScore;
    roughnessBuf_[head_]  = roughness;
    head_ = (head_ + 1) % kHistorySize;
    if (count_ < kHistorySize) ++count_;

    TemporalMetrics metrics{};
    metrics.persistence = computePersistence();
    metrics.stability   = computeStability();
    return metrics;
}

void TemporalAnalyzer::reset() {
    depressionBuf_.fill(0.0f);
    roughnessBuf_.fill(0.0f);
    head_ = 0;
    count_ = 0;
}

float TemporalAnalyzer::computePersistence() const {
    if (count_ < 2) return 0.0f;
    const float n = static_cast<float>(count_);
    float sum = 0.0f, sumSq = 0.0f;
    for (std::size_t i = 0; i < count_; ++i) {
        const float v = depressionBuf_[i];
        sum   += v;
        sumSq += v * v;
    }
    const float mean   = sum / n;
    const float var    = (sumSq / n) - (mean * mean);
    const float stddev = std::sqrt(std::max(0.0f, var));
    return mean / (stddev + 1e-4f);
}

float TemporalAnalyzer::computeStability() const {
    if (count_ < 2) return 0.0f;
    const float n = static_cast<float>(count_);
    float sum = 0.0f, sumSq = 0.0f;
    for (std::size_t i = 0; i < count_; ++i) {
        const float v = roughnessBuf_[i];
        sum   += v;
        sumSq += v * v;
    }
    const float mean   = sum / n;
    const float var    = (sumSq / n) - (mean * mean);
    const float stddev = std::sqrt(std::max(0.0f, var));
    return 1.0f / (stddev + 1e-4f);
}

} // namespace vigia

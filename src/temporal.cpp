#include "temporal.hpp"

#include <numeric>
#include <cmath>

namespace vigia {

/* ===================== Constructor ===================== */

TemporalAnalyzer::TemporalAnalyzer(std::size_t historySize)
    : historySize_(historySize) {}

/* ===================== Public Update ===================== */

TemporalMetrics TemporalAnalyzer::update(
    float depressionScore,
    float roughness
) {
    // Push new samples
    depressionHistory_.push_back(depressionScore);
    roughnessHistory_.push_back(roughness);

    // Maintain fixed history length
    if (depressionHistory_.size() > historySize_)
        depressionHistory_.pop_front();

    if (roughnessHistory_.size() > historySize_)
        roughnessHistory_.pop_front();

    TemporalMetrics metrics{};
    metrics.persistence = computePersistence();
    metrics.stability   = computeStability();

    return metrics;
}

/* ===================== Reset ===================== */

void TemporalAnalyzer::reset() {
    depressionHistory_.clear();
    roughnessHistory_.clear();
}

/* ===================== Persistence ===================== */
/*
Persistence answers:
"Is this depression consistently present over time?"

High when:
- Depression score is non-zero
- Variance over time is low
*/

float TemporalAnalyzer::computePersistence() const {
    if (depressionHistory_.size() < 2)
        return 0.0f;

    const float mean =
        std::accumulate(
            depressionHistory_.begin(),
            depressionHistory_.end(),
            0.0f
        ) / depressionHistory_.size();

    float var = 0.0f;
    for (float v : depressionHistory_)
        var += (v - mean) * (v - mean);

    var /= depressionHistory_.size();

    const float stddev = std::sqrt(var);

    // Mean dominates, but unstable signals get penalized
    return mean / (stddev + 1e-4f);
}

/* ===================== Stability ===================== */
/*
Stability answers:
"Is the surface texture temporally consistent?"

High when roughness does NOT fluctuate wildly.
*/

float TemporalAnalyzer::computeStability() const {
    if (roughnessHistory_.size() < 2)
        return 0.0f;

    const float mean =
        std::accumulate(
            roughnessHistory_.begin(),
            roughnessHistory_.end(),
            0.0f
        ) / roughnessHistory_.size();

    float var = 0.0f;
    for (float v : roughnessHistory_)
        var += (v - mean) * (v - mean);

    var /= roughnessHistory_.size();

    const float stddev = std::sqrt(var);

    // Inverse variance → higher means more stable
    return 1.0f / (stddev + 1e-4f);
}

} // namespace vigia

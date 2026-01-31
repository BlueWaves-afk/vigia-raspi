#pragma once

#include <atomic>
#include <cstdint>

#include "analytical.hpp"
#include "safe_queue.hpp"

namespace vigia {

struct FusionWeights {
    float wConfidence{0.5F};
    float wGeometry{0.35F};
    float wPersistence{0.15F};
};

struct FusionState {
    std::uint64_t frameId{0};
    float rri{0.0F};
    float persistence{0.0F};
    float geometricResidual{0.0F};
    PerceptionResult perception;
};

class FusionEngine {
public:
    explicit FusionEngine(const FusionWeights& weights);

    void run(SafeQueue<AnalyticalResult>& analyticsQueue,
             SafeQueue<FusionState>& fusionQueue,
             std::atomic<bool>& running);

    float computeRRI(float yoloConfidence,
                     float geometricMagnitude,
                     float persistence) const;

    const FusionWeights& weights() const { return weights_; }

private:
    FusionWeights weights_{};
};

} // namespace vigia

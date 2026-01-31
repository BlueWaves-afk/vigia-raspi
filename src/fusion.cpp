#include "fusion.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

namespace vigia {

FusionEngine::FusionEngine(const FusionWeights& weights)
    : weights_(weights)
{
}

void FusionEngine::run(SafeQueue<AnalyticalResult>& analyticsQueue,
                       SafeQueue<FusionState>& fusionQueue,
                       std::atomic<bool>& running)
{
    float persistenceState = 0.0F;

    while (running.load()) {
        auto result = analyticsQueue.try_pop();
        if (!result) {
            std::this_thread::sleep_for(std::chrono::milliseconds{2});
            continue;
        }

        float targetPersistence = result->perception.greatestConfidence;
        constexpr float smoothing = 0.8F;
        persistenceState = smoothing * persistenceState + (1.0F - smoothing) * targetPersistence;

        FusionState state;
        state.frameId = result->frameId;
        state.perception = result->perception;
        state.geometricResidual = result->geometricMagnitude;
        state.persistence = persistenceState;
        state.rri = computeRRI(result->perception.greatestConfidence,
                               result->geometricMagnitude,
                               state.persistence);

        std::cout << "[Fusion] frame=" << state.frameId
                  << " geometricResidual=" << state.geometricResidual << '\n';

        fusionQueue.push(std::move(state));
    }
}

float FusionEngine::computeRRI(float yoloConfidence,
                               float geometricMagnitude,
                               float persistence) const
{
    float score = weights_.wConfidence * yoloConfidence
        + weights_.wGeometry * geometricMagnitude
        + weights_.wPersistence * persistence;
    return std::clamp(score, 0.0F, 1.0F);
}

} // namespace vigia

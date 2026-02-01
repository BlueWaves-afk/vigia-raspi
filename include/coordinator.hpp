#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include "perception.hpp"
#include "analytical.hpp"
#include "temporal.hpp"
#include "fusion.hpp"

namespace vigia {

/* ===================== System Output ===================== */

struct CoordinatorOutput {
    float finalConfidence = 0.0f;
    bool isHazard = false;

    // Debug / telemetry
    float yoloConfidence = 0.0f;
    float geometryConfidence = 0.0f;
    float temporalConfidence = 0.0f;

    float depressionScore = 0.0f;
    float roughness = 0.0f;
    float persistence = 0.0f;
    float stability = 0.0f;
};

/* ===================== Coordinator Config ===================== */

struct CoordinatorConfig {
    // FPS control
    int targetFPS = 10;

    // Decision threshold
    float hazardThreshold = 0.65f;

    // Thermal throttling
    float maxCPUTempC = 80.0f;

    // Threading
    bool enableAsync = true;

    // Debug
    bool verbose = false;
};

/* ===================== Coordinator ===================== */

class Coordinator {
public:
    explicit Coordinator(const CoordinatorConfig& config);

    /* ----- Dependency Injection ----- */
    void setPerceptionAgent(std::shared_ptr<PerceptionAgent> agent);
    void setAnalyticalAgent(std::shared_ptr<AnalyticalAgent> agent);
    void setTemporalAnalyzer(std::shared_ptr<TemporalAnalyzer> analyzer);
    void setFusionEngine(std::shared_ptr<FusionEngine> fusion);

    /* ----- Lifecycle ----- */
    bool initialize();
    void shutdown();

    /* ----- Main Entry ----- */
    CoordinatorOutput processFrame(const cv::Mat& frame);

    /* ----- Telemetry ----- */
    float getCurrentFPS() const;
    float getLastCPUTemperature() const;

private:
    /* ===================== Internal Steps ===================== */

    void enforceFPSLimit();
    bool thermalSafe() const;

    /* ===================== Internal State ===================== */

    CoordinatorConfig config_;

    std::shared_ptr<PerceptionAgent> perception_;
    std::shared_ptr<AnalyticalAgent> analytical_;
    std::shared_ptr<TemporalAnalyzer> temporal_;
    std::shared_ptr<FusionEngine> fusion_;

    // Timing
    std::chrono::steady_clock::time_point lastFrameTime_;
    std::atomic<float> currentFPS_{0.0f};

    // Thermal
    mutable float lastCPUTempC_ = 0.0f;

    // Thread safety
    std::mutex mutex_;
    std::atomic<bool> initialized_{false};
};

} // namespace vigia

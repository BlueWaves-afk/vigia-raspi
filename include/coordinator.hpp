#pragma once

#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

#include <opencv2/core.hpp>

#include "perception.hpp"
#include "analytical.hpp"
#include "temporal.hpp"
#include "fusion.hpp"

namespace vigia {

class Coordinator {
public:
    /* ===================== Constructor ===================== */
    Coordinator(
        PerceptionAgent& perception,
        AnalyticalAgent& analytical,
        TemporalAnalyzer& temporal,
        FusionEngine& fusion,
        int targetFps
    );

    /* ===================== Lifecycle ===================== */
    void start();
    void stop();

private:
    /* ===================== Threads ===================== */
    void captureLoop();
    void processLoop();

    /* ===================== Core Logic ===================== */
    void processFrame();
    void adaptiveControl(long elapsedMs);
    void frameLimiter(long elapsedMs) const;

    /* ===================== Utilities ===================== */
    float readTemperature() const;
    void pinThread(std::thread& t, int coreId);

    /* ===================== Output ===================== */
    void publishResult(const Detection& det, const FusionOutput& out) const;

private:
    /* ===================== Dependencies ===================== */
    PerceptionAgent& perception_;
    AnalyticalAgent& analytical_;
    TemporalAnalyzer& temporal_;
    FusionEngine& fusion_;

    /* ===================== Timing ===================== */
    long targetFrameTimeMs_;

    /* ===================== Threading ===================== */
    std::atomic<bool> running_;
    std::thread captureThread_;
    std::thread mainThread_;

    /* ===================== Frame Buffer ===================== */
    std::vector<cv::Mat> frameBuffer_;
    std::mutex bufferMutex_;
    std::size_t frameIndex_;

    /* ===================== Adaptive Control ===================== */
    int midasStride_;
};

} // namespace vigia

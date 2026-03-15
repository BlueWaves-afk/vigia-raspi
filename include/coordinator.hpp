#pragma once

#include <chrono>
#include <cstdint>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

#include <opencv2/core.hpp>

#include "perception.hpp"
#include "analytical.hpp"
#include "temporal.hpp"
#include "fusion.hpp"
#include "safe_queue.hpp"

namespace vigia {

class Coordinator {
public:
    Coordinator(
        PerceptionAgent& perception,
        AnalyticalAgent& analytical,
        TemporalAnalyzer& temporal,
        FusionEngine& fusion,
        int targetFps
    );

    ~Coordinator();
    void start();
    void stop();

private:
    void captureLoop();
    void processLoop();
    void midasLoop();          // dedicated MiDaS thread on Core 2

    void processFrame();
    void adaptiveControl(long elapsedMs);
    void frameLimiter(long elapsedMs) const;

    float readTemperature() const;
    void pinThread(std::thread& t, int coreId);

    void publishResult(const Detection& det, const FusionOutput& out) const;

private:
    PerceptionAgent& perception_;
    AnalyticalAgent& analytical_;
    TemporalAnalyzer& temporal_;
    FusionEngine& fusion_;

    long targetFrameTimeMs_;

    std::atomic<bool> running_;
    std::thread captureThread_;
    std::thread mainThread_;
    std::thread midasThread_;  // Core 2 — async MiDaS

    // Frame buffer — size 8 to cover MiDaS's ~525ms window at 15 FPS
    static constexpr std::size_t FRAME_BUFFER_SIZE = 8;
    std::vector<cv::Mat> frameBuffer_;
    std::mutex bufferMutex_;
    std::uint64_t frameIndex_;

    // MiDaS work queue: carries frame + YOLO detections for fusion on Core 2
    struct MidasWork {
        std::uint64_t frameIdx;
        cv::Mat frame;
        std::vector<Detection> detections;
    };
    SafeQueue<MidasWork> midasQueue_;

    int midasStride_;
    float cachedTemp_{0.0f};
    std::chrono::steady_clock::time_point lastTempReadTs_{};
};

} // namespace vigia

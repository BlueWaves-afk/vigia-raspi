#include "coordinator.hpp"

#include <chrono>
#include <thread>
#include <atomic>
#include <iostream>
#include <fstream>
#include <mutex>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

namespace vigia {

/* ===================== Constants ===================== */

static constexpr std::size_t FRAME_BUFFER_SIZE = 4;
static constexpr float TEMP_WARN_C = 75.0f;
static constexpr float TEMP_CRITICAL_C = 85.0f;

/* ===================== Constructor ===================== */

Coordinator::Coordinator(
    PerceptionAgent& perception,
    AnalyticalAgent& analytical,
    TemporalAnalyzer& temporal,
    FusionEngine& fusion,
    int targetFps
)
    : perception_(perception),
      analytical_(analytical),
      temporal_(temporal),
      fusion_(fusion),
      targetFrameTimeMs_(1000 / targetFps),
      running_(false),
      frameIndex_(0),
      midasStride_(1)
{
    frameBuffer_.resize(FRAME_BUFFER_SIZE);
}

/* ===================== Lifecycle ===================== */

void Coordinator::start() {
    running_ = true;

    captureThread_ = std::thread(&Coordinator::captureLoop, this);
    mainThread_    = std::thread(&Coordinator::processLoop, this);

    pinThread(captureThread_, 0); // LITTLE core
    pinThread(mainThread_,  1);   // BIG core
}

void Coordinator::stop() {
    running_ = false;

    if (captureThread_.joinable())
        captureThread_.join();

    if (mainThread_.joinable())
        mainThread_.join();
}

/* ===================== Capture Loop ===================== */

void Coordinator::captureLoop() {
    while (running_) {
        cv::Mat frame;
        if (!perception_.captureFrame(frame))
            continue;

        {
            std::lock_guard<std::mutex> lock(bufferMutex_);
            frameBuffer_[frameIndex_ % FRAME_BUFFER_SIZE] = frame.clone();
            frameIndex_++;
        }
    }
}

/* ===================== Processing Loop ===================== */

void Coordinator::processLoop() {
    using clock = std::chrono::steady_clock;

    while (running_) {
        const auto start = clock::now();

        processFrame();

        const auto elapsedMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                clock::now() - start
            ).count();

        adaptiveControl(elapsedMs);
        frameLimiter(elapsedMs);
    }
}

/* ===================== Frame Processing ===================== */

void Coordinator::processFrame() {
    cv::Mat frame;

    {
        std::lock_guard<std::mutex> lock(bufferMutex_);
        if (frameIndex_ == 0)
            return;

        frame = frameBuffer_[(frameIndex_ - 1) % FRAME_BUFFER_SIZE];
    }

    /* ---------- YOLO ---------- */
    auto detections = perception_.runInference(frame);

    const bool runMidas = (frameIndex_ % midasStride_ == 0);
    cv::Mat depthMap;

    if (runMidas)
        depthMap = analytical_.runInference(frame);

    for (const auto& det : detections) {
        if (!det.isPothole())
            continue;

        if (!runMidas || depthMap.empty())
            continue;

        cv::Rect depthROI =
            analytical_.scaleROIToDepth(
                det.bbox,
                frame.size(),
                depthMap.size()
            );

        cv::Mat roiDepth =
            analytical_.extractDepthROI(depthMap, depthROI);

        if (roiDepth.empty())
            continue;

        auto residuals =
            analytical_.computeDepthResiduals(roiDepth);

        auto geom =
            analytical_.computeGeometryMetrics(roiDepth, residuals);

        auto temporalMetrics =
            temporal_.update(geom.depressionScore, geom.roughness);

        FusionInput fin{};
        fin.yoloConfidence   = det.confidence;
        fin.depressionScore  = geom.depressionScore;
        fin.roughness        = geom.roughness;
        fin.persistence      = temporalMetrics.persistence;
        fin.stability        = temporalMetrics.stability;

        FusionOutput fout = fusion_.fuse(fin);

        publishResult(det, fout);
    }
}

/* ===================== Adaptive Control ===================== */

void Coordinator::adaptiveControl(long elapsedMs) {
    const float temp = readTemperature();

    if (temp > TEMP_CRITICAL_C) {
        midasStride_ = 5;
    }
    else if (temp > TEMP_WARN_C) {
        midasStride_ = 3;
    }
    else if (elapsedMs > targetFrameTimeMs_) {
        midasStride_ = std::min(midasStride_ + 1, 5);
    }
    else {
        midasStride_ = std::max(1, midasStride_ - 1);
    }
}

/* ===================== Temperature ===================== */

float Coordinator::readTemperature() const {
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    if (!file.is_open())
        return 0.0f;

    float tempMilli = 0.0f;
    file >> tempMilli;
    return tempMilli / 1000.0f;
}

/* ===================== FPS Limiter ===================== */

void Coordinator::frameLimiter(long elapsedMs) const {
    if (elapsedMs < targetFrameTimeMs_) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                targetFrameTimeMs_ - elapsedMs
            )
        );
    }
}

/* ===================== Thread Pinning ===================== */

void Coordinator::pinThread(std::thread& t, int coreId) {
#ifdef __linux__
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(coreId, &cpuset);

    pthread_setaffinity_np(
        t.native_handle(),
        sizeof(cpu_set_t),
        &cpuset
    );
#endif
}

/* ===================== Output ===================== */

void Coordinator::publishResult(
    const Detection& det,
    const FusionOutput& out
) const {
    std::cout
        << "[POTHOLE]"
        << " conf=" << out.finalConfidence
        << " geo="  << out.geometryConfidence
        << " tmp="  << out.temporalConfidence
        << " stride=" << midasStride_
        << "\n";
}

} // namespace vigia

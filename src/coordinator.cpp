#include "coordinator.hpp"
#include "sensor_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <iostream>
#include <fstream>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

namespace vigia {

/* ===================== Constants ===================== */

static constexpr float TEMP_WARN_C = 75.0f;
static constexpr float TEMP_CRITICAL_C = 85.0f;
static constexpr std::int32_t POTHOLE_CLASS_ID = 0;

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

Coordinator::~Coordinator() {
    running_ = false;
    midasQueue_.shutdown();

    if (captureThread_.joinable()) captureThread_.join();
    if (mainThread_.joinable())    mainThread_.join();
    if (midasThread_.joinable())   midasThread_.join();
}

void Coordinator::start() {
    running_ = true;

    captureThread_ = std::thread(&Coordinator::captureLoop, this);
    mainThread_    = std::thread(&Coordinator::processLoop, this);
    midasThread_   = std::thread(&Coordinator::midasLoop,   this);

    pinThread(captureThread_, 0);
    pinThread(mainThread_,    1);
    pinThread(midasThread_,   2);
}

void Coordinator::stop() {
    running_ = false;
    midasQueue_.shutdown();

    if (captureThread_.joinable()) captureThread_.join();
    if (mainThread_.joinable())    mainThread_.join();
    if (midasThread_.joinable())   midasThread_.join();
}

/* ===================== Sensor Bridge ===================== */

void Coordinator::setSensorBridge(SensorBridge& bridge)
{
    sensorBridge_ = &bridge;
}

Coordinator::SensorSnapshot Coordinator::querySensors() const
{
    SensorSnapshot snap{};
    if (!sensorBridge_)
        return snap;

    const auto proc = sensorProcessor_.process(sensorBridge_->state());

    snap.imuIss   = proc.imuIss;
    snap.speedMs  = proc.speedMs;
    snap.gpsLat   = proc.gpsLat;
    snap.gpsLon   = proc.gpsLon;
    snap.gpsValid = proc.gpsValid;
    return snap;
}

/* ===================== Capture Loop ===================== */

void Coordinator::captureLoop() {
#if defined(__linux__) && defined(__aarch64__)
    // Pin capture thread to Core 0 — dedicated to frame acquisition.
    // Avoids cross-core migration and keeps L1/L2 cache warm for
    // VideoCapture decode operations.
    {
        cpu_set_t cpuSet;
        CPU_ZERO(&cpuSet);
        CPU_SET(0, &cpuSet);
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuSet);
    }
#endif

    try {
        while (running_) {
            cv::Mat frame;
            if (!perception_.captureFrame(frame)) {
                // Video ended or camera disconnected — signal shutdown
                running_ = false;
                break;
            }

            // Clone outside the lock — ~0.9ms clone must not block processLoop.
            cv::Mat cloned = frame.clone();
            {
                std::lock_guard<std::mutex> lock(bufferMutex_);
                frameBuffer_[frameIndex_ % FRAME_BUFFER_SIZE] = std::move(cloned);
                frameIndex_++;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[Coordinator] Capture thread exception: " << e.what() << std::endl;
        running_ = false;
    } catch (...) {
        std::cerr << "[Coordinator] Capture thread unknown exception" << std::endl;
        running_ = false;
    }
}

/* ===================== MiDaS Loop (Core 2) ===================== */

void Coordinator::midasLoop() {
#if defined(__linux__) && defined(__aarch64__)
    {
        cpu_set_t cpuSet;
        CPU_ZERO(&cpuSet);
        CPU_SET(2, &cpuSet);
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuSet);
    }
#endif
    try {
        while (running_) {
            MidasWork work;
            if (!midasQueue_.wait_and_pop(work))
                continue;
            if (!running_) break;

            // InstrumentedAnalyticalAgent::runInference records depth to bus
            cv::Mat depthMap = analytical_.runInference(work.frame);
            if (depthMap.empty()) continue;

            // Re-run fusion for any potholes detected on this frame.
            // We don't have the detection list here, so we rely on the
            // InstrumentationBus slot's filteredDetections via the coordinator's
            // stored detections. Instead, we store detections per frame in midasWork.
            for (const auto& det : work.detections) {
                if (det.classId != POTHOLE_CLASS_ID) continue;

                cv::Rect depthROI = analytical_.scaleROIToDepth(
                    det.boundingBox, work.frame.size(), depthMap.size());
                cv::Mat roiDepth = analytical_.extractDepthROI(depthMap, depthROI);
                if (roiDepth.empty()) continue;

                auto residuals = analytical_.computeDepthResiduals(roiDepth);
                auto geom      = analytical_.computeGeometryMetrics(roiDepth, residuals);
                auto temporal  = temporal_.update(geom.depressionScore, geom.roughness);

                FusionInput fin{};
                fin.yoloConfidence  = det.confidence;
                fin.depressionScore = geom.depressionScore;
                fin.roughness       = geom.roughness;
                fin.persistence     = temporal.persistence;
                fin.stability       = temporal.stability;
                // M6: inject sensor snapshot captured at enqueue time
                fin.imuIss   = work.sensors.imuIss;
                fin.speedMs  = work.sensors.speedMs;
                fin.gpsLat   = work.sensors.gpsLat;
                fin.gpsLon   = work.sensors.gpsLon;
                fin.gpsValid = work.sensors.gpsValid;

                auto fout = fusion_.fuse(fin);
                publishResult(det, fout);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[Coordinator] MiDaS thread exception: " << e.what() << std::endl;
        running_ = false;
    }
}

/* ===================== Processing Loop ===================== */

void Coordinator::processLoop() {
#if defined(__linux__) && defined(__aarch64__)
    // Do NOT pin the process thread when using ACL — ACL's CPPScheduler
    // spawns worker threads that need access to all cores. Pinning to a
    // single core prevents ACL from utilizing the other 3 cores.
    // Core affinity is handled at the coordinator level via pinThread().
#endif

    using clock = std::chrono::steady_clock;

    try {
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
    } catch (const std::exception& e) {
        std::cerr << "[Coordinator] Process thread exception: " << e.what() << std::endl;
        running_ = false;
    } catch (...) {
        std::cerr << "[Coordinator] Process thread unknown exception" << std::endl;
        running_ = false;
    }
}

/* ===================== Frame Processing ===================== */

void Coordinator::processFrame() {
    cv::Mat frame;
    std::uint64_t currentIdx;

    {
        std::lock_guard<std::mutex> lock(bufferMutex_);
        if (frameIndex_ == 0)
            return;
        frame = frameBuffer_[(frameIndex_ - 1) % FRAME_BUFFER_SIZE];
        currentIdx = frameIndex_;
    }

    /* ---------- Sensor snapshot (once per frame) ---------- */
    const SensorSnapshot sensors = querySensors();

    /* ---------- YOLO (Core 1 — every frame) ---------- */
    auto detections = perception_.runInference(frame);

    /* ---------- MiDaS (Core 2 — async, stride-gated) ---------- */
    if (currentIdx % midasStride_ == 0) {
        midasQueue_.push({currentIdx, frame.clone(), detections, sensors});
    }

    // YOLO-only baseline fusion — MiDaS geometry arrives asynchronously.
    for (const auto& det : detections) {
        if (det.classId != POTHOLE_CLASS_ID)
            continue;

        FusionInput fin{};
        fin.yoloConfidence = det.confidence;
        fin.imuIss         = sensors.imuIss;
        fin.speedMs        = sensors.speedMs;
        fin.gpsLat         = sensors.gpsLat;
        fin.gpsLon         = sensors.gpsLon;
        fin.gpsValid       = sensors.gpsValid;

        FusionOutput fout  = fusion_.fuse(fin);
        publishResult(det, fout);
    }

    perception_.notifyProcessingComplete(currentIdx);
}

/* ===================== Adaptive Control ===================== */

void Coordinator::adaptiveControl(long elapsedMs) {
    // Throttle sysfs temperature reads to once per second
    const auto now = std::chrono::steady_clock::now();
    if (now - lastTempReadTs_ >= std::chrono::seconds(1)) {
        cachedTemp_ = readTemperature();
        lastTempReadTs_ = now;
    }
    const float temp = cachedTemp_;

    if (temp > TEMP_CRITICAL_C) {
        midasStride_ = 5;
    }
    else if (temp > TEMP_WARN_C) {
        midasStride_ = 3;
    }
    else if (elapsedMs > 150) {
        // Only throttle MiDaS if genuinely below ~7 FPS — not just because
        // targetFrameTimeMs_ is set to an unreachable 30 FPS target.
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
#ifndef NDEBUG
    std::cout
        << "[POTHOLE]"
        << " conf="   << out.finalConfidence
        << " geo="    << out.geometryConfidence
        << " tmp="    << out.temporalConfidence
        << " stride=" << midasStride_;

    if (out.gpsValid) {
        std::cout
            << std::fixed
            << " lat="  << out.latitude
            << " lon="  << out.longitude
            << " spd="  << out.speedMs << "m/s";
    }
    std::cout << "\n";
#else
    (void)det; (void)out;
#endif
}

} // namespace vigia
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
#include "sensor_processor.hpp"
#include "safe_queue.hpp"

namespace vigia {

class SensorBridge;  // forward-declared; include sensor_bridge.hpp only in .cpp
class EventStore;    // forward-declared; include event_store.hpp only in .cpp

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

    // M6: wire in a live SensorBridge — call before start().
    // The bridge must outlive the Coordinator.
    // If never called the pipeline runs vision-only (ISS = 0, no geo-tag).
    void setSensorBridge(SensorBridge& bridge);

    // M7: wire event logging — call before start(). Must outlive Coordinator.
    void setEventStore(EventStore& store);

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

    // M6: build a sensor-aware FusionInput snapshot for the current frame.
    // Returns cached zero-sensor input when no bridge is wired.
    struct SensorSnapshot {
        float imuIss{0.0f};
        float speedMs{0.0f};
        double gpsLat{0.0};
        double gpsLon{0.0};
        float gpsHdop{99.0f};
        uint8_t gpsFixType{0};
        bool gpsValid{false};
    };
    SensorSnapshot querySensors() const;

private:
    PerceptionAgent& perception_;
    AnalyticalAgent& analytical_;
    TemporalAnalyzer& temporal_;
    FusionEngine& fusion_;
    SensorBridge* sensorBridge_{nullptr};
    EventStore* eventStore_{nullptr};
    mutable SensorProcessor sensorProcessor_;

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

    // MiDaS work queue: carries frame + YOLO detections + sensor snapshot for Core 2
    struct MidasWork {
        std::uint64_t frameIdx;
        cv::Mat frame;
        std::vector<Detection> detections;
        SensorSnapshot sensors;  // M6: captured at enqueue time — consistent with frame
    };
    SafeQueue<MidasWork> midasQueue_;

    int midasStride_;
    float cachedTemp_{0.0f};
    std::chrono::steady_clock::time_point lastTempReadTs_{};
};

} // namespace vigia

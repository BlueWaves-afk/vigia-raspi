#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include "analytical.hpp"
#include "fusion.hpp"
#include "perception.hpp"
#include "safe_queue.hpp"

namespace vigia {

struct CoordinatorConfig {
    std::size_t frameBufferSize{8};
    std::size_t initialFrameSkip{1};
    std::chrono::milliseconds thermalCheckInterval{std::chrono::milliseconds{1000}};
    float thermalThresholdC{75.0F};
};

class CircularFrameBuffer {
public:
    explicit CircularFrameBuffer(std::size_t capacity);

    void insert(const FramePacket& packet);
    std::optional<FramePacket> fetch(std::uint64_t frameId) const;

private:
    std::size_t capacity_{0};
    mutable std::mutex mutex_{};
    std::vector<FramePacket> frames_;
};

class Coordinator {
public:
    Coordinator(const CoordinatorConfig& config,
                SafeQueue<FramePacket>& perceptionQueue,
                SafeQueue<AnalyticalRequest>& analyticalQueue,
                SafeQueue<PerceptionResult>& perceptionResults,
                SafeQueue<AnalyticalResult>& analyticalResults,
                SafeQueue<FusionState>& fusionQueue);

    void run(std::atomic<bool>& running);

    void setFrameSkip(std::size_t skip);
    std::size_t frameSkip() const;

private:
    void adjustThermals(float temperatureC);
    std::optional<float> readCpuTemperatureCelsius() const;
    void dispatchAnalysis(const PerceptionResult& perception);

    CoordinatorConfig config_{};
    SafeQueue<FramePacket>& perceptionQueue_;
    SafeQueue<AnalyticalRequest>& analyticalQueue_;
    SafeQueue<PerceptionResult>& perceptionResults_;
    SafeQueue<AnalyticalResult>& analyticalResults_;
    SafeQueue<FusionState>& fusionQueue_;

    CircularFrameBuffer frameBuffer_;
    std::atomic<std::size_t> frameSkip_{1};
    std::uint64_t frameCounter_{0};
    std::chrono::steady_clock::time_point lastThermalSample_{};
};

} // namespace vigia

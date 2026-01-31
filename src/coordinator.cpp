#include "coordinator.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <thread>

namespace vigia {

CircularFrameBuffer::CircularFrameBuffer(std::size_t capacity)
    : capacity_(capacity)
    , frames_(capacity)
{
}

void CircularFrameBuffer::insert(const FramePacket& packet)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (frames_.empty()) {
        return;
    }
    std::size_t index = packet.frameId % capacity_;
    frames_[index] = packet;
}

std::optional<FramePacket> CircularFrameBuffer::fetch(std::uint64_t frameId) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (frames_.empty()) {
        return std::nullopt;
    }
    std::size_t index = frameId % capacity_;
    const FramePacket& candidate = frames_[index];
    if (candidate.frameId != frameId) {
        return std::nullopt;
    }
    return candidate;
}

Coordinator::Coordinator(const CoordinatorConfig& config,
                         SafeQueue<FramePacket>& perceptionQueue,
                         SafeQueue<AnalyticalRequest>& analyticalQueue,
                         SafeQueue<PerceptionResult>& perceptionResults,
                         SafeQueue<AnalyticalResult>& analyticalResults,
                         SafeQueue<FusionState>& fusionQueue)
    : config_(config)
    , perceptionQueue_(perceptionQueue)
    , analyticalQueue_(analyticalQueue)
    , perceptionResults_(perceptionResults)
    , analyticalResults_(analyticalResults)
    , fusionQueue_(fusionQueue)
    , frameBuffer_(config.frameBufferSize)
    , frameSkip_(config.initialFrameSkip)
{
}

void Coordinator::run(std::atomic<bool>& running)
{
    lastThermalSample_ = std::chrono::steady_clock::now();

    while (running.load()) {
        FramePacket packet;
        packet.frameId = ++frameCounter_;
        packet.timestamp = std::chrono::steady_clock::now();
        packet.frame = cv::Mat::zeros(cv::Size(256, 256), CV_8UC3); // Placeholder frame acquisition.

        frameBuffer_.insert(packet);

        if (frameSkip_.load() <= 1 || (packet.frameId % frameSkip_.load()) == 0) {
            perceptionQueue_.push(packet);
        }

        while (auto perception = perceptionResults_.try_pop()) {
            dispatchAnalysis(*perception);
        }

        auto now = std::chrono::steady_clock::now();
        if (now - lastThermalSample_ >= config_.thermalCheckInterval) {
            if (auto temperature = readCpuTemperatureCelsius()) {
                adjustThermals(*temperature);
            }
            lastThermalSample_ = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
}

void Coordinator::setFrameSkip(std::size_t skip)
{
    frameSkip_.store(std::max<std::size_t>(1, skip));
}

std::size_t Coordinator::frameSkip() const
{
    return frameSkip_.load();
}

void Coordinator::adjustThermals(float temperatureC)
{
    constexpr std::size_t maxSkip = 6;
    auto currentSkip = frameSkip_.load();
    if (temperatureC >= config_.thermalThresholdC) {
        if (currentSkip < maxSkip) {
            frameSkip_.store(currentSkip + 1);
            std::cout << "[Coordinator] Thermal limit breached (" << temperatureC
                      << "C). Increasing frame skip to " << frameSkip_.load() << '\n';
        }
    } else if (temperatureC < config_.thermalThresholdC - 3.0F) {
        if (currentSkip > config_.initialFrameSkip) {
            frameSkip_.store(currentSkip - 1);
            std::cout << "[Coordinator] Temperature stable (" << temperatureC
                      << "C). Decreasing frame skip to " << frameSkip_.load() << '\n';
        }
    }
}

std::optional<float> Coordinator::readCpuTemperatureCelsius() const
{
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    if (!file.is_open()) {
        return std::nullopt;
    }

    long milliDegrees = 0;
    file >> milliDegrees;
    if (file.fail()) {
        return std::nullopt;
    }

    return static_cast<float>(milliDegrees) / 1000.0F;
}

void Coordinator::dispatchAnalysis(const PerceptionResult& perception)
{
    auto frame = frameBuffer_.fetch(perception.frameId);
    if (!frame) {
        return;
    }

    AnalyticalRequest request;
    request.frameId = perception.frameId;
    request.framePacket = *frame;
    request.perception = perception;
    analyticalQueue_.push(std::move(request));
}

} // namespace vigia

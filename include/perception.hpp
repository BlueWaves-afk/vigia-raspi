#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <openvino/openvino.hpp>

#include "safe_queue.hpp"

namespace vigia {

/* ===================== Data Structures ===================== */

struct FramePacket {
    std::uint64_t frameId{0};
    cv::Mat frame;
    std::chrono::steady_clock::time_point timestamp{};
};

struct Detection {
    cv::Rect boundingBox;
    float confidence{0.0F};
    std::int32_t classId{-1};
};

struct PerceptionResult {
    std::uint64_t frameId{0};
    std::vector<Detection> detections;
    float greatestConfidence{0.0F};
    std::chrono::steady_clock::time_point timestamp{};
};

/* ===================== Perception Agent ===================== */

class PerceptionAgent {
public:
    PerceptionAgent(const std::string& modelXmlPath,
                    const std::string& device = "CPU");

    void run(SafeQueue<FramePacket>& inputQueue,
             SafeQueue<PerceptionResult>& outputQueue,
             std::atomic<bool>& running);

private:
    /* ---------- OpenVINO ---------- */
    void loadNetwork(const std::string& modelXmlPath,
                     const std::string& device);

    std::vector<Detection> runInference(const cv::Mat& frame);
    float aggregateConfidence(const std::vector<Detection>& detections) const;

    /* ---------- Runtime ---------- */
    ov::Core core_;
    ov::CompiledModel compiledModel_;
    ov::InferRequest inferRequest_;
    ov::Output<const ov::Node> outputTensor_;

    /* ---------- Model Metadata ---------- */
    std::size_t inputWidth_{320};
    std::size_t inputHeight_{320};
    float confThreshold_{0.4f};
};

} // namespace vigia

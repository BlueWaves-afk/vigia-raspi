#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <openvino/openvino.hpp>

#include "safe_queue.hpp"

namespace vigia {

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

/**
 * @brief Helper for Ultralytics-style Letterboxing to maintain aspect ratio
 */
struct Letterbox {
    float scale;
    int pad_w;
    int pad_h;
};

class PerceptionAgent {
public:
    PerceptionAgent(const std::string& modelXmlPath,
                    const std::string& device = "CPU",
                    int cameraIndex = 0);
    virtual ~PerceptionAgent();

    virtual bool captureFrame(cv::Mat& frame);
    virtual std::vector<Detection> runInference(const cv::Mat& frame);

    void run(SafeQueue<FramePacket>& inputQueue,
             SafeQueue<PerceptionResult>& outputQueue,
             std::atomic<bool>& running);

protected:
    PerceptionAgent() = default;

private:
    void loadNetwork(const std::string& modelXmlPath, const std::string& device);
    float aggregateConfidence(const std::vector<Detection>& detections) const;
    
    // Core Logic
    cv::Mat preprocess(const cv::Mat& frame, Letterbox& lb);
    std::vector<Detection> postprocess(const ov::Tensor& output, const Letterbox& lb, const cv::Size& origSize);

    ov::Core core_;
    ov::CompiledModel compiledModel_;
    ov::InferRequest inferRequest_;
    ov::Output<const ov::Node> outputTensor_;

    int cameraIndex_{0};
    bool cameraInitialized_{false};
    cv::VideoCapture camera_;

    int inputWidth_{320};
    int inputHeight_{320};
    float confThreshold_{0.4f};
};

} // namespace vigia
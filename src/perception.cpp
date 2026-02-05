#include "perception.hpp"
#include <algorithm>
#include <thread>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace vigia {

PerceptionAgent::PerceptionAgent(const std::string& modelXmlPath,
                                 const std::string& device,
                                 int cameraIndex)
    : cameraIndex_(cameraIndex) {
    loadNetwork(modelXmlPath, device);
}

PerceptionAgent::~PerceptionAgent() {
    if (camera_.isOpened())
        camera_.release();
}

/* ===================== H-HMAS Agent Loop ===================== */

void PerceptionAgent::run(SafeQueue<FramePacket>& inputQueue,
                         SafeQueue<PerceptionResult>& outputQueue,
                         std::atomic<bool>& running) {
    while (running.load(std::memory_order_relaxed)) {
        auto packetOpt = inputQueue.try_pop();
        if (!packetOpt) {
            continue;
        }
        FramePacket packet = std::move(*packetOpt);

        auto detections = runInference(packet.frame);

        PerceptionResult result;
        result.frameId = packet.frameId;
        result.detections = detections;
        result.greatestConfidence = aggregateConfidence(detections);
        result.timestamp = packet.timestamp;

        outputQueue.push(std::move(result));
    }
}

bool PerceptionAgent::captureFrame(cv::Mat& frame) {
    if (!cameraInitialized_) {
        cameraInitialized_ = camera_.open(cameraIndex_);
        if (!cameraInitialized_)
            return false;

        camera_.set(cv::CAP_PROP_FRAME_WIDTH, 640.0);
        camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 480.0);
    }

    if (!camera_.isOpened())
        return false;

    if (!camera_.read(frame))
        return false;

    return true;
}

/* ===================== Network Loading ===================== */

void PerceptionAgent::loadNetwork(const std::string& modelXmlPath,
                                  const std::string& device) {
    auto model = core_.read_model(modelXmlPath);

    // Set layout to NCHW as required by OpenVINO for this model type
    model->get_parameters()[0]->set_layout("NCHW");
    ov::set_batch(model, 1);

    compiledModel_ = core_.compile_model(
        model,
        device,
        {
            ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
            ov::hint::num_requests(1)
        }
    );

    inferRequest_ = compiledModel_.create_infer_request();
    outputTensor_ = compiledModel_.output(0);

    const auto& inputShape = compiledModel_.input().get_shape();
    inputHeight_ = static_cast<int>(inputShape[2]);
    inputWidth_  = static_cast<int>(inputShape[3]);

    std::cout << "[YOLO26] Model Loaded. Input: " << inputWidth_ << "x" << inputHeight_ << "\n";
}

/* ===================== Core Logic ===================== */

cv::Mat PerceptionAgent::preprocess(const cv::Mat& frame, Letterbox& lb) {
    // 1. Convert BGR to RGB
    cv::Mat rgb;
    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);

    // 2. Calculate Letterbox (preserves aspect ratio)
    float r = std::min((float)inputWidth_ / frame.cols, (float)inputHeight_ / frame.rows);
    int new_unpad_w = std::round(frame.cols * r);
    int new_unpad_h = std::round(frame.rows * r);

    lb.scale = r;
    lb.pad_w = (inputWidth_ - new_unpad_w) / 2;
    lb.pad_h = (inputHeight_ - new_unpad_h) / 2;

    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(new_unpad_w, new_unpad_h));

    cv::Mat padded;
    cv::copyMakeBorder(resized, padded, lb.pad_h, inputHeight_ - new_unpad_h - lb.pad_h,
                       lb.pad_w, inputWidth_ - new_unpad_w - lb.pad_w,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    // 3. Normalize to [0.0, 1.0]
    cv::Mat blob;
    padded.convertTo(blob, CV_32F, 1.0 / 255.0);
    return blob;
}

std::vector<Detection> PerceptionAgent::runInference(const cv::Mat& frame) {
    Letterbox lb;
    cv::Mat blob = preprocess(frame, lb);

    // HWC to CHW manual transposition
    ov::Tensor inputTensor(ov::element::f32, {1, 3, (size_t)inputHeight_, (size_t)inputWidth_});
    float* tensorData = inputTensor.data<float>();
    float* blobData = blob.ptr<float>();

    for (int c = 0; c < 3; ++c) {
        for (int i = 0; i < inputHeight_ * inputWidth_; ++i) {
            tensorData[c * inputHeight_ * inputWidth_ + i] = blobData[i * 3 + c];
        }
    }

    inferRequest_.set_input_tensor(inputTensor);
    inferRequest_.infer();

    return postprocess(inferRequest_.get_tensor(outputTensor_), lb, frame.size());
}

std::vector<Detection> PerceptionAgent::postprocess(const ov::Tensor& output, const Letterbox& lb, const cv::Size& origSize) {
    std::vector<Detection> detections;
    const float* data = output.data<const float>();
    const auto& shape = output.get_shape(); 

    // YOLO26 NMS-Free output is typically [1, 300, 6]
    // Indices: 0:x1, 1:y1, 2:x2, 3:y2, 4:score, 5:class_id
    int num_detections = static_cast<int>(shape[1]);

    for (int i = 0; i < num_detections; ++i) {
        const float* row = data + (i * 6);
        float confidence = row[4];

        if (confidence < confThreshold_) continue;

        // Map coordinates back to original frame (removing padding and scaling)
        float x1 = (row[0] - lb.pad_w) / lb.scale;
        float y1 = (row[1] - lb.pad_h) / lb.scale;
        float x2 = (row[2] - lb.pad_w) / lb.scale;
        float y2 = (row[3] - lb.pad_h) / lb.scale;

        // Clip to original frame boundaries
        x1 = std::clamp(x1, 0.0f, (float)origSize.width);
        y1 = std::clamp(y1, 0.0f, (float)origSize.height);
        x2 = std::clamp(x2, 0.0f, (float)origSize.width);
        y2 = std::clamp(y2, 0.0f, (float)origSize.height);

        Detection det;
        det.confidence = confidence;
        det.classId = static_cast<int>(row[5]);
        det.boundingBox = cv::Rect(cv::Point(static_cast<int>(x1), static_cast<int>(y1)), 
                                   cv::Point(static_cast<int>(x2), static_cast<int>(y2)));

        detections.push_back(det);
    }

    return detections;
}

float PerceptionAgent::aggregateConfidence(const std::vector<Detection>& detections) const {
    float maxConfidence = 0.0F;
    for (const auto& det : detections)
        maxConfidence = std::max(maxConfidence, det.confidence);
    return maxConfidence;
}

} // namespace vigia
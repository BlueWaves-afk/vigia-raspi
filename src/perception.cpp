#include "perception.hpp"
#include <algorithm>
#include <thread>
#include <opencv2/imgproc.hpp>

namespace vigia {

PerceptionAgent::PerceptionAgent(const std::string& modelXmlPath,
                                 const std::string& device) {
    loadNetwork(modelXmlPath, device);
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

/* ===================== Network Loading ===================== */

void PerceptionAgent::loadNetwork(const std::string& modelXmlPath,
                                  const std::string& device) {
    // Read model
    auto model = core_.read_model(modelXmlPath);

    // ✅ Explicitly define layout (THIS FIXES YOUR CRASH)
    model->get_parameters()[0]->set_layout("NCHW");

    // ✅ Now batch is well-defined
    ov::set_batch(model, 1);

    // Compile with latency-focused config
    compiledModel_ = core_.compile_model(
        model,
        device,
        {
            ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
            ov::hint::num_requests(1),
            ov::inference_num_threads(2)
        }
    );

    inferRequest_ = compiledModel_.create_infer_request();
    outputTensor_ = compiledModel_.output(0);

    const auto& inputShape = compiledModel_.input().get_shape();
    inputHeight_ = inputShape[2];
    inputWidth_  = inputShape[3];
}


/* ===================== Inference Logic ===================== */

std::vector<Detection> PerceptionAgent::runInference(const cv::Mat& frame) {
    std::vector<Detection> detections;

    /* ---------- Resize ---------- */
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(inputWidth_, inputHeight_));

    /* ---------- Normalize to FP32 ---------- */
    cv::Mat inputBlob;
    resized.convertTo(inputBlob, CV_32F, 1.0f / 255.0f); // FP32 [0,1]

    /* ---------- HWC → CHW (FP32) ---------- */
    std::vector<float> chw(inputWidth_ * inputHeight_ * 3);
    size_t idx = 0;

    for (int c = 0; c < 3; ++c) {
        for (int y = 0; y < inputHeight_; ++y) {
            for (int x = 0; x < inputWidth_; ++x) {
                chw[idx++] = inputBlob.at<cv::Vec3f>(y, x)[c];
            }
        }
    }

    /* ---------- Bind Tensor ---------- */
    ov::Tensor inputTensor(
        ov::element::f32,
        {1, 3, inputHeight_, inputWidth_},
        chw.data()
    );

    inferRequest_.set_input_tensor(inputTensor);
    inferRequest_.infer();

    /* ---------- Postprocessing ---------- */
    const ov::Tensor output = inferRequest_.get_tensor(outputTensor_);
    const float* outputData = static_cast<const float*>(output.data());

    constexpr int ELEMENTS = 6; // cx, cy, w, h, conf, class
    const int numDetections = static_cast<int>(output.get_shape()[1]);

    for (int i = 0; i < numDetections; ++i) {
        const float confidence = outputData[i * ELEMENTS + 4];
        if (confidence < confThreshold_) continue;

        Detection det;
        det.confidence = confidence;
        det.classId = static_cast<int>(outputData[i * ELEMENTS + 5]);

        float cx = outputData[i * ELEMENTS + 0] * frame.cols;
        float cy = outputData[i * ELEMENTS + 1] * frame.rows;
        float w  = outputData[i * ELEMENTS + 2] * frame.cols;
        float h  = outputData[i * ELEMENTS + 3] * frame.rows;

        det.boundingBox = cv::Rect(
            static_cast<int>(cx - w / 2),
            static_cast<int>(cy - h / 2),
            static_cast<int>(w),
            static_cast<int>(h)
        );

        detections.emplace_back(det);
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
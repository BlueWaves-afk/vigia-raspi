#include "analytical.hpp"

#include <opencv2/imgproc.hpp>
#include <cstring>

namespace vigia {

/* ===================== Constructor ===================== */

AnalyticalAgent::AnalyticalAgent(
    const std::string& modelXmlPath,
    const std::string& device
) {
    loadNetwork(modelXmlPath, device);
}

/* ===================== Network Loading ===================== */

void AnalyticalAgent::loadNetwork(
    const std::string& modelXmlPath,
    const std::string& device
) {
    auto model = core_.read_model(modelXmlPath);

    /* Explicit layout for stability */
    model->get_parameters()[0]->set_layout("NCHW");
    ov::set_batch(model, 1);

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

/* ===================== MiDaS Inference ===================== */

cv::Mat AnalyticalAgent::runInference(const cv::Mat& frame) {
    /* ---------- Resize ---------- */
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(inputWidth_, inputHeight_));

    /* ---------- Normalize to FP32 [0,1] ---------- */
    cv::Mat inputBlob;
    resized.convertTo(inputBlob, CV_32F, 1.0f / 255.0f);

    /* ---------- HWC → CHW ---------- */
    std::vector<float> chw(inputWidth_ * inputHeight_ * 3);
    size_t idx = 0;

    for (int c = 0; c < 3; ++c) {
        for (int y = 0; y < static_cast<int>(inputHeight_); ++y) {
            for (int x = 0; x < static_cast<int>(inputWidth_); ++x) {
                chw[idx++] = inputBlob.at<cv::Vec3f>(y, x)[c];
            }
        }
    }

    /* ---------- Bind Input ---------- */
    ov::Tensor inputTensor(
        ov::element::f32,
        {1, 3, inputHeight_, inputWidth_},
        chw.data()
    );

    inferRequest_.set_input_tensor(inputTensor);
    inferRequest_.infer();

    /* ---------- Retrieve Output ---------- */
    const ov::Tensor output = inferRequest_.get_tensor(outputTensor_);
    const float* depthData =
        static_cast<const float*>(output.data());

    cv::Mat depthMap(
        static_cast<int>(inputHeight_),
        static_cast<int>(inputWidth_),
        CV_32F
    );

    std::memcpy(
        depthMap.data,
        depthData,
        inputWidth_ * inputHeight_ * sizeof(float)
    );

    return depthMap;
}

/* ===================== Phase 2a: ROI Depth Prep ===================== */

cv::Mat AnalyticalAgent::extractDepthROI(
    const cv::Mat& depthMap,
    const cv::Rect& roi
) const {
    cv::Rect boundedROI = roi & cv::Rect(
        0, 0, depthMap.cols, depthMap.rows
    );

    if (boundedROI.empty()) {
        return {};
    }

    cv::Mat roiDepth = depthMap(boundedROI).clone();

    /* Noise suppression */
    cv::medianBlur(roiDepth, roiDepth, 5);

    /* Normalize locally for geometric comparison */
    cv::normalize(
        roiDepth,
        roiDepth,
        0.0f,
        1.0f,
        cv::NORM_MINMAX
    );

    return roiDepth;
}

} // namespace vigia

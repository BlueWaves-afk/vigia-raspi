#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <openvino/openvino.hpp>

namespace vigia {

/**
 * @brief Analytical Agent (Phase 2a)
 * Runs MiDaS monocular depth estimation and prepares
 * geometry-aware depth signals for later verification.
 */
class AnalyticalAgent {
public:
    explicit AnalyticalAgent(
        const std::string& modelXmlPath,
        const std::string& device = "CPU"
    );

    /**
     * @brief Run MiDaS inference on a full frame
     * @param frame BGR input image
     * @return CV_32F depth map (relative inverse depth)
     */
    cv::Mat runInference(const cv::Mat& frame);

    /**
     * @brief Extract and normalize depth inside a region of interest
     * @param depthMap Full-frame depth map from MiDaS
     * @param roi Region of interest (e.g., YOLO bbox)
     * @return CV_32F normalized depth ROI
     */
    cv::Mat extractDepthROI(
        const cv::Mat& depthMap,
        const cv::Rect& roi
    ) const;

private:
    void loadNetwork(
        const std::string& modelXmlPath,
        const std::string& device
    );

private:
    /* ---------- OpenVINO Runtime ---------- */
    ov::Core core_;
    ov::CompiledModel compiledModel_;
    ov::InferRequest inferRequest_;
    ov::Output<const ov::Node> outputTensor_;

    /* ---------- Model Metadata ---------- */
    std::size_t inputWidth_{256};
    std::size_t inputHeight_{256};
};

} // namespace vigia

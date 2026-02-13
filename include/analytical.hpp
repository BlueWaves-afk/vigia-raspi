#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <openvino/openvino.hpp>

namespace vigia {

/* ===================== Phase 2b Output ===================== */
struct DepthResidualStats {
    float meanResidual{0.0f};
    float minResidual{0.0f};
    float stdResidual{0.0f};
};

/* ===================== Phase 3 Output ===================== */
struct DepthGeometryMetrics {
    float depressionScore{0.0f};   // instantaneous geometry signal
    float roughness{0.0f};         // local surface variation
    float persistence{0.0f};       // temporal stability score
};

/**
 * @brief Analytical Agent
 *
 * Phase 1: MiDaS inference
 * Phase 2a: ROI depth extraction
 * Phase 2b: Plane residual analysis
 * Phase 3: Geometry + temporal physics metrics (NO decisions)
 */
class AnalyticalAgent {
public:
    explicit AnalyticalAgent(
        const std::string& modelXmlPath,
        const std::string& device = "CPU"
    );
    AnalyticalAgent(
        ov::Core& sharedCore,
        const std::string& modelXmlPath,
        const std::string& device = "CPU"
    );
    virtual ~AnalyticalAgent() = default;

    /* ---------- Phase 1 ---------- */
    virtual cv::Mat runInference(const cv::Mat& frame);

    /* ---------- Phase 2a ---------- */
    virtual cv::Rect scaleROIToDepth(
        const cv::Rect& roi,
        const cv::Size& originalSize,
        const cv::Size& depthSize
    ) const;

    virtual cv::Mat extractDepthROI(
        const cv::Mat& depthMap,
        const cv::Rect& roi
    ) const;

    /* ---------- Phase 2b ---------- */
    virtual DepthResidualStats computeDepthResiduals(
        const cv::Mat& roiDepth
    ) const;

    /* ---------- Phase 3 ---------- */
    virtual DepthGeometryMetrics computeGeometryMetrics(
        const cv::Mat& roiDepth,
        const DepthResidualStats& residuals
    );

protected:
    AnalyticalAgent() = default;

private:
    void loadNetwork(
        const std::string& modelXmlPath,
        const std::string& device
    );

private:
    /* ---------- OpenVINO ---------- */
    ov::Core ownedCore_;                   // used when no shared core is provided
    ov::Core* core_;                        // points to ownedCore_ or external shared core
    ov::CompiledModel compiledModel_;
    ov::InferRequest inferRequest_;
    ov::Output<const ov::Node> outputTensor_;

    std::size_t inputWidth_{256};
    std::size_t inputHeight_{256};

    /* ---------- Preallocated Buffers ---------- */
    std::vector<float> chwBuffer_;
};

} // namespace vigia

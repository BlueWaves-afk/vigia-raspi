#pragma once

#include <string>
#include <vector>
#include <deque>

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

    /* ---------- Phase 1 ---------- */
    cv::Mat runInference(const cv::Mat& frame);

    /* ---------- Phase 2a ---------- */
    cv::Mat extractDepthROI(
        const cv::Mat& depthMap,
        const cv::Rect& roi
    ) const;

    /* ---------- Phase 2b ---------- */
    DepthResidualStats computeDepthResiduals(
        const cv::Mat& roiDepth
    ) const;

    /* ---------- Phase 3 ---------- */
    DepthGeometryMetrics computeGeometryMetrics(
        const cv::Mat& roiDepth,
        const DepthResidualStats& residuals
    );

private:
    void loadNetwork(
        const std::string& modelXmlPath,
        const std::string& device
    );

    void updateTemporalBuffers(
        float depression,
        float roughness
    );

    float computePersistenceScore() const;

private:
    /* ---------- OpenVINO ---------- */
    ov::Core core_;
    ov::CompiledModel compiledModel_;
    ov::InferRequest inferRequest_;
    ov::Output<const ov::Node> outputTensor_;

    std::size_t inputWidth_{256};
    std::size_t inputHeight_{256};

    /* ---------- Temporal State (Phase 3) ---------- */
    static constexpr std::size_t kHistorySize = 5;
    std::deque<float> depressionHistory_;
    std::deque<float> roughnessHistory_;
};

} // namespace vigia

#pragma once

#include <atomic>
#include <cstdint>

#include <opencv2/core.hpp>

#include "perception.hpp"
#include "safe_queue.hpp"

namespace vigia {

struct CameraIntrinsics {
    float fx{1.0F};
    float fy{1.0F};
    float cx{0.0F};
    float cy{0.0F};
};

struct RansacParameters {
    int iterations{200};
    float distanceThreshold{0.05F};
    float minInlierRatio{0.5F};
};

struct PlaneModel {
    cv::Vec3f normal{0.0F, 1.0F, 0.0F};
    float offset{0.0F};
    float inlierRatio{0.0F};
    float residualMean{0.0F};
};

struct AnalyticalRequest {
    std::uint64_t frameId{0};
    FramePacket framePacket;
    PerceptionResult perception;
};

struct AnalyticalResult {
    std::uint64_t frameId{0};
    PlaneModel plane;
    cv::Mat residualMap;
    float geometricMagnitude{0.0F};
    PerceptionResult perception;
};

class AnalyticalAgent {
public:
    AnalyticalAgent(const CameraIntrinsics& intrinsics,
                    const RansacParameters& params);

    void run(SafeQueue<AnalyticalRequest>& inputQueue,
             SafeQueue<AnalyticalResult>& outputQueue,
             std::atomic<bool>& running);

    const CameraIntrinsics& intrinsics() const { return intrinsics_; }
    const RansacParameters& params() const { return params_; }

private:
    CameraIntrinsics intrinsics_{};
    RansacParameters params_{};
};

PlaneModel fitRoadPlane(const cv::Mat& depthMap,
                        const CameraIntrinsics& intrinsics,
                        const RansacParameters& params,
                        cv::Mat& residualMap,
                        float* geometricMagnitude = nullptr);

} // namespace vigia

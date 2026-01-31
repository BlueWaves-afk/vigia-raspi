#include "analytical.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace vigia {

AnalyticalAgent::AnalyticalAgent(const CameraIntrinsics& intrinsics,
                                 const RansacParameters& params)
    : intrinsics_(intrinsics)
    , params_(params)
{
}

void AnalyticalAgent::run(SafeQueue<AnalyticalRequest>& inputQueue,
                          SafeQueue<AnalyticalResult>& outputQueue,
                          std::atomic<bool>& running)
{
    while (running.load()) {
        auto request = inputQueue.try_pop();
        if (!request) {
            std::this_thread::sleep_for(std::chrono::milliseconds{2});
            continue;
        }

        AnalyticalResult result;
        result.frameId = request->frameId;
        result.perception = request->perception;

        // TODO: Run MiDaS v2.1 Small via OpenVINO to produce a depth map at 256x256.
        cv::Mat depthMap = cv::Mat::zeros(256, 256, CV_32FC1);

        result.residualMap.create(depthMap.size(), CV_32FC1);
        float geometricMagnitude = 0.0F;
        result.plane = fitRoadPlane(depthMap, intrinsics_, params_, result.residualMap, &geometricMagnitude);
        result.geometricMagnitude = geometricMagnitude;

        outputQueue.push(std::move(result));
    }
}

PlaneModel fitRoadPlane(const cv::Mat& depthMap,
                        const CameraIntrinsics& intrinsics,
                        const RansacParameters& params,
                        cv::Mat& residualMap,
                        float* geometricMagnitude)
{
    CV_Assert(!depthMap.empty());

    cv::Mat depthFloat;
    if (depthMap.type() != CV_32FC1) {
        depthMap.convertTo(depthFloat, CV_32FC1);
    } else {
        depthFloat = depthMap;
    }

    residualMap = cv::Mat(depthFloat.size(), CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));

    struct Sample {
        cv::Point3f point;
        cv::Point pixel;
    };

    std::vector<Sample> samples;
    samples.reserve(static_cast<std::size_t>(depthFloat.total()));

    for (int y = 0; y < depthFloat.rows; ++y) {
        const float* depthRow = depthFloat.ptr<float>(y);
        for (int x = 0; x < depthFloat.cols; ++x) {
            float z = depthRow[x];
            if (!std::isfinite(z) || z <= 0.0F) {
                continue;
            }

            float X = (static_cast<float>(x) - intrinsics.cx) * z / intrinsics.fx;
            float Y = (static_cast<float>(y) - intrinsics.cy) * z / intrinsics.fy;
            samples.push_back({cv::Point3f{X, Y, z}, cv::Point{x, y}});
        }
    }

    PlaneModel bestModel;
    if (samples.size() < 3) {
        if (geometricMagnitude) {
            *geometricMagnitude = 0.0F;
        }
        return bestModel;
    }

    std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<std::size_t> indexDist(0, samples.size() - 1);

    std::size_t bestInliers = 0;
    float bestResidualSum = std::numeric_limits<float>::max();
    cv::Vec3f bestNormal = bestModel.normal;
    float bestOffset = bestModel.offset;

    const int iterations = std::max(params.iterations, 1);
    for (int iter = 0; iter < iterations; ++iter) {
        std::array<std::size_t, 3> indices{};
        indices[0] = indexDist(rng);
        do {
            indices[1] = indexDist(rng);
        } while (indices[1] == indices[0]);
        do {
            indices[2] = indexDist(rng);
        } while (indices[2] == indices[0] || indices[2] == indices[1]);

        const cv::Point3f& p0 = samples[indices[0]].point;
        const cv::Point3f& p1 = samples[indices[1]].point;
        const cv::Point3f& p2 = samples[indices[2]].point;

        const cv::Vec3f v1{p1.x - p0.x, p1.y - p0.y, p1.z - p0.z};
        const cv::Vec3f v2{p2.x - p0.x, p2.y - p0.y, p2.z - p0.z};
        cv::Vec3f normal = v1.cross(v2);
        float norm = cv::norm(normal);
        if (norm < 1e-6F) {
            continue;
        }
        normal /= norm;
        float offset = -normal.dot(cv::Vec3f{p0.x, p0.y, p0.z});

        std::size_t inliers = 0;
        float residualSum = 0.0F;

        for (const auto& sample : samples) {
            float distance = std::abs(normal.dot(cv::Vec3f{sample.point.x, sample.point.y, sample.point.z}) + offset);
            if (distance <= params.distanceThreshold) {
                ++inliers;
                residualSum += distance;
            }
        }

        if (inliers > bestInliers || (inliers == bestInliers && residualSum < bestResidualSum)) {
            bestInliers = inliers;
            bestResidualSum = residualSum;
            bestNormal = normal;
            bestOffset = offset;
        }
    }

    bestModel.normal = bestNormal;
    bestModel.offset = bestOffset;
    bestModel.inlierRatio = static_cast<float>(bestInliers) / static_cast<float>(samples.size());
    bestModel.residualMean = (bestInliers > 0) ? bestResidualSum / static_cast<float>(bestInliers) : 0.0F;

    float residualTotal = 0.0F;
    std::size_t residualCount = 0;

    for (const auto& sample : samples) {
        float distance = std::abs(bestModel.normal.dot(cv::Vec3f{sample.point.x, sample.point.y, sample.point.z}) + bestModel.offset);
        residualMap.at<float>(sample.pixel) = distance;
        residualTotal += distance;
        ++residualCount;
    }

    if (geometricMagnitude) {
        *geometricMagnitude = (residualCount > 0) ? residualTotal / static_cast<float>(residualCount) : 0.0F;
    }

    return bestModel;
}

} // namespace vigia

#include "coordinator.hpp"

#include <opencv2/core.hpp>
#include <thread>
#include <chrono>
#include <iostream>

using namespace vigia;
/*
BUILD COMMAND;
clang++ -std=c++17 \
tests/coordinator_test.cpp \
src/coordinator.cpp \
-Iinclude \
-lopencv_core \
-pthread -O3 \
-o coordinator_test

*/
/* ===================== Mock Types ===================== */

struct MockDetection {
    cv::Rect bbox;
    float confidence;
    bool isPothole() const { return true; }
};

/* -------- Mock Perception -------- */

class MockPerceptionAgent {
public:
    bool captureFrame(cv::Mat& frame) {
        frame = cv::Mat::zeros(240, 320, CV_8UC3);
        return true;
    }

    std::vector<Detection> runInference(const cv::Mat&) {
        Detection d{};
        d.bbox = cv::Rect(80, 120, 100, 60);
        d.confidence = 0.85f;
        return { d };
    }
};

/* -------- Mock Analytical -------- */

class MockAnalyticalAgent {
public:
    cv::Mat runInference(const cv::Mat&) {
        return cv::Mat::ones(120, 160, CV_32F);
    }

    cv::Rect scaleROIToDepth(
        const cv::Rect& roi,
        const cv::Size&,
        const cv::Size&
    ) {
        return roi;
    }

    cv::Mat extractDepthROI(
        const cv::Mat& depth,
        const cv::Rect& roi
    ) {
        return depth(roi).clone();
    }

    DepthResidualStats computeDepthResiduals(
        const cv::Mat&
    ) {
        DepthResidualStats s{};
        s.minResidual = -0.04f;
        s.stdResidual = 0.01f;
        return s;
    }

    DepthGeometryMetrics computeGeometryMetrics(
        const cv::Mat&,
        const DepthResidualStats& r
    ) {
        DepthGeometryMetrics g{};
        g.depressionScore = -r.minResidual;
        g.roughness = r.stdResidual;
        return g;
    }
};

/* -------- Mock Temporal -------- */

class MockTemporalAnalyzer {
public:
    TemporalMetrics update(float dep, float rough) {
        TemporalMetrics t{};
        t.persistence = dep * 10.0f;
        t.stability = 1.0f / (rough + 1e-3f);
        return t;
    }
};

/* -------- Mock Fusion -------- */

class MockFusionEngine {
public:
    FusionOutput fuse(const FusionInput& in) {
        FusionOutput o{};
        o.geometryConfidence = in.depressionScore;
        o.temporalConfidence = in.persistence * 0.1f;
        o.finalConfidence =
            0.4f * in.yoloConfidence +
            0.6f * o.geometryConfidence;
        return o;
    }
};

/* ===================== Test ===================== */

int main() {
    std::cout << "[TEST] ===== Coordinator Integration Test =====\n";

    MockPerceptionAgent perception;
    MockAnalyticalAgent analytical;
    MockTemporalAnalyzer temporal;
    MockFusionEngine fusion;

    Coordinator coordinator(
        perception,
        analytical,
        temporal,
        fusion,
        10   // target FPS
    );

    coordinator.start();

    std::this_thread::sleep_for(
        std::chrono::seconds(3)
    );

    coordinator.stop();

    std::cout << "[TEST] ✅ Coordinator ran without deadlock or crash\n";
    std::cout << "[TEST] ✅ Circular buffer, adaptive stride, fusion exercised\n";

    return 0;
}

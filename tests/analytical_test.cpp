#include "analytical.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <iomanip>

using namespace vigia;

/*
BUILD:
clang++ -std=c++17 \
  tests/analytical_test.cpp src/analytical.cpp \
  -Iinclude \
  -I/opt/homebrew/opt/openvino/include \
  -I/opt/homebrew/include/opencv4 \
  -L/opt/homebrew/lib \
  -L/opt/homebrew/opt/openvino/lib \
  -Wl,-rpath,/opt/homebrew/opt/openvino/lib \
  -lopenvino \
  -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio \
  -pthread -O3 \
  -o analytical_test
*/

int main() {
try {
    std::cout << "\n[TEST] ===== Analytical Pipeline Validation =====\n";

    /* --------------------------------------------------
     Phase 1: Load Analytical Agent
    -------------------------------------------------- */
    AnalyticalAgent agent(
        "models/midasv21/openvino_midas_v21_small_256.xml",
        "CPU"
    );

    std::cout << "[TEST] MiDaS model loaded\n";

    /* --------------------------------------------------
     Load Test Image
    -------------------------------------------------- */
    cv::Mat img = cv::imread("pothole_image.jpeg");
    if (img.empty()) {
        std::cerr << "[ERROR] Failed to load pothole_image.jpeg\n";
        return 1;
    }

    std::cout << "[TEST] Image loaded: "
              << img.cols << "x" << img.rows << "\n";

    /* --------------------------------------------------
     Phase 1: MiDaS Inference
    -------------------------------------------------- */
    cv::Mat depthMap = agent.runInference(img);

    if (depthMap.empty() || depthMap.type() != CV_32F) {
        std::cerr << "[ERROR] Invalid depth map\n";
        return 1;
    }

    double globalMin, globalMax;
    cv::minMaxLoc(depthMap, &globalMin, &globalMax);

    std::cout << "[PHASE 1] Depth map OK\n";
    std::cout << "          Global depth range: "
              << globalMin << " → " << globalMax << "\n";

    /* --------------------------------------------------
     Define ROI (simulating YOLO pothole box)
    -------------------------------------------------- */
    cv::Rect roiImg(
        static_cast<int>(img.cols * 0.35),
        static_cast<int>(img.rows * 0.55),
        static_cast<int>(img.cols * 0.30),
        static_cast<int>(img.rows * 0.20)
    );

    float sx = static_cast<float>(depthMap.cols) / img.cols;
    float sy = static_cast<float>(depthMap.rows) / img.rows;

    cv::Rect roiDepthRect(
        roiImg.x * sx,
        roiImg.y * sy,
        roiImg.width * sx,
        roiImg.height * sy
    );

    roiDepthRect &= cv::Rect(0, 0, depthMap.cols, depthMap.rows);

    /* --------------------------------------------------
     Phase 2a: ROI Extraction
    -------------------------------------------------- */
    cv::Mat roiDepth = agent.extractDepthROI(depthMap, roiDepthRect);

    if (roiDepth.empty()) {
        std::cerr << "[ERROR] ROI depth extraction failed\n";
        return 1;
    }

    std::cout << "\n[PHASE 2a] ROI depth extraction OK\n";
    std::cout << "           ROI size: "
              << roiDepth.cols << "x" << roiDepth.rows << "\n";

    /* --------------------------------------------------
     Phase 2b: Plane Residual Analysis
    -------------------------------------------------- */
    DepthResidualStats residuals =
        agent.computeDepthResiduals(roiDepth);

    std::cout << "\n[PHASE 2b] Plane residual stats\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "           Mean residual: "
              << residuals.meanResidual << "\n";
    std::cout << "           Min residual:  "
              << residuals.minResidual << "\n";
    std::cout << "           Std residual:  "
              << residuals.stdResidual << "\n";

    /* --------------------------------------------------
     Phase 3: Geometry Metrics + Temporal Stability
    -------------------------------------------------- */
    std::cout << "\n[PHASE 3] Geometry + temporal metrics\n";

    for (int frame = 1; frame <= 10; ++frame) {
        DepthGeometryMetrics metrics =
            agent.computeGeometryMetrics(roiDepth, residuals);

        std::cout << " Frame " << std::setw(2) << frame
                  << " | depression: "
                  << metrics.depressionScore
                  << " | roughness: "
                  << metrics.roughness
                  << " | persistence: "
                  << metrics.persistence
                  << "\n";
    }

    /* --------------------------------------------------
     Visualization (debug)
    -------------------------------------------------- */
    cv::Mat depthVis;
    cv::normalize(depthMap, depthVis, 0, 255, cv::NORM_MINMAX);
    depthVis.convertTo(depthVis, CV_8U);
    cv::applyColorMap(depthVis, depthVis, cv::COLORMAP_INFERNO);

    cv::rectangle(
        depthVis,
        roiDepthRect,
        cv::Scalar(0, 255, 0),
        2
    );

    cv::imwrite("depth_debug_roi.png", depthVis);

    std::cout << "\n[TEST] Saved depth_debug_roi.png\n";
    std::cout << "[TEST] ✅ Analytical pipeline fully validated\n\n";
}
catch (const ov::Exception& e) {
    std::cerr << "[OpenVINO ERROR] " << e.what() << "\n";
    return 2;
}
catch (const std::exception& e) {
    std::cerr << "[STD ERROR] " << e.what() << "\n";
    return 3;
}

return 0;
}

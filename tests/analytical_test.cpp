#include "analytical.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

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
    std::cout << "[TEST] Starting Analytical (MiDaS) inference test\n";

    /* -------- Load Analytical Agent -------- */
    AnalyticalAgent agent(
        "models/midasv21/openvino_midas_v21_small_256.xml",
        "CPU"
    );

    std::cout << "[TEST] MiDaS model loaded\n";

    /* -------- Load Test Image -------- */
    cv::Mat img = cv::imread("pothole_image.jpeg");
    if (img.empty()) {
        std::cerr << "[ERROR] Failed to load test image\n";
        return 1;
    }

    std::cout << "[TEST] Image loaded: "
              << img.cols << "x" << img.rows << "\n";

    /* -------- Run MiDaS Inference -------- */
    cv::Mat depthMap = agent.runInference(img);

    if (depthMap.empty() || depthMap.type() != CV_32F) {
        std::cerr << "[ERROR] Invalid depth map\n";
        return 1;
    }

    /* -------- Global Depth Stats -------- */
    double globalMin, globalMax;
    cv::minMaxLoc(depthMap, &globalMin, &globalMax);

    std::cout << "[TEST] Global depth range: "
              << globalMin << " → " << globalMax << "\n";

    /* =======================================================
       ROI DEPTH TEST (Simulates YOLO pothole box)
       ======================================================= */

    // 🔧 Manually define ROI (adjust to your image)
    cv::Rect roi(
        img.cols * 0.35,
        img.rows * 0.55,
        img.cols * 0.3,
        img.rows * 0.2
    );

    // Scale ROI to depth map resolution
    float sx = static_cast<float>(depthMap.cols) / img.cols;
    float sy = static_cast<float>(depthMap.rows) / img.rows;

    cv::Rect depthROI(
        roi.x * sx,
        roi.y * sy,
        roi.width * sx,
        roi.height * sy
    );

    depthROI &= cv::Rect(0, 0, depthMap.cols, depthMap.rows);

    cv::Mat roiDepth = depthMap(depthROI);

    /* -------- ROI Statistics -------- */
    cv::Scalar mean, stddev;
    cv::meanStdDev(roiDepth, mean, stddev);

    double roiMin, roiMax;
    cv::minMaxLoc(roiDepth, &roiMin, &roiMax);

    std::cout << "\n[ROI DEPTH ANALYSIS]\n";
    std::cout << " ROI Min: " << roiMin << "\n";
    std::cout << " ROI Max: " << roiMax << "\n";
    std::cout << " ROI Mean: " << mean[0] << "\n";
    std::cout << " ROI StdDev: " << stddev[0] << "\n";

    /* -------- Visualization -------- */
    cv::Mat depthVis;
    cv::normalize(depthMap, depthVis, 0, 255, cv::NORM_MINMAX);
    depthVis.convertTo(depthVis, CV_8U);
    cv::applyColorMap(depthVis, depthVis, cv::COLORMAP_INFERNO);

    // Draw ROI box
    cv::rectangle(
        depthVis,
        depthROI,
        cv::Scalar(0, 255, 0),
        2
    );

    cv::imwrite("depth_debug_roi.png", depthVis);

    std::cout << "\n[TEST] Saved depth_debug_roi.png\n";
    std::cout << "[TEST] ✅ ROI depth analysis complete\n";
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

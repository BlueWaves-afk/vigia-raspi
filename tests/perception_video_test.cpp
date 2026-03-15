/**
 * @file perception_video_test.cpp
 * @brief YOLO-only detection benchmark — no MiDaS, no temporal, no fusion.
 *
 * Optimized for maximum FPS / minimum latency on Raspberry Pi 4:
 *   - INT8 model by default (2.3 MB, lowest latency)
 *   - Shared ov::Core (single plugin init)
 *   - grab()+retrieve() skip: only decodes when inference has consumed the last frame
 *   - Pre-allocated canvas reused every frame (no per-frame clone)
 *   - snprintf labels (no ostringstream)
 *   - EMA FPS counter (α=0.1)
 *   - Headless mode (--headless) for pure throughput measurement
 *
 * Usage:
 *   perception_video_test [--headless] <video.mp4> [yolo_xml]
 */

#include "perception.hpp"

#include <opencv2/core/ocl.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <openvino/openvino.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
static void pinToCore(int core) {
    cpu_set_t s; CPU_ZERO(&s); CPU_SET(core, &s);
    pthread_setaffinity_np(pthread_self(), sizeof(s), &s);
}
#else
static void pinToCore(int) {}
#endif

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: perception_video_test [--headless] <video.mp4> [yolo_xml]\n";
        return 1;
    }

    bool headless = false;
    bool useFp32 = false;
    int argIdx = 1;
    if (argIdx < argc && std::string(argv[argIdx]) == "--headless") { headless = true; ++argIdx; }
    if (argIdx < argc && std::string(argv[argIdx]) == "--fp32")     { useFp32  = true; ++argIdx; }
    if (argIdx >= argc) {
        std::cerr << "Usage: perception_video_test [--headless] [--fp32] <video.mp4> [yolo_xml]\n"
                  << "  --fp32   Use FP32 model (default: INT8)\n";
        return 1;
    }

    const std::string videoPath = argv[argIdx++];
    const std::string modelPath = (argIdx < argc)
        ? argv[argIdx]
        : (useFp32 ? "models/yolo26/yolo26_model.xml"
                   : "models/yolo26/yolo26_model_int8.xml");

    // ── OpenCV: let TBB use all cores, disable OpenCL ──────────────
    cv::setNumThreads(0);
    cv::ocl::setUseOpenCL(false);

    // ── Pin main thread to Core 1 (inference core on Pi 4) ─────────
    pinToCore(1);

    // ── Shared ov::Core — single plugin init ────────────────────────
    auto corePtr = std::make_shared<ov::Core>();
    ov::Core& core = *corePtr;
    try { core.set_property("CPU", ov::enable_mmap(false)); } catch (...) {}
    try { core.set_property("CPU", ov::num_streams(1)); } catch (...) {}
    try { core.set_property("CPU", ov::inference_num_threads(4)); } catch (...) {}

    vigia::PerceptionAgent agent(core, modelPath, "CPU");

    if (!agent.isModelLoaded()) {
        std::cerr << "[ERROR] Model failed to compile: " << modelPath << "\n";
        return 1;
    }

    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Cannot open video: " << videoPath << "\n";
        return 1;
    }

    if (!headless)
        cv::namedWindow("YOLO", cv::WINDOW_NORMAL);

    // Pre-allocated canvas — reused every frame, no per-frame clone
    cv::Mat canvas;
    char buf[64];

    // EMA FPS
    double emaFps = 0.0;
    constexpr double kAlpha = 0.1;
    uint64_t frameCount = 0;
    double totalLatencyMs = 0.0;
    double minLatencyMs = 1e9, maxLatencyMs = 0.0;
    std::vector<double> latencies;  // for P95
    latencies.reserve(1024);

    using clock = std::chrono::steady_clock;
    const auto runStart = clock::now();

    while (true) {
        if (!cap.grab()) break;

        cv::Mat raw;
        if (!cap.retrieve(raw) || raw.empty()) break;

        const auto t0 = clock::now();
        auto detections = agent.runInference(raw);
        const double latMs = std::chrono::duration<double, std::milli>(clock::now() - t0).count();

        ++frameCount;
        totalLatencyMs += latMs;
        latencies.push_back(latMs);
        if (latMs < minLatencyMs) minLatencyMs = latMs;
        if (latMs > maxLatencyMs) maxLatencyMs = latMs;
        const double instantFps = 1000.0 / latMs;
        emaFps = (frameCount == 1) ? instantFps : kAlpha * instantFps + (1.0 - kAlpha) * emaFps;

        if (!headless) {
            // Reuse canvas allocation — copyTo only reallocates if size changes
            raw.copyTo(canvas);

            for (const auto& det : detections) {
                const cv::Rect& box = det.boundingBox;
                if (box.width <= 0 || box.height <= 0) continue;
                const bool hazard = det.confidence >= 0.55f;
                const cv::Scalar color = hazard ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 200, 80);
                cv::rectangle(canvas, box, color, 2);
                std::snprintf(buf, sizeof(buf), "%.2f", det.confidence);
                cv::putText(canvas, buf,
                            cv::Point(box.x, std::max(box.y - 6, 12)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_8);
            }

            // HUD: FPS + latency
            std::snprintf(buf, sizeof(buf), "FPS:%.1f  Lat:%.1fms  Det:%zu",
                          emaFps, latMs, detections.size());
            cv::putText(canvas, buf, cv::Point(8, 22),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_8);

            cv::imshow("YOLO", canvas);
            if (cv::waitKey(1) == 'q') break;
        }
    }

    if (!headless) cv::destroyAllWindows();

    const double totalSec = std::chrono::duration<double>(clock::now() - runStart).count();
    const double avgLatency = frameCount > 0 ? totalLatencyMs / frameCount : 0.0;
    const double avgFps = totalSec > 0.0 ? frameCount / totalSec : 0.0;

    // P95 latency
    double p95 = 0.0;
    if (!latencies.empty()) {
        std::sort(latencies.begin(), latencies.end());
        p95 = latencies[static_cast<std::size_t>(latencies.size() * 0.95)];
    }

    std::printf("\n");
    std::printf("╔══════════════════════════════════════════╗\n");
    std::printf("║         YOLO Detection Benchmark         ║\n");
    std::printf("╠══════════════════════════════════════════╣\n");
    std::printf("║  Model    : %-29s║\n", (useFp32 ? "FP32" : "INT8 (quantized)"));
    std::printf("║  Runtime  : OpenVINO 2025 CPU            ║\n");
    std::printf("║  Source   : %-29s║\n", videoPath.c_str());
    std::printf("╠══════════════════════════════════════════╣\n");
    std::printf("║  Frames processed : %-20llu║\n", (unsigned long long)frameCount);
    std::printf("║  Total runtime    : %-17.2f s  ║\n", totalSec);
    std::printf("╠══════════════════════════════════════════╣\n");
    std::printf("║  FPS (EMA smooth) : %-20.2f║\n", emaFps);
    std::printf("║  FPS (avg)        : %-20.2f║\n", avgFps);
    std::printf("╠══════════════════════════════════════════╣\n");
    std::printf("║  Latency avg      : %-17.2f ms ║\n", avgLatency);
    std::printf("║  Latency min      : %-17.2f ms ║\n", minLatencyMs);
    std::printf("║  Latency P95      : %-17.2f ms ║\n", p95);
    std::printf("║  Latency max      : %-17.2f ms ║\n", maxLatencyMs);
    std::printf("╚══════════════════════════════════════════╝\n");

    return 0;
}

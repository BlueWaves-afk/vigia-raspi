/**
 * @file yolo_benchmark_test.cpp
 * @brief Pure YOLO26 inference benchmark on video file
 *
 * Measures raw YOLO inference throughput with zero overhead from MiDaS,
 * fusion, temporal, coordinator, or instrumentation bus.  Uses the same
 * PerceptionAgent (perception.cpp) that the full pipeline uses, so all
 * ARM optimizations are active:
 *
 *   • KleidiCV 0.7.0 HAL — NEON-accelerated cv::resize / cv::cvtColor
 *   • TBB parallel backend — multi-core dispatch for OpenCV internals
 *   • OpenVINO + ACL — ARM Compute Library inference on Cortex-A72
 *   • Pre-allocated ov::Tensor — zero per-frame heap allocation
 *   • Async start_async()+wait() — pipelined inference
 *   • FP32 precision — avoids FP16 alignment traps on Pi 4
 *   • SIGBUS recovery — graceful degradation if ACL is missing
 *
 * Usage:
 *   ./yolo_benchmark_test --video hazard.mp4 [model.xml]
 *   ./yolo_benchmark_test --cam [index] [model.xml]
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <openvino/openvino.hpp>

#include "perception.hpp"

#ifdef __linux__
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#endif

using namespace vigia;
using Clock = std::chrono::steady_clock;

/* ── Helpers ──────────────────────────────────────────────────────────── */

static constexpr int POTHOLE_CLASS_ID = 0;

struct FrameStats {
    double inferMs{0.0};
    int    detections{0};
    int    potholes{0};
    float  maxConf{0.0f};
};

static void printHistogram(const std::vector<double>& latencies) {
    if (latencies.size() < 2) return;

    // Use P1–P99 range to avoid outlier-stretching
    const std::size_t n = latencies.size();
    const double lo = latencies[static_cast<std::size_t>(n * 0.01)];
    const double hi = latencies[static_cast<std::size_t>(n * 0.99)];
    if (hi <= lo) return;

    constexpr int BINS = 10;
    const double binWidth = (hi - lo) / BINS;

    std::vector<int> counts(BINS, 0);
    int outlierLow = 0, outlierHigh = 0;
    for (double v : latencies) {
        if (v < lo) { outlierLow++; continue; }
        if (v > hi) { outlierHigh++; continue; }
        int bin = static_cast<int>((v - lo) / binWidth);
        bin = std::clamp(bin, 0, BINS - 1);
        counts[static_cast<std::size_t>(bin)]++;
    }

    const int maxCount = *std::max_element(counts.begin(), counts.end());
    constexpr int BAR_WIDTH = 36;

    std::cout << "\n── Latency Distribution (P1–P99) ──\n";
    for (int i = 0; i < BINS; ++i) {
        const double edge = lo + i * binWidth;
        const int barLen = maxCount > 0 ? (counts[static_cast<std::size_t>(i)] * BAR_WIDTH / maxCount) : 0;
        std::cout << std::fixed << std::setprecision(1) << std::setw(7) << edge << " ms │";
        for (int j = 0; j < barLen; ++j) std::cout << "#";
        std::cout << ' ' << counts[static_cast<std::size_t>(i)] << '\n';
    }
    if (outlierLow > 0 || outlierHigh > 0)
        std::cout << "  (outliers: " << outlierLow << " below, " << outlierHigh << " above)\n";
    std::cout << '\n';
}

/* ── Main ─────────────────────────────────────────────────────────────── */

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: yolo_benchmark_test (--video <file.mp4> | --cam [index]) [model.xml]\n";
        return 1;
    }

    // ── Parse arguments ─────────────────────────────────────────────
    bool useCamera = false;
    int cameraIndex = 0;
    std::string videoPath;
    int argIdx = 1;

    const std::string mode = argv[argIdx++];
    if (mode == "--cam") {
        useCamera = true;
        if (argIdx < argc && argv[argIdx][0] != '-') {
            try { cameraIndex = std::stoi(argv[argIdx++]); }
            catch (...) { std::cerr << "Bad camera index\n"; return 1; }
        }
    } else if (mode == "--video") {
        if (argIdx >= argc) { std::cerr << "--video requires a path\n"; return 1; }
        videoPath = argv[argIdx++];
    } else {
        std::cerr << "Usage: yolo_benchmark_test (--video <file.mp4> | --cam [index]) [model.xml]\n";
        return 1;
    }

    const std::string modelPath = (argIdx < argc)
        ? argv[argIdx++]
        : "models/yolo26/yolo26_model.xml";

    // ── OpenCV + threading setup ────────────────────────────────────
    // Let TBB use all cores for KleidiCV HAL (cv::resize, cvtColor, etc.)
    cv::setNumThreads(0);
    cv::ocl::setUseOpenCL(false);

#ifdef __linux__
    // Pin benchmark thread to core 2 (avoid sharing with GPU IRQ on core 0)
    cpu_set_t cpuSet;
    CPU_ZERO(&cpuSet);
    CPU_SET(2, &cpuSet);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuSet);
#endif

    // ── OpenVINO Core (heap-allocated for ARM64 alignment safety) ──
    auto corePtr = std::make_shared<ov::Core>();
    ov::Core& core = *corePtr;

    // Force FP32 — prevents FP16 alignment traps on Cortex-A72
    try { core.set_property("CPU", ov::hint::inference_precision(ov::element::f32)); }
    catch (...) {}

    // Disable mmap — prevents SIGBUS from SD-card mmap page faults
    try { core.set_property("CPU", ov::enable_mmap(false)); }
    catch (...) {}

    // Log device info
    try {
        const std::string devName = core.get_property("CPU", ov::device::full_name);
        std::cout << "[YOLO-BENCH] Device: " << devName << '\n';
    } catch (...) {}

    // ── Construct PerceptionAgent with shared core ──────────────────
    std::cout << "[YOLO-BENCH] Loading model: " << modelPath << '\n';
    PerceptionAgent agent(core, modelPath, "CPU");
    if (!agent.isModelLoaded()) {
        std::cerr << "[YOLO-BENCH] FATAL: Model failed to compile (ACL missing?)\n";
        return 1;
    }
    std::cout << "[YOLO-BENCH] Model compiled OK\n";

    // ── Open video / camera ─────────────────────────────────────────
    cv::VideoCapture cap;
    if (useCamera) {
        if (!cap.open(cameraIndex)) {
            std::cerr << "[YOLO-BENCH] Failed to open camera " << cameraIndex << '\n';
            return 1;
        }
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    } else {
        if (!cap.open(videoPath)) {
            std::cerr << "[YOLO-BENCH] Failed to open video: " << videoPath << '\n';
            return 1;
        }
    }

    const int totalFrames = useCamera ? 0 : static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    const double videoFps = cap.get(cv::CAP_PROP_FPS);
    const int frameW = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int frameH = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    std::cout << "[YOLO-BENCH] Source: " << (useCamera ? "camera" : videoPath) << '\n';
    if (!useCamera)
        std::cout << "[YOLO-BENCH] Frames: " << totalFrames
                  << "  FPS: " << videoFps
                  << "  Resolution: " << frameW << "x" << frameH << '\n';

    // ── Benchmark loop ──────────────────────────────────────────────
    std::vector<FrameStats> stats;
    std::vector<double> latencies;
    stats.reserve(useCamera ? 1000 : static_cast<std::size_t>(totalFrames));
    latencies.reserve(stats.capacity());

    int frameCount = 0;
    int totalPotholes = 0;
    float peakConfidence = 0.0f;
    bool shouldQuit = false;

    const auto benchStart = Clock::now();

    std::cout << "[YOLO-BENCH] Running inference on every frame...\n\n";

    while (!shouldQuit) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty())
            break;

        frameCount++;

        // ── Inference (timed) ───────────────────────────────────────
        const auto t0 = Clock::now();
        auto detections = agent.runInference(frame);
        const auto t1 = Clock::now();

        const double inferMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // ── Collect stats ───────────────────────────────────────────
        FrameStats fs;
        fs.inferMs = inferMs;
        fs.detections = static_cast<int>(detections.size());
        fs.potholes = 0;
        fs.maxConf = 0.0f;

        for (const auto& d : detections) {
            fs.maxConf = std::max(fs.maxConf, d.confidence);
            if (d.classId == POTHOLE_CLASS_ID)
                fs.potholes++;
        }

        totalPotholes += fs.potholes;
        peakConfidence = std::max(peakConfidence, fs.maxConf);
        stats.push_back(fs);
        latencies.push_back(inferMs);

        // ── Progress (every 50 frames) ──────────────────────────────
        if (frameCount % 50 == 0 || frameCount == 1) {
            const double elapsed = std::chrono::duration<double>(Clock::now() - benchStart).count();
            const double fps = frameCount / elapsed;
            std::cout << "\r  Frame " << std::setw(5) << frameCount;
            if (totalFrames > 0)
                std::cout << " / " << totalFrames
                          << " (" << std::setw(3) << (frameCount * 100 / totalFrames) << "%)";
            std::cout << "  │ " << std::fixed << std::setprecision(1) << inferMs << " ms"
                      << "  │ " << std::setprecision(2) << fps << " fps"
                      << "  │ det=" << fs.detections
                      << " pot=" << fs.potholes
                      << std::flush;
        }

        // Allow 'q' to quit early (1ms waitKey for keyboard poll)
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
            shouldQuit = true;
    }

    const auto benchEnd = Clock::now();
    const double totalSec = std::chrono::duration<double>(benchEnd - benchStart).count();

    cap.release();

    // ── Compute statistics ──────────────────────────────────────────
    if (latencies.empty()) {
        std::cerr << "\n[YOLO-BENCH] No frames processed!\n";
        return 1;
    }

    std::sort(latencies.begin(), latencies.end());
    const std::size_t n = latencies.size();

    const double avgMs   = std::accumulate(latencies.begin(), latencies.end(), 0.0) / static_cast<double>(n);
    const double minMs   = latencies.front();
    const double maxMs   = latencies.back();
    const double medMs   = latencies[n / 2];
    const double p95Ms   = latencies[static_cast<std::size_t>(n * 0.95)];
    const double p99Ms   = latencies[static_cast<std::size_t>(n * 0.99)];
    const double stddev  = std::sqrt(
        std::accumulate(latencies.begin(), latencies.end(), 0.0,
            [avgMs](double acc, double v) { return acc + (v - avgMs) * (v - avgMs); })
        / static_cast<double>(n));
    const double avgFps  = static_cast<double>(n) / totalSec;

    // Warmup cost: first frame vs average
    const double warmupMs = stats[0].inferMs;

    // Detection rate
    int framesWithDetections = 0;
    int framesWithPotholes = 0;
    for (const auto& s : stats) {
        if (s.detections > 0) framesWithDetections++;
        if (s.potholes > 0) framesWithPotholes++;
    }

    // ── Print report ────────────────────────────────────────────────
    std::cout << "\n\n"
        << "╔══════════════════════════════════════════════════════════════╗\n"
        << "║              YOLO26 Pure Inference Benchmark                ║\n"
        << "╠══════════════════════════════════════════════════════════════╣\n"
        << "║                                                              ║\n"
        << "║  Source       : " << std::setw(42) << std::left << (useCamera ? "camera" : videoPath) << " ║\n"
        << "║  Resolution   : " << frameW << "x" << frameH << std::setw(34) << "" << " ║\n"
        << "║  Model        : " << std::setw(42) << modelPath << " ║\n"
        << "║                                                              ║\n"
        << "╠══════════════════════════════════════════════════════════════╣\n"
        << "║  TIMING                                                      ║\n"
        << "╠══════════════════════════════════════════════════════════════╣\n"
        << std::fixed << std::setprecision(2)
        << "║  Frames       : " << std::setw(42) << frameCount << " ║\n"
        << "║  Total time   : " << std::setw(39) << totalSec << " s  ║\n"
        << "║  Throughput   : " << std::setw(37) << avgFps << " fps  ║\n"
        << "║  Warmup (1st) : " << std::setw(38) << warmupMs << " ms  ║\n"
        << "║                                                              ║\n"
        << "╠══════════════════════════════════════════════════════════════╣\n"
        << "║  LATENCY (per-frame inference only)                          ║\n"
        << "╠══════════════════════════════════════════════════════════════╣\n"
        << "║  Average      : " << std::setw(38) << avgMs << " ms  ║\n"
        << "║  Median       : " << std::setw(38) << medMs << " ms  ║\n"
        << "║  Min          : " << std::setw(38) << minMs << " ms  ║\n"
        << "║  Max          : " << std::setw(38) << maxMs << " ms  ║\n"
        << "║  Std dev      : " << std::setw(38) << stddev << " ms  ║\n"
        << "║  P95          : " << std::setw(38) << p95Ms << " ms  ║\n"
        << "║  P99          : " << std::setw(38) << p99Ms << " ms  ║\n"
        << "║                                                              ║\n"
        << "╠══════════════════════════════════════════════════════════════╣\n"
        << "║  DETECTIONS                                                  ║\n"
        << "╠══════════════════════════════════════════════════════════════╣\n"
        << "║  Frames w/ det: " << framesWithDetections << " / " << frameCount
            << " (" << (frameCount > 0 ? framesWithDetections * 100 / frameCount : 0) << "%)"
            << std::setw(25) << "" << " ║\n"
        << "║  Frames w/ pot: " << framesWithPotholes << " / " << frameCount
            << " (" << (frameCount > 0 ? framesWithPotholes * 100 / frameCount : 0) << "%)"
            << std::setw(25) << "" << " ║\n"
        << "║  Total potholes: " << std::setw(41) << totalPotholes << " ║\n"
        << "║  Peak conf    : " << std::setw(42) << peakConfidence << " ║\n"
        << "║                                                              ║\n"
        << "╚══════════════════════════════════════════════════════════════╝\n";

    printHistogram(latencies);

    return 0;
}

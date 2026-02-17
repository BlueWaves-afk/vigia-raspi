/**
 * @file system_sequential_test.cpp
 * @brief VIGIA single-threaded sequential pipeline — benchmark-parity visual test
 *
 * Optimized for Raspberry Pi 4 (Cortex-A72 / aarch64 / Debian Trixie)
 *   • OpenCV 4.14  — KleidiCV 0.7.0 HAL + TBB parallel backend
 *   • OpenVINO 2025 — CPU plugin with KleidiAI + NEON backend
 *
 * Motivation
 * ──────────
 *  system_visual_test uses a multi-threaded Coordinator with capture /
 *  process / UI threads and an InstrumentationBus ring buffer.  On Pi 4,
 *  the inter-thread handoff overhead, lock contention, and cross-core
 *  cache invalidation add ~40–60 ms of latency per frame — enough to drop
 *  from 10 fps (benchmark-measured) to ~4–6 fps in the visual test.
 *
 *  This file collapses everything into a single sequential loop that
 *  mirrors performance_benchmark_test.cpp exactly:
 *
 *    capture → YOLO → (MiDaS?) → fusion/temporal → draw → imshow
 *
 *  Because all operations run on one core with no mutex round-trips,
 *  the L1/L2 hot data (OpenVINO tensor buffers, cv::Mat headers, YOLO
 *  output vectors) stays resident across the entire pipeline pass.
 *
 * Architecture delta vs system_visual_test
 * ─────────────────────────────────────────
 *  Removed:
 *    • Coordinator / InstrumentationBus / ring buffer
 *    • InstrumentedPerceptionAgent / InstrumentedAnalyticalAgent wrappers
 *    • Five-panel dashboard (expensive composite)
 *    • std::thread, std::mutex (except OpenVINO-internal)
 *
 *  Kept / reused (zero API changes needed):
 *    • PerceptionAgent, AnalyticalAgent — called synchronously
 *    • TemporalAnalyzer, FusionEngine   — same call sites as benchmark
 *    • adaptiveStride()                 — verbatim from benchmark
 *    • PerfTracker (EMA FPS, latency stats)
 *    • SIGBUS handler + FP32 / mmap guards
 *    • pinCurrentThreadToCore()
 *    • readCPUTemperature() / readRssMb()
 *
 * Display
 * ───────
 *  "Visual Overlay" mode — detection boxes and a compact HUD are drawn
 *  directly onto a clone of the source frame.  Total rendering cost is
 *  < 1 ms vs ~15–20 ms for the 5-panel composite.
 *
 * Usage
 * ─────
 *   ./system_sequential_test [-F] (--video <path> | --cam [idx]) \
 *       [yolo_xml] [midas_xml] [target_fps]
 *
 *   -F              Start in fullscreen (default: windowed 960×540)
 *   --video <path>  Use a video file as source
 *   --cam [idx]     Use libcamera via GStreamer (default index 0)
 *   yolo_xml        Path to YOLO .xml (default: models/yolo26/yolo26_model_int8.xml)
 *   midas_xml       Path to MiDaS .xml (default: models/midasv21/openvino_midas_v21_small_256.xml)
 *   target_fps      Target FPS for adaptive stride (default: 15)
 *
 * Keys
 * ────
 *   q / Q    Quit
 *   Space    Toggle pause
 *   s / S    Single-step (while paused)
 *   d / D    Toggle depth map overlay
 *   f / F    Toggle fullscreen
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <ctime>

// unistd.h is available on all POSIX platforms (Linux + macOS).
// Only skip on Windows — no SIGBUS there anyway.
#if defined(__linux__) || defined(__APPLE__)
#  include <unistd.h>          // write(), _exit(), STDERR_FILENO
#endif

#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <openvino/openvino.hpp>

#include "analytical.hpp"
#include "fusion.hpp"
#include "perception.hpp"
#include "roi_utils.hpp"
#include "temporal.hpp"

// ═══════════════════════════════════════════════════════════════════════════
//  Compile-time platform profile
// ═══════════════════════════════════════════════════════════════════════════

#if defined(__aarch64__) || defined(__ARM_NEON)
static constexpr bool kArmProfile = true;
#else
static constexpr bool kArmProfile = false;
#endif

namespace vigia {
namespace seq {
namespace {

// ─── Constants ──────────────────────────────────────────────────────────────

constexpr int   POTHOLE_CLASS_ID   = 0;
constexpr float HAZARD_THRESHOLD   = 0.55f;
constexpr float TEMP_WARN_C        = 75.0f;
constexpr float TEMP_CRITICAL_C    = 85.0f;
constexpr int   WARMUP_FRAMES      = 5;
constexpr int   DISPLAY_W          = 960;
constexpr int   DISPLAY_H          = 540;
constexpr int   RENDER_INTERVAL_MS = 33;   // ~30 fps display cap

// EMA smoothing factor — matches benchmark exactly.
constexpr double kFpsEmaAlpha = 0.1;

// ─── Thread pinning (Linux / Pi 4 only) ─────────────────────────────────────

#ifdef __linux__
#  include <pthread.h>
#  include <sched.h>
inline void pinCurrentThreadToCore(int coreId) {
    cpu_set_t cs;
    CPU_ZERO(&cs);
    CPU_SET(coreId, &cs);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cs);
}
#else
inline void pinCurrentThreadToCore(int) {}
#endif

} // anonymous namespace

// ═══════════════════════════════════════════════════════════════════════════
//  PerfTracker — identical to system_visual_test version
// ═══════════════════════════════════════════════════════════════════════════

class PerfTracker {
public:
    PerfTracker() : startTs_(std::chrono::steady_clock::now()) {}

    void recordFrame(double latencyMs) {
        if (latencyMs <= 0.0) return;

        const double instantFps = 1000.0 / latencyMs;

        if (frameCount_ == 0) {
            emaFps_ = instantFps;
            firstInferenceLatencyMs_ =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - startTs_).count();
        } else {
            emaFps_ = kFpsEmaAlpha * instantFps + (1.0 - kFpsEmaAlpha) * emaFps_;
        }

        cumulativeLatencyMs_ += latencyMs;
        ++frameCount_;

        if (latencyMs < minLatencyMs_) minLatencyMs_ = latencyMs;
        if (latencyMs > maxLatencyMs_) maxLatencyMs_ = latencyMs;
    }

    double smoothedFps()  const { return emaFps_; }
    double avgLatencyMs() const { return frameCount_ > 0 ? cumulativeLatencyMs_ / frameCount_ : 0.0; }
    double minLatencyMs() const { return minLatencyMs_; }
    double maxLatencyMs() const { return maxLatencyMs_; }
    double firstInferMs() const { return firstInferenceLatencyMs_; }
    uint64_t frames()     const { return frameCount_; }

private:
    std::chrono::steady_clock::time_point startTs_;
    double   emaFps_{0.0};
    double   cumulativeLatencyMs_{0.0};
    double   firstInferenceLatencyMs_{0.0};
    double   minLatencyMs_{std::numeric_limits<double>::max()};
    double   maxLatencyMs_{0.0};
    uint64_t frameCount_{0};
};

// ═══════════════════════════════════════════════════════════════════════════
//  System helpers — verbatim from benchmark / visual test
// ═══════════════════════════════════════════════════════════════════════════

inline float readCPUTemperature() {
#ifdef __linux__
    std::ifstream f("/sys/class/thermal/thermal_zone0/temp");
    if (!f.is_open()) return std::numeric_limits<float>::quiet_NaN();
    float milli = 0.0f;
    f >> milli;
    return milli / 1000.0f;
#else
    return std::numeric_limits<float>::quiet_NaN();
#endif
}

inline float readRssMb() {
#ifdef __linux__
    std::ifstream f("/proc/self/status");
    if (!f.is_open()) return 0.0f;
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            long kb = 0;
            std::sscanf(line.c_str(), "VmRSS: %ld kB", &kb);
            return static_cast<float>(kb) / 1024.0f;
        }
    }
#endif
    return 0.0f;
}

// ═══════════════════════════════════════════════════════════════════════════
//  adaptiveStride — verbatim from performance_benchmark_test.cpp
//
//  Raises MiDaS stride when the previous frame exceeded targetMs or when
//  the SoC approaches thermal limits.  Lowers stride when headroom exists.
// ═══════════════════════════════════════════════════════════════════════════

static int adaptiveStride(int cur, float tempC, long elapsedMs, long targetMs) {
    if (tempC > TEMP_CRITICAL_C) return 5;
    if (tempC > TEMP_WARN_C)     return 3;
    if (elapsedMs > targetMs)    return std::min(cur + 1, 5);
    return std::max(1, cur - 1);
}

// ═══════════════════════════════════════════════════════════════════════════
//  FrameStats — per-frame telemetry passed to the renderer
// ═══════════════════════════════════════════════════════════════════════════

struct PotholeTelemetry {
    cv::Rect  box{};
    float     yoloConf{0.0f};
    float     fusionConf{0.0f};
    float     geoConf{0.0f};
    float     depression{0.0f};
    float     roughness{0.0f};
    float     persistence{0.0f};
    float     stability{0.0f};
    bool      hazard{false};
};

struct FrameStats {
    int    frameNum{0};
    double yoloMs{0.0};
    double midasMs{0.0};
    double fusionMs{0.0};
    double totalMs{0.0};
    double smoothFps{0.0};
    float  tempC{0.0f};
    float  rssMb{0.0f};
    int    stride{1};
    bool   ranMidas{false};
    int    totalDetections{0};
    std::vector<PotholeTelemetry> potholes;
};

// ═══════════════════════════════════════════════════════════════════════════
//  Overlay renderer — O(detections) with TBB parallel_for_
//
//  All drawing targets a pre-cloned mutable canvas so we never modify the
//  raw capture buffer.  The HUD block is rendered with a dim-multiply over
//  a fixed top-left ROI to remain readable regardless of background colour.
// ═══════════════════════════════════════════════════════════════════════════

static void drawOverlay(cv::Mat& canvas,
                        const FrameStats& stats,
                        bool showDepthHint,
                        const PerfTracker& perf) {

    if (canvas.empty()) return;

    // ── HUD bar (top-left) ───────────────────────────────────────────────
    {
        const int hudW = std::min(440, canvas.cols);
        const int hudH = std::min(84,  canvas.rows);
        cv::Mat roi = canvas(cv::Rect(0, 0, hudW, hudH));
        cv::multiply(roi, cv::Scalar(0.30, 0.30, 0.30), roi);

        std::ostringstream l1;
        l1 << std::fixed << std::setprecision(1)
           << "FPS: " << stats.smoothFps
           << "  Latency: " << stats.totalMs << " ms"
           << "  Frame: " << stats.frameNum;
        cv::putText(canvas, l1.str(), cv::Point(10, 22),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(200, 220, 255), 1, cv::LINE_AA);

        std::ostringstream l2;
        l2 << "YOLO: " << std::setprecision(1) << stats.yoloMs << " ms";
        if (stats.ranMidas)
            l2 << "  MiDaS: " << stats.midasMs << " ms";
        l2 << "  Stride: " << stats.stride;
        cv::putText(canvas, l2.str(), cv::Point(10, 44),
                    cv::FONT_HERSHEY_SIMPLEX, 0.48,
                    cv::Scalar(180, 195, 180), 1, cv::LINE_AA);

        std::ostringstream l3;
        l3 << "Dets: " << stats.totalDetections
           << "  Potholes: " << stats.potholes.size();
        if (!std::isnan(stats.tempC))
            l3 << "  CPU: " << std::setprecision(1) << stats.tempC << " C";
        if (stats.rssMb > 0.0f)
            l3 << "  RSS: " << static_cast<int>(stats.rssMb) << " MB";
        if (showDepthHint && !stats.ranMidas)
            l3 << "  [depth pending stride=" << stats.stride << "]";
        cv::putText(canvas, l3.str(), cv::Point(10, 66),
                    cv::FONT_HERSHEY_SIMPLEX, 0.44,
                    cv::Scalar(150, 160, 150), 1, cv::LINE_AA);
    }

    // ── Per-pothole bounding boxes ───────────────────────────────────────
    if (stats.potholes.empty()) return;

    const int numDets = static_cast<int>(stats.potholes.size());

    cv::parallel_for_(cv::Range(0, numDets), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            const auto& p = stats.potholes[static_cast<std::size_t>(i)];
            const cv::Rect& box = p.box;
            if (box.width <= 0 || box.height <= 0) continue;

            const cv::Scalar boxCol = p.hazard
                ? cv::Scalar(40,  40, 235)    // red for hazard
                : cv::Scalar(60, 210, 100);   // green for safe

            // Bounding box
            cv::rectangle(canvas, box, boxCol, 2, cv::LINE_AA);

            // Corner accents
            const int cl = std::min(14, std::min(box.width, box.height) / 3);
            auto corner = [&](cv::Point a, cv::Point b) {
                cv::line(canvas, a, b, boxCol, 3, cv::LINE_AA);
            };
            corner({box.x,           box.y},           {box.x + cl,        box.y});
            corner({box.x,           box.y},           {box.x,              box.y + cl});
            corner({box.br().x,      box.br().y},      {box.br().x - cl,    box.br().y});
            corner({box.br().x,      box.br().y},      {box.br().x,         box.br().y - cl});

            // HAZARD / SAFE badge
            const std::string badge = p.hazard ? "HAZARD" : "SAFE";
            int baseline = 0;
            cv::Size tSz = cv::getTextSize(badge, cv::FONT_HERSHEY_SIMPLEX,
                                           0.52, 2, &baseline);
            int bx = box.x;
            int by = std::max(box.y - tSz.height - 12, 0);
            cv::rectangle(canvas, cv::Rect(bx, by, tSz.width + 10, tSz.height + 8),
                          boxCol, cv::FILLED);
            cv::putText(canvas, badge, cv::Point(bx + 5, by + tSz.height + 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.52,
                        cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

            // Confidence label below badge
            std::ostringstream conf;
            conf << std::fixed << std::setprecision(2)
                 << "Y:" << p.yoloConf
                 << " F:" << p.fusionConf;
            cv::putText(canvas, conf.str(),
                        cv::Point(box.x, box.y + box.height + 16),
                        cv::FONT_HERSHEY_SIMPLEX, 0.44,
                        boxCol, 1, cv::LINE_AA);

            // Fusion confidence bar at box bottom
            const int barH = 5;
            const int barY = box.y + box.height + 24;
            if (barY + barH < canvas.rows) {
                const int filled = static_cast<int>(
                    box.width * std::min(1.0f, p.fusionConf));
                cv::rectangle(canvas,
                              cv::Rect(box.x, barY, box.width, barH),
                              cv::Scalar(35, 35, 35), cv::FILLED);
                cv::rectangle(canvas,
                              cv::Rect(box.x, barY, filled, barH),
                              boxCol, cv::FILLED);
            }

            // Compact depth/temporal info panel (right of box, if space)
            const int panelX = box.x + box.width + 6;
            const int panelW = 210;
            const int panelH = 110;
            if (panelX + panelW <= canvas.cols && box.y + panelH <= canvas.rows) {
                cv::Mat pr = canvas(cv::Rect(panelX, box.y, panelW, panelH));
                cv::multiply(pr, cv::Scalar(0.28, 0.28, 0.28), pr);

                auto put = [&](const std::string& txt, int dy, const cv::Scalar& col) {
                    cv::putText(canvas, txt,
                                cv::Point(panelX + 6, box.y + dy),
                                cv::FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv::LINE_AA);
                };
                put("Depress: " + [&]{ std::ostringstream o; o << std::fixed << std::setprecision(3) << p.depression; return o.str(); }(),
                    18,  cv::Scalar(100, 200, 255));
                put("Roughns: " + [&]{ std::ostringstream o; o << std::fixed << std::setprecision(3) << p.roughness; return o.str(); }(),
                    34,  cv::Scalar(100, 200, 255));
                put("Persist: " + [&]{ std::ostringstream o; o << std::fixed << std::setprecision(3) << p.persistence; return o.str(); }(),
                    50,  cv::Scalar(180, 160, 255));
                put("Stabili: " + [&]{ std::ostringstream o; o << std::fixed << std::setprecision(3) << p.stability; return o.str(); }(),
                    66,  cv::Scalar(180, 160, 255));
                put("GeomCnf: " + [&]{ std::ostringstream o; o << std::fixed << std::setprecision(3) << p.geoConf; return o.str(); }(),
                    82,  cv::Scalar(80, 200, 180));
                put("Fusion:  " + [&]{ std::ostringstream o; o << std::fixed << std::setprecision(3) << p.fusionConf; return o.str(); }(),
                    98,  boxCol);
            }
        }
    });

    // ── Hazard alert banner (bottom of frame) ───────────────────────────
    // Drawn as a full-width coloured strip so it's visible at a glance
    // without needing to read the per-box labels.
    {
        bool anyHazard = false;
        for (const auto& p : stats.potholes)
            if (p.hazard) { anyHazard = true; break; }

        if (anyHazard) {
            const int bannerH = 34;
            const int bannerY = canvas.rows - bannerH;
            cv::Mat banner = canvas(cv::Rect(0, bannerY, canvas.cols, bannerH));
            cv::multiply(banner, cv::Scalar(0.15, 0.15, 0.55), banner);  // red tint
            cv::putText(canvas,
                        "⚠  HAZARD DETECTED — POTHOLE CONFIRMED",
                        cv::Point(canvas.cols / 2 - 240, bannerY + 23),
                        cv::FONT_HERSHEY_DUPLEX, 0.70,
                        cv::Scalar(80, 80, 255), 2, cv::LINE_AA);
        } else if (!stats.potholes.empty()) {
            // Potholes detected but below hazard threshold
            const int bannerH = 28;
            const int bannerY = canvas.rows - bannerH;
            cv::Mat banner = canvas(cv::Rect(0, bannerY, canvas.cols, bannerH));
            cv::multiply(banner, cv::Scalar(0.15, 0.45, 0.15), banner);  // green tint
            cv::putText(canvas,
                        "✓  Pothole detected — below hazard threshold",
                        cv::Point(canvas.cols / 2 - 220, bannerY + 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        cv::Scalar(80, 220, 80), 1, cv::LINE_AA);
        }
    }

    // ── Keyboard hint strip (bottom-right) ───────────────────────────────
    cv::putText(canvas, "[Q]Quit [Spc]Pause [S]Step [D]Depth [F]Fullscr",
                cv::Point(canvas.cols - 350, canvas.rows - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.38,
                cv::Scalar(100, 100, 120), 1, cv::LINE_AA);
}

// ─── Depth map compositor (tinted side-by-side blend) ───────────────────────

static cv::Mat blendDepthOverlay(const cv::Mat& frame, const cv::Mat& depthMap) {
    if (depthMap.empty()) return frame;

    // Normalise and colour-map — KleidiCV HAL accelerates both ops on A72
    cv::Mat norm;
    cv::normalize(depthMap, norm, 0.0, 255.0, cv::NORM_MINMAX);
    norm.convertTo(norm, CV_8U);
    cv::Mat coloured;
    cv::applyColorMap(norm, coloured, cv::COLORMAP_INFERNO);

    // Resize depth to match frame (MiDaS output is 256x256)
    if (coloured.size() != frame.size())
        cv::resize(coloured, coloured, frame.size());

    // Semi-transparent blend: 65% original, 35% depth
    cv::Mat out;
    cv::addWeighted(frame, 0.65, coloured, 0.35, 0.0, out);
    return out;
}

// ─── OpenVINO device capability log ─────────────────────────────────────────

static void logDeviceCapabilities(ov::Core& core, const std::string& device) {
    std::cout << "[seq-test] OpenVINO device: " << device << '\n';
    try {
        std::cout << "[seq-test]   Full name  : "
                  << core.get_property(device, ov::device::full_name) << '\n';
    } catch (...) {}
    try {
        const auto caps = core.get_property(device, ov::device::capabilities);
        std::cout << "[seq-test]   Capabilities:";
        for (const auto& c : caps) std::cout << ' ' << c;
        std::cout << '\n';
    } catch (...) {}
#if (defined(__aarch64__) || defined(__ARM_NEON)) && !defined(__APPLE__)
    std::cout << "[seq-test]   ARM backend : ACTIVE (KleidiAI + NEON)\n";
#endif
}

} // namespace seq
} // namespace vigia

// ═══════════════════════════════════════════════════════════════════════════
//  SIGBUS handler (identical to system_visual_test)
// ═══════════════════════════════════════════════════════════════════════════

static volatile const char* g_currentStep = "(not started)";

static void sigbusHandler(int sig) {
#if defined(__linux__) || defined(__APPLE__)
    const char prefix[] = "\n[FATAL] Caught SIGBUS during step: ";
    const char suffix[] = "\n[FATAL] Rebuild OpenVINO with -DENABLE_KLEIDIAI=ON\n";
    (void)::write(STDERR_FILENO, prefix,  sizeof(prefix)  - 1);
    const volatile char* p = g_currentStep;
    std::size_t len = 0;
    while (p[len]) ++len;
    (void)::write(STDERR_FILENO, (const char*)p, len);
    (void)::write(STDERR_FILENO, suffix,  sizeof(suffix)  - 1);
    ::_exit(128 + sig);
#else
    // Non-POSIX fallback — just abort.
    (void)sig;
    std::abort();
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
//  main()
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    using namespace vigia;
    using namespace vigia::seq;

    // ── Argument parsing ────────────────────────────────────────────────
    if (argc < 2) {
        std::cerr << "Usage: system_sequential_test [-F] "
                     "(--video <path> | --cam [idx]) "
                     "[yolo_xml] [midas_xml] [target_fps]\n";
        return 1;
    }

    bool startFullscreen = false;
    bool useCamera       = false;
    int  cameraIndex     = 0;
    std::string videoPath;

    int argIndex = 1;

    if (std::string(argv[argIndex]) == "-F") {
        startFullscreen = true;
        ++argIndex;
    }

    if (argIndex >= argc) {
        std::cerr << "[seq-test] Missing --video / --cam argument\n";
        return 1;
    }

    const std::string modeArg = argv[argIndex++];

    if (modeArg == "--cam") {
        useCamera = true;
        if (argIndex < argc && argv[argIndex][0] != '-') {
            try { cameraIndex = std::stoi(argv[argIndex++]); }
            catch (...) { std::cerr << "[seq-test] Camera index must be integer\n"; return 1; }
        }
    } else if (modeArg == "--video") {
        if (argIndex >= argc) { std::cerr << "[seq-test] --video requires a path\n"; return 1; }
        videoPath = argv[argIndex++];
    } else {
        std::cerr << "[seq-test] Unknown mode: " << modeArg << "\n";
        return 1;
    }

    const std::string yoloModel = (argIndex < argc)
        ? argv[argIndex++]
        : "models/yolo26/yolo26_model_int8.xml";

    const std::string midasModel = (argIndex < argc)
        ? argv[argIndex++]
        : "models/midasv21/openvino_midas_v21_small_256.xml";

    int targetFps = 15;
    if (argIndex < argc) {
        try { targetFps = std::max(1, std::stoi(argv[argIndex++])); }
        catch (...) { std::cerr << "[seq-test] target_fps must be integer\n"; return 1; }
    }
    const long targetFrameTimeMs = 1000L / targetFps;

    const std::string sourceLabel = useCamera
        ? ("camera:" + std::to_string(cameraIndex))
        : videoPath;

    // ── SIGBUS handler ──────────────────────────────────────────────────
    std::signal(SIGBUS, sigbusHandler);

    // ── OpenCV thread config ────────────────────────────────────────────
    // cv::setNumThreads(0) = let TBB use all cores for internal HAL dispatch
    cv::setNumThreads(0);
    cv::ocl::setUseOpenCL(false);

    // Pin the sequential loop to Core 1 (performance core on Pi 4).
    // Core 0 = OS / kernel tasks, Core 1 = our pipeline, Core 2-3 = TBB.
    // Using Core 1 keeps us off the OS-busy core while TBB gets 2-3.
    pinCurrentThreadToCore(1);

#ifdef __linux__
    // Verify 'performance' CPU governor
    {
        std::ifstream gov("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
        if (gov.is_open()) {
            std::string g;
            std::getline(gov, g);
            while (!g.empty() && (g.back() == '\n' || g.back() == ' ')) g.pop_back();
            if (g == "performance")
                std::cout << "[seq-test] CPU governor : performance (optimal)\n";
            else
                std::cerr << "[seq-test] WARNING: CPU governor is '" << g
                          << "' — run: echo performance | sudo tee "
                             "/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor\n";
        }
    }
#endif

    try {
        // ── Shared ov::Core (heap-allocated for ARM64 alignment safety) ─
        g_currentStep = "ov::Core construction";
        std::cout << "[seq-test] Constructing ov::Core...\n" << std::flush;
        auto corePtr = std::make_shared<ov::Core>();
        ov::Core& core = *corePtr;
        const std::string device = "CPU";

        // FP32 precision guard — prevents FP16 SIGBUS on Cortex-A72
        g_currentStep = "Setting FP32 inference precision";
        try {
            core.set_property("CPU", ov::hint::inference_precision(ov::element::f32));
            std::cout << "[seq-test] FP32 inference precision set\n";
        } catch (const std::exception& e) {
            std::cerr << "[seq-test] WARNING: " << e.what() << '\n';
        }

        // Disable mmap — prevents SIGBUS from SD-card page faults
        g_currentStep = "Disabling mmap";
        try {
            core.set_property("CPU", ov::enable_mmap(false));
            std::cout << "[seq-test] mmap disabled\n";
        } catch (const std::exception& e) {
            std::cerr << "[seq-test] WARNING: " << e.what() << '\n';
        }

        logDeviceCapabilities(core, device);

        // ── Load YOLO model ──────────────────────────────────────────────
        g_currentStep = "PerceptionAgent construction";
        std::cout << "[seq-test] Loading YOLO: " << yoloModel << " ...\n" << std::flush;
        PerceptionAgent perception(core, yoloModel, device);
        if (!perception.isModelLoaded()) {
            std::cerr << "[seq-test] FATAL: YOLO model failed to compile\n";
            return 1;
        }
        std::cout << "[seq-test] YOLO compiled OK\n";

        // ── Load MiDaS model ─────────────────────────────────────────────
        g_currentStep = "AnalyticalAgent construction";
        std::cout << "[seq-test] Loading MiDaS: " << midasModel << " ...\n" << std::flush;
        AnalyticalAgent analytical(core, midasModel, device);
        if (!analytical.isModelLoaded()) {
            std::cerr << "[seq-test] FATAL: MiDaS model failed to compile\n";
            return 1;
        }
        std::cout << "[seq-test] MiDaS compiled OK\n";

        // ── Temporal + Fusion (stack — no heap overhead) ─────────────────
        TemporalAnalyzer temporal;
        FusionEngine     fusion;

        // ── Video capture ─────────────────────────────────────────────────
        g_currentStep = "VideoCapture open";
        cv::VideoCapture cap;

        if (useCamera) {
            // GStreamer pipeline for libcamera — matches VideoPerceptionAgent
            const std::string pipeline =
                "libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 "
                "! videoconvert ! video/x-raw, format=BGR ! appsink";
            std::cout << "[seq-test] Opening camera via GStreamer...\n";
            if (!cap.open(pipeline, cv::CAP_GSTREAMER))
                throw std::runtime_error(
                    "Failed to open camera via GStreamer. "
                    "Ensure libcamera-dev and gstreamer plugins are installed.");
        } else {
            if (!cap.open(videoPath))
                throw std::runtime_error("Failed to open video: " + videoPath);
        }

        const double videoFps = [&] {
            double f = cap.get(cv::CAP_PROP_FPS);
            return (f > 1.0) ? f : 30.0;
        }();

        std::cout << "[seq-test] Source FPS: " << videoFps << "\n";
        std::cout << "[seq-test] Target FPS: " << targetFps
                  << "  (target frame time: " << targetFrameTimeMs << " ms)\n\n";

        // ── Display window ────────────────────────────────────────────────
        cv::namedWindow("VIGIA Sequential", cv::WINDOW_NORMAL);
        if (startFullscreen) {
            cv::setWindowProperty("VIGIA Sequential",
                                  cv::WND_PROP_FULLSCREEN,
                                  cv::WINDOW_FULLSCREEN);
        } else {
            cv::resizeWindow("VIGIA Sequential", DISPLAY_W, DISPLAY_H);
        }

        // ── Loop state ────────────────────────────────────────────────────
        g_currentStep = "Main sequential loop";

        PerfTracker perf;

        int   frameCount   = 0;
        int   midasStride  = 1;
        bool  paused       = false;
        bool  showDepth    = false;
        bool  fullscreen   = startFullscreen;
        bool  shouldQuit   = false;

        cv::Mat lastDepthMap;   // retained between MiDaS frames
        auto lastRenderTs = std::chrono::steady_clock::now();

        // Throttle console prints so they don't add >0.1 ms per frame
        int nextConsolePrint = 1;

        std::cout << "[seq-test] Starting loop. Keys: Q=quit  Spc=pause  S=step  "
                     "D=depth  F=fullscreen\n\n";

        // ══════════════════════════════════════════════════════════════════
        //  SEQUENTIAL MAIN LOOP
        //
        //  All stages run on the same thread in order:
        //    1. Capture
        //    2. YOLO inference
        //    3. MiDaS inference (conditional)
        //    4. Fusion + Temporal
        //    5. Draw overlay
        //    6. imshow + waitKey
        // ══════════════════════════════════════════════════════════════════

        while (!shouldQuit) {

            // ── Pause handling (spin-wait with low CPU burn) ─────────────
            if (paused) {
                const int key = cv::waitKey(40);
                if (key == 'q' || key == 'Q') { shouldQuit = true; break; }
                if (key == ' ')               { paused = false; }
                if (key == 's' || key == 'S') { /* fall through once */ }
                else continue;
            }

            // ── 1. Capture ───────────────────────────────────────────────
            const auto tFrame0 = std::chrono::steady_clock::now();

            cv::Mat rawFrame;
            if (!cap.read(rawFrame) || rawFrame.empty()) {
                if (!useCamera) {
                    // End of video file — loop back
                    cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                    if (!cap.read(rawFrame) || rawFrame.empty()) break;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
            }
            ++frameCount;

            // ── 2. YOLO inference ────────────────────────────────────────
            const auto tY0 = std::chrono::steady_clock::now();
            const auto detections = perception.runInference(rawFrame);
            const auto tY1 = std::chrono::steady_clock::now();
            const double yoloMs =
                std::chrono::duration<double, std::milli>(tY1 - tY0).count();

            // ── 3. MiDaS inference (adaptive stride) ─────────────────────
            const bool runMidas = (frameCount % midasStride == 0);
            double midasMs = 0.0;

            if (runMidas) {
                const auto tM0 = std::chrono::steady_clock::now();
                lastDepthMap = analytical.runInference(rawFrame);
                const auto tM1 = std::chrono::steady_clock::now();
                midasMs = std::chrono::duration<double, std::milli>(tM1 - tM0).count();
            }

            // ── 4. Fusion + Temporal per pothole detection ───────────────
            const auto tF0 = std::chrono::steady_clock::now();

            FrameStats stats;
            stats.frameNum        = frameCount;
            stats.yoloMs          = yoloMs;
            stats.midasMs         = midasMs;
            stats.ranMidas        = runMidas;
            stats.stride          = midasStride;
            stats.totalDetections = static_cast<int>(detections.size());

            // Read system telemetry once per frame (cheap, <0.1 ms)
            stats.tempC  = readCPUTemperature();
            stats.rssMb  = readRssMb();

            for (const auto& det : detections) {
                if (det.classId != POTHOLE_CLASS_ID) continue;

                PotholeTelemetry tele;
                tele.box      = det.boundingBox;
                tele.yoloConf = det.confidence;

                // Only run geometry / temporal / fusion when we have fresh depth
                if (!lastDepthMap.empty()) {
                    cv::Rect roi = analytical.scaleROIToDepth(
                        det.boundingBox, rawFrame.size(), lastDepthMap.size());
                    cv::Mat roiDepth = analytical.extractDepthROI(lastDepthMap, roi);

                    if (!roiDepth.empty()) {
                        const auto residuals = analytical.computeDepthResiduals(roiDepth);
                        const auto geom      = analytical.computeGeometryMetrics(roiDepth, residuals);
                        const auto tMetrics  = temporal.update(geom.depressionScore, geom.roughness);

                        FusionInput fin{};
                        fin.yoloConfidence  = det.confidence;
                        fin.depressionScore = geom.depressionScore;
                        fin.roughness       = geom.roughness;
                        fin.persistence     = tMetrics.persistence;
                        fin.stability       = tMetrics.stability;

                        const FusionOutput fout = fusion.fuse(fin);

                        tele.fusionConf  = fout.finalConfidence;
                        tele.geoConf     = fout.geometryConfidence;
                        tele.depression  = geom.depressionScore;
                        tele.roughness   = geom.roughness;
                        tele.persistence = tMetrics.persistence;
                        tele.stability   = tMetrics.stability;
                    } else {
                        // No valid depth ROI — fall back to raw YOLO confidence
                        tele.fusionConf = det.confidence;
                    }
                } else {
                    // No depth data yet — raw YOLO confidence as fallback
                    tele.fusionConf = det.confidence;
                }

                tele.hazard = (tele.fusionConf >= HAZARD_THRESHOLD);
                stats.potholes.push_back(tele);
            }

            const auto tF1 = std::chrono::steady_clock::now();
            stats.fusionMs =
                std::chrono::duration<double, std::milli>(tF1 - tF0).count();

            // ── 5. End-to-end latency + PerfTracker ──────────────────────
            const auto tFrame1 = std::chrono::steady_clock::now();
            stats.totalMs =
                std::chrono::duration<double, std::milli>(tFrame1 - tFrame0).count();

            // Skip warmup frames from EMA / stats (mirrors benchmark)
            if (frameCount > WARMUP_FRAMES) {
                perf.recordFrame(stats.totalMs);
                if (perf.frames() == 1) {
                    std::cout << "[seq-test] Time to first result: "
                              << std::fixed << std::setprecision(1)
                              << perf.firstInferMs() << " ms\n";
                }
            }

            stats.smoothFps = perf.smoothedFps();

            // Update adaptive stride for the NEXT frame
            midasStride = adaptiveStride(
                midasStride,
                stats.tempC,
                static_cast<long>(stats.totalMs),
                targetFrameTimeMs);

            // ── 6. Render and display ─────────────────────────────────────
            //
            // Render policy (in priority order):
            //   A) Always render when potholes are present this frame —
            //      ensures detection boxes appear with zero extra latency.
            //   B) Render when the throttle interval has elapsed —
            //      gives a smooth live feed even with no detections.
            //   C) Skip otherwise — avoids burning ~1 ms on clone+imshow
            //      for identical frames between inference updates.
            //
            // This decouples "show detections immediately" from "show
            // background video smoothly", which is the right trade-off
            // on Pi 4 where inference takes 80–100 ms.
            const auto renderNow = std::chrono::steady_clock::now();
            const bool throttleElapsed =
                (renderNow - lastRenderTs) >=
                std::chrono::milliseconds(RENDER_INTERVAL_MS);
            const bool hasPotholes = !stats.potholes.empty();

            if (hasPotholes || throttleElapsed) {
                // Build canvas:
                //   • depth mode  → blendDepthOverlay returns a fresh Mat,
                //                   no extra clone needed.
                //   • normal mode → .clone() gives us a mutable copy of
                //                   rawFrame without touching the capture buf.
                cv::Mat canvas = (showDepth && !lastDepthMap.empty())
                    ? blendDepthOverlay(rawFrame, lastDepthMap)
                    : rawFrame.clone();

                drawOverlay(canvas, stats, showDepth, perf);
                cv::imshow("VIGIA Sequential", canvas);

                lastRenderTs = renderNow;
            }

            // ── Throttle for video files so we don't race through frames ─
            // Never throttle when potholes were detected — we want the
            // next inference to start immediately so boxes stay current.
            if (!useCamera && !hasPotholes) {
                const double interFrameMs = 1000.0 / videoFps;
                if (stats.totalMs < interFrameMs) {
                    std::this_thread::sleep_for(
                        std::chrono::duration<double, std::milli>(
                            interFrameMs - stats.totalMs));
                }
            }

            // ── Console status (throttled to avoid I/O overhead) ─────────
            if (frameCount == nextConsolePrint) {
                std::cout << '\r'
                          << "[F" << std::setw(5) << frameCount << "]"
                          << std::fixed << std::setprecision(1)
                          << " YOLO=" << yoloMs << "ms"
                          << (runMidas ? " MiDaS=" + std::to_string(static_cast<int>(midasMs)) + "ms" : "")
                          << " E2E=" << stats.totalMs << "ms"
                          << " FPS=" << stats.smoothFps
                          << " s=" << midasStride
                          << " T=" << stats.tempC << "C"
                          << " dets=" << detections.size()
                          << " potholes=" << stats.potholes.size()
                          << std::flush;

                // Exponential back-off: print every 1, 2, 4 … 32 frames
                nextConsolePrint = std::min(frameCount * 2, frameCount + 32);
            }

            // ── Key handling ─────────────────────────────────────────────
            const int key = cv::waitKey(1);
            if (key < 0) continue;

            switch (key) {
            case 'q': case 'Q':
                shouldQuit = true;
                break;
            case ' ':
                paused = !paused;
                if (paused) std::cout << "\n[seq-test] PAUSED — press Space to resume, S to step\n";
                break;
            case 's': case 'S':
                // Already executed one frame above; pause after this one
                paused = true;
                break;
            case 'd': case 'D':
                showDepth = !showDepth;
                std::cout << "\n[seq-test] Depth overlay: " << (showDepth ? "ON" : "OFF") << '\n';
                break;
            case 'f': case 'F':
                fullscreen = !fullscreen;
                cv::setWindowProperty("VIGIA Sequential",
                                      cv::WND_PROP_FULLSCREEN,
                                      fullscreen ? cv::WINDOW_FULLSCREEN : cv::WINDOW_NORMAL);
                break;
            default:
                break;
            }
        } // end sequential main loop

        // ── Cleanup ───────────────────────────────────────────────────────
        cap.release();
        cv::destroyAllWindows();

        // ── Final performance summary ─────────────────────────────────────
        std::cout << "\n\n";
        std::cout << "===== Sequential Test Summary =====\n";
        std::cout << "Source         : " << sourceLabel << "\n";
        std::cout << "Frames total   : " << frameCount << "\n";
        std::cout << "Frames metered : " << perf.frames()
                  << " (excl. " << WARMUP_FRAMES << " warmup)\n";
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "FPS (EMA)      : " << perf.smoothedFps() << "\n";
        std::cout << "Latency avg    : " << perf.avgLatencyMs() << " ms\n";
        std::cout << "Latency min    : " << perf.minLatencyMs() << " ms\n";
        std::cout << "Latency max    : " << perf.maxLatencyMs() << " ms\n";
        std::cout << "Time to 1st    : " << perf.firstInferMs() << " ms\n";
        std::cout << "===================================\n";

    } catch (const std::exception& ex) {
        std::cerr << "\n[seq-test] FATAL: " << ex.what() << '\n';
        return 1;
    }

    return 0;
}
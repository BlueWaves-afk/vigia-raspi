/**
 * @file performance_benchmark_test.cpp
 * @brief Comprehensive VIGIA performance benchmark with graph generation
 *
 * Runs the full pipeline (YOLO26 + MiDaS v2.1 + Fusion + Temporal) on a
 * video file and collects:
 *
 *   1. Inference latency (ms) — per-stage breakdown (YOLO vs MiDaS)
 *   2. Throughput (FPS) — stable vs peak, time-series
 *   3. Thermal throttling — temperature, stride, and their correlation
 *   4. Resource utilisation — per-core CPU %, RSS memory footprint
 *
 * All graphs are rendered using OpenCV drawing and saved as PNG images
 * in a "benchmark_results/" directory alongside the binary.
 *
 * Usage:
 *   ./performance_benchmark_test --video hazard.mp4 [yolo.xml] [midas.xml]
 *   ./performance_benchmark_test --cam [index] [yolo.xml] [midas.xml]
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <openvino/openvino.hpp>

#include "perception.hpp"
#include "analytical.hpp"
#include "temporal.hpp"
#include "fusion.hpp"

#ifdef __linux__
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#endif

using namespace vigia;
using Clock = std::chrono::steady_clock;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Constants
 * ═══════════════════════════════════════════════════════════════════════════ */

static constexpr int POTHOLE_CLASS_ID = 0;
static constexpr int WARMUP_FRAMES    = 5;

// Chart rendering constants
static constexpr int CHART_W = 1200;
static constexpr int CHART_H = 600;
static constexpr int MARGIN_L = 90;
static constexpr int MARGIN_R = 40;
static constexpr int MARGIN_T = 60;
static constexpr int MARGIN_B = 80;
static constexpr int PLOT_W = CHART_W - MARGIN_L - MARGIN_R;
static constexpr int PLOT_H = CHART_H - MARGIN_T - MARGIN_B;

// Colors (BGR)
static const cv::Scalar COL_BG       (30,  25,  20);
static const cv::Scalar COL_GRID     (60,  55,  50);
static const cv::Scalar COL_TEXT     (220, 210, 200);
static const cv::Scalar COL_AXIS     (160, 150, 140);
static const cv::Scalar COL_YOLO     (80,  180, 255);   // orange
static const cv::Scalar COL_MIDAS    (255, 140,  60);   // blue
static const cv::Scalar COL_FPS      (80,  255, 120);   // green
static const cv::Scalar COL_PEAK     (80,  80,  255);   // red
static const cv::Scalar COL_TEMP     (60,  60,  255);   // red
static const cv::Scalar COL_STRIDE   (255, 200,  60);   // cyan-ish
static const cv::Scalar COL_CPU0     (80,  180, 255);
static const cv::Scalar COL_CPU1     (80,  255, 120);
static const cv::Scalar COL_CPU2     (255, 140,  60);
static const cv::Scalar COL_CPU3     (200,  80, 255);
static const cv::Scalar COL_MEM      (100, 220, 220);
static const cv::Scalar COL_TITLE    (255, 255, 255);
static const cv::Scalar COL_BAR_YOLO (80,  180, 255);
static const cv::Scalar COL_BAR_MIDAS(255, 140,  60);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Per-frame sample
 * ═══════════════════════════════════════════════════════════════════════════ */

struct FrameSample {
    int    frameNum{0};
    double yoloMs{0.0};
    double midasMs{0.0};        // 0 if MiDaS was skipped
    double totalMs{0.0};        // wall-clock for entire processFrame
    double fps{0.0};            // instantaneous (1/totalMs)
    double smoothFps{0.0};      // EMA-smoothed
    float  tempC{0.0f};         // CPU temperature
    int    stride{1};           // MiDaS stride at this frame
    bool   ranMidas{false};
    int    detections{0};
    int    potholes{0};
    float  maxConf{0.0f};
    // CPU usage per core (0–100%)
    std::array<float, 4> cpuPct{};
    // RSS in MB
    float  rssMb{0.0f};
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  System monitoring helpers (Linux only — stubs on other platforms)
 * ═══════════════════════════════════════════════════════════════════════════ */

struct CpuTimes {
    std::array<long long, 4> busy{};
    std::array<long long, 4> total{};
};

static CpuTimes readCpuTimes() {
    CpuTimes ct{};
#ifdef __linux__
    std::ifstream f("/proc/stat");
    if (!f.is_open()) return ct;
    std::string line;
    // Skip the aggregate "cpu" line
    std::getline(f, line);
    // Read cpu0..cpu3
    for (int i = 0; i < 4; ++i) {
        if (!std::getline(f, line)) break;
        // Format: cpuN user nice system idle iowait irq softirq steal ...
        long long user, nice, sys, idle, iowait, irq, softirq, steal;
        char tag[16];
        if (std::sscanf(line.c_str(), "%s %lld %lld %lld %lld %lld %lld %lld %lld",
                        tag, &user, &nice, &sys, &idle, &iowait, &irq, &softirq, &steal) >= 8) {
            ct.busy[i]  = user + nice + sys + irq + softirq + steal;
            ct.total[i] = ct.busy[i] + idle + iowait;
        }
    }
#endif
    return ct;
}

static std::array<float, 4> computeCpuPct(const CpuTimes& prev, const CpuTimes& curr) {
    std::array<float, 4> pct{};
    for (int i = 0; i < 4; ++i) {
        const long long dt = curr.total[i] - prev.total[i];
        const long long db = curr.busy[i]  - prev.busy[i];
        pct[i] = (dt > 0) ? (static_cast<float>(db) / static_cast<float>(dt) * 100.0f) : 0.0f;
    }
    return pct;
}

static float readTemperature() {
#ifdef __linux__
    std::ifstream f("/sys/class/thermal/thermal_zone0/temp");
    if (!f.is_open()) return 0.0f;
    float milliC = 0.0f;
    f >> milliC;
    return milliC / 1000.0f;
#else
    return 0.0f;
#endif
}

static float readRssMb() {
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

/* ═══════════════════════════════════════════════════════════════════════════
 *  Chart drawing utilities
 * ═══════════════════════════════════════════════════════════════════════════ */

static cv::Mat makeCanvas(const std::string& title) {
    cv::Mat canvas(CHART_H, CHART_W, CV_8UC3, COL_BG);
    // Title
    cv::putText(canvas, title, cv::Point(MARGIN_L, 35),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, COL_TITLE, 2);
    // Plot border
    cv::rectangle(canvas,
                  cv::Point(MARGIN_L, MARGIN_T),
                  cv::Point(MARGIN_L + PLOT_W, MARGIN_T + PLOT_H),
                  COL_AXIS, 1);
    return canvas;
}

static void drawGridH(cv::Mat& canvas, int numLines, double minVal, double maxVal,
                       const std::string& unit, const cv::Scalar& col = COL_GRID) {
    for (int i = 0; i <= numLines; ++i) {
        const int y = MARGIN_T + PLOT_H - i * PLOT_H / numLines;
        cv::line(canvas, cv::Point(MARGIN_L, y), cv::Point(MARGIN_L + PLOT_W, y), col, 1);
        const double val = minVal + i * (maxVal - minVal) / numLines;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << val << unit;
        cv::putText(canvas, oss.str(), cv::Point(5, y + 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, COL_TEXT, 1);
    }
}

static void drawXLabel(cv::Mat& canvas, const std::string& label) {
    cv::putText(canvas, label,
                cv::Point(MARGIN_L + PLOT_W / 2 - 40, CHART_H - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, COL_TEXT, 1);
}

static void drawLegendItem(cv::Mat& canvas, int x, int y,
                            const cv::Scalar& col, const std::string& text) {
    cv::rectangle(canvas, cv::Point(x, y - 8), cv::Point(x + 16, y + 2), col, cv::FILLED);
    cv::putText(canvas, text, cv::Point(x + 22, y + 2),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, COL_TEXT, 1);
}

/// Map a value in [minV, maxV] to a Y pixel in the plot area.
static int toY(double val, double minV, double maxV) {
    if (maxV <= minV) return MARGIN_T + PLOT_H;
    const double frac = (val - minV) / (maxV - minV);
    return MARGIN_T + PLOT_H - static_cast<int>(frac * PLOT_H);
}

/// Map a frame index in [0, count) to an X pixel in the plot area.
static int toX(int idx, int count) {
    if (count <= 1) return MARGIN_L;
    return MARGIN_L + idx * PLOT_W / (count - 1);
}

/// Draw a time-series polyline.
static void drawSeries(cv::Mat& canvas, const std::vector<double>& data,
                        double minV, double maxV, const cv::Scalar& col,
                        int thickness = 1) {
    const int n = static_cast<int>(data.size());
    if (n < 2) return;
    for (int i = 1; i < n; ++i) {
        cv::line(canvas,
                 cv::Point(toX(i - 1, n), toY(data[i - 1], minV, maxV)),
                 cv::Point(toX(i, n),     toY(data[i],     minV, maxV)),
                 col, thickness, cv::LINE_AA);
    }
}

/// Draw a stepped line (for integer data like stride).
static void drawSteppedSeries(cv::Mat& canvas, const std::vector<int>& data,
                               int minV, int maxV, const cv::Scalar& col,
                               int thickness = 2) {
    const int n = static_cast<int>(data.size());
    if (n < 2) return;
    for (int i = 1; i < n; ++i) {
        const int x0 = toX(i - 1, n);
        const int x1 = toX(i, n);
        const int y0 = toY(data[i - 1], minV, maxV);
        const int y1 = toY(data[i], minV, maxV);
        // Horizontal then vertical (step function)
        cv::line(canvas, cv::Point(x0, y0), cv::Point(x1, y0), col, thickness);
        cv::line(canvas, cv::Point(x1, y0), cv::Point(x1, y1), col, thickness);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Graph generators
 * ═══════════════════════════════════════════════════════════════════════════ */

static void generateLatencyChart(const std::vector<FrameSample>& samples,
                                  const std::string& outDir) {
    // Gather YOLO and MiDaS latencies
    std::vector<double> yoloLat, midasLat;
    for (const auto& s : samples) {
        yoloLat.push_back(s.yoloMs);
        if (s.ranMidas) midasLat.push_back(s.midasMs);
    }

    // ── Time-series chart ───────────────────────────────────────────
    auto canvas = makeCanvas("Inference Latency per Frame (ms)");

    double maxLat = 1.0;
    for (const auto& s : samples)
        maxLat = std::max(maxLat, std::max(s.yoloMs, s.ranMidas ? s.midasMs : 0.0));
    maxLat = std::ceil(maxLat / 50.0) * 50.0;   // round up to nearest 50

    drawGridH(canvas, 8, 0.0, maxLat, " ms");
    drawXLabel(canvas, "Frame Number");

    // Draw YOLO series
    {
        std::vector<double> series;
        for (const auto& s : samples) series.push_back(s.yoloMs);
        drawSeries(canvas, series, 0.0, maxLat, COL_YOLO, 1);
    }

    // Draw MiDaS points (only when it ran)
    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (samples[i].ranMidas) {
            const int x = toX(static_cast<int>(i), static_cast<int>(samples.size()));
            const int y = toY(samples[i].midasMs, 0.0, maxLat);
            cv::circle(canvas, cv::Point(x, y), 3, COL_MIDAS, cv::FILLED);
        }
    }

    drawLegendItem(canvas, MARGIN_L + PLOT_W - 250, MARGIN_T + 20, COL_YOLO,  "YOLO26");
    drawLegendItem(canvas, MARGIN_L + PLOT_W - 120, MARGIN_T + 20, COL_MIDAS, "MiDaS v2.1");

    cv::imwrite(outDir + "/1_latency_timeseries.png", canvas);

    // ── Bar chart: Average latency comparison ───────────────────────
    auto barCanvas = makeCanvas("Average Inference Latency Breakdown (ms)");

    double yoloAvg = yoloLat.empty() ? 0.0 :
        std::accumulate(yoloLat.begin(), yoloLat.end(), 0.0) / yoloLat.size();
    double midasAvg = midasLat.empty() ? 0.0 :
        std::accumulate(midasLat.begin(), midasLat.end(), 0.0) / midasLat.size();

    // Sorted latencies for percentiles
    auto yoloSorted = yoloLat;
    auto midasSorted = midasLat;
    std::sort(yoloSorted.begin(), yoloSorted.end());
    std::sort(midasSorted.begin(), midasSorted.end());

    double yoloP50 = yoloSorted.empty() ? 0 : yoloSorted[yoloSorted.size() / 2];
    double yoloP95 = yoloSorted.empty() ? 0 : yoloSorted[std::min(yoloSorted.size() - 1, (std::size_t)(yoloSorted.size() * 0.95))];
    double yoloP99 = yoloSorted.empty() ? 0 : yoloSorted[std::min(yoloSorted.size() - 1, (std::size_t)(yoloSorted.size() * 0.99))];
    double midasP50 = midasSorted.empty() ? 0 : midasSorted[midasSorted.size() / 2];
    double midasP95 = midasSorted.empty() ? 0 : midasSorted[std::min(midasSorted.size() - 1, (std::size_t)(midasSorted.size() * 0.95))];
    double midasP99 = midasSorted.empty() ? 0 : midasSorted[std::min(midasSorted.size() - 1, (std::size_t)(midasSorted.size() * 0.99))];

    const double barMax = std::max({yoloP99, midasP99, yoloAvg, midasAvg}) * 1.2;
    drawGridH(barCanvas, 6, 0.0, barMax, " ms");

    // Draw bars: Avg | P50 | P95 | P99 for each model
    const std::array<std::pair<std::string, std::array<double, 2>>, 4> barGroups = {{
        {"Avg",  {yoloAvg,  midasAvg}},
        {"P50",  {yoloP50,  midasP50}},
        {"P95",  {yoloP95,  midasP95}},
        {"P99",  {yoloP99,  midasP99}},
    }};

    const int groupW = PLOT_W / 5;
    for (int g = 0; g < 4; ++g) {
        const int cx = MARGIN_L + groupW * (g + 1);
        const int barW = 40;
        // YOLO bar
        const int yY = toY(barGroups[g].second[0], 0.0, barMax);
        cv::rectangle(barCanvas,
                      cv::Point(cx - barW - 2, yY),
                      cv::Point(cx - 2, MARGIN_T + PLOT_H),
                      COL_BAR_YOLO, cv::FILLED);
        // MiDaS bar
        const int mY = toY(barGroups[g].second[1], 0.0, barMax);
        cv::rectangle(barCanvas,
                      cv::Point(cx + 2, mY),
                      cv::Point(cx + barW + 2, MARGIN_T + PLOT_H),
                      COL_BAR_MIDAS, cv::FILLED);
        // Value labels
        {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << barGroups[g].second[0];
            cv::putText(barCanvas, oss.str(), cv::Point(cx - barW - 2, yY - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35, COL_YOLO, 1);
        }
        {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << barGroups[g].second[1];
            cv::putText(barCanvas, oss.str(), cv::Point(cx + 2, mY - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35, COL_MIDAS, 1);
        }
        // Group label
        cv::putText(barCanvas, barGroups[g].first,
                    cv::Point(cx - 15, MARGIN_T + PLOT_H + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, COL_TEXT, 1);
    }

    drawLegendItem(barCanvas, MARGIN_L + PLOT_W - 250, MARGIN_T + 20, COL_BAR_YOLO,  "YOLO26");
    drawLegendItem(barCanvas, MARGIN_L + PLOT_W - 120, MARGIN_T + 20, COL_BAR_MIDAS, "MiDaS v2.1");

    cv::imwrite(outDir + "/2_latency_breakdown.png", barCanvas);
    std::cout << "  [SAVED] 1_latency_timeseries.png\n";
    std::cout << "  [SAVED] 2_latency_breakdown.png\n";
}

static void generateFpsChart(const std::vector<FrameSample>& samples,
                              const std::string& outDir) {
    auto canvas = makeCanvas("Throughput: FPS over Time (Stable vs Peak)");

    std::vector<double> fps, smooth;
    double peakFps = 0.0;
    for (const auto& s : samples) {
        fps.push_back(s.fps);
        smooth.push_back(s.smoothFps);
        peakFps = std::max(peakFps, s.fps);
    }

    const double maxFps = std::ceil(peakFps / 5.0) * 5.0 + 5.0;

    drawGridH(canvas, 8, 0.0, maxFps, " fps");
    drawXLabel(canvas, "Frame Number");

    // Draw instantaneous FPS (faint)
    drawSeries(canvas, fps, 0.0, maxFps, cv::Scalar(60, 100, 50), 1);
    // Draw smooth FPS (bold)
    drawSeries(canvas, smooth, 0.0, maxFps, COL_FPS, 2);

    // Draw peak line
    {
        const int peakY = toY(peakFps, 0.0, maxFps);
        cv::line(canvas,
                 cv::Point(MARGIN_L, peakY),
                 cv::Point(MARGIN_L + PLOT_W, peakY),
                 COL_PEAK, 1, cv::LINE_AA);
        std::ostringstream oss;
        oss << "Peak: " << std::fixed << std::setprecision(1) << peakFps << " fps";
        cv::putText(canvas, oss.str(), cv::Point(MARGIN_L + 10, peakY - 6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, COL_PEAK, 1);
    }

    // Draw stable average line
    double avgSmooth = std::accumulate(smooth.begin(), smooth.end(), 0.0) / smooth.size();
    {
        const int avgY = toY(avgSmooth, 0.0, maxFps);
        cv::line(canvas,
                 cv::Point(MARGIN_L, avgY),
                 cv::Point(MARGIN_L + PLOT_W, avgY),
                 COL_FPS, 1, cv::LINE_4);
        std::ostringstream oss;
        oss << "Stable avg: " << std::fixed << std::setprecision(2) << avgSmooth << " fps";
        cv::putText(canvas, oss.str(), cv::Point(MARGIN_L + 10, avgY + 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, COL_FPS, 1);
    }

    drawLegendItem(canvas, MARGIN_L + PLOT_W - 300, MARGIN_T + 20, cv::Scalar(60,100,50), "Instantaneous");
    drawLegendItem(canvas, MARGIN_L + PLOT_W - 160, MARGIN_T + 20, COL_FPS,  "EMA Smoothed");

    cv::imwrite(outDir + "/3_fps_throughput.png", canvas);
    std::cout << "  [SAVED] 3_fps_throughput.png\n";
}

static void generateThermalChart(const std::vector<FrameSample>& samples,
                                  const std::string& outDir) {
    auto canvas = makeCanvas("Thermal Behavior: Temperature + Adaptive Stride");

    // Dual Y-axis: temperature (left) and stride (right)
    float minTemp = 100.0f, maxTemp = 0.0f;
    int maxStride = 1;
    for (const auto& s : samples) {
        if (s.tempC > 0.0f) {
            minTemp = std::min(minTemp, s.tempC);
            maxTemp = std::max(maxTemp, s.tempC);
        }
        maxStride = std::max(maxStride, s.stride);
    }

    // Expand temp range for readability
    minTemp = std::max(0.0f, std::floor(minTemp / 5.0f) * 5.0f - 5.0f);
    maxTemp = std::ceil(maxTemp / 5.0f) * 5.0f + 5.0f;
    if (maxTemp <= minTemp) { minTemp = 30.0f; maxTemp = 90.0f; }
    maxStride = std::max(maxStride, 5);

    drawGridH(canvas, 8, static_cast<double>(minTemp), static_cast<double>(maxTemp), " C");
    drawXLabel(canvas, "Frame Number");

    // Draw 75°C and 85°C threshold lines
    {
        auto drawThreshold = [&](float thresh, const std::string& label) {
            if (thresh >= minTemp && thresh <= maxTemp) {
                const int y = toY(thresh, minTemp, maxTemp);
                cv::line(canvas, cv::Point(MARGIN_L, y), cv::Point(MARGIN_L + PLOT_W, y),
                         cv::Scalar(40, 40, 180), 1, cv::LINE_AA);
                cv::putText(canvas, label, cv::Point(MARGIN_L + PLOT_W - 120, y - 4),
                            cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(40, 40, 180), 1);
            }
        };
        drawThreshold(75.0f, "WARN 75C");
        drawThreshold(85.0f, "CRIT 85C");
    }

    // Temperature series
    {
        std::vector<double> temps;
        for (const auto& s : samples) temps.push_back(s.tempC);
        drawSeries(canvas, temps, minTemp, maxTemp, COL_TEMP, 2);
    }

    // Stride series (right Y axis — map stride 1–5 to plot area)
    {
        std::vector<int> strides;
        for (const auto& s : samples) strides.push_back(s.stride);
        drawSteppedSeries(canvas, strides, 0, maxStride, COL_STRIDE, 2);
    }

    // Right Y-axis labels for stride
    for (int i = 0; i <= maxStride; ++i) {
        const int y = MARGIN_T + PLOT_H - i * PLOT_H / maxStride;
        std::ostringstream oss;
        oss << "s=" << i;
        cv::putText(canvas, oss.str(),
                    cv::Point(MARGIN_L + PLOT_W + 5, y + 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, COL_STRIDE, 1);
    }

    drawLegendItem(canvas, MARGIN_L + 10, MARGIN_T + 20, COL_TEMP,   "CPU Temp (C)");
    drawLegendItem(canvas, MARGIN_L + 170, MARGIN_T + 20, COL_STRIDE, "MiDaS Stride");

    cv::imwrite(outDir + "/4_thermal_stride.png", canvas);
    std::cout << "  [SAVED] 4_thermal_stride.png\n";
}

static void generateCpuChart(const std::vector<FrameSample>& samples,
                              const std::string& outDir) {
    auto canvas = makeCanvas("CPU Utilization per Core (%)");

    drawGridH(canvas, 5, 0.0, 100.0, " %");
    drawXLabel(canvas, "Frame Number");

    const std::array<cv::Scalar, 4> cpuCols = {COL_CPU0, COL_CPU1, COL_CPU2, COL_CPU3};
    const std::array<std::string, 4> cpuLabels = {
        "Core 0 (Capture)", "Core 1 (Process)", "Core 2 (TBB)", "Core 3 (UI)"
    };

    for (int c = 0; c < 4; ++c) {
        std::vector<double> series;
        for (const auto& s : samples) series.push_back(s.cpuPct[c]);
        drawSeries(canvas, series, 0.0, 100.0, cpuCols[c], 1);
        drawLegendItem(canvas, MARGIN_L + 10 + c * 200, MARGIN_T + 20, cpuCols[c], cpuLabels[c]);
    }

    cv::imwrite(outDir + "/5_cpu_utilization.png", canvas);
    std::cout << "  [SAVED] 5_cpu_utilization.png\n";
}

static void generateMemoryChart(const std::vector<FrameSample>& samples,
                                 const std::string& outDir) {
    auto canvas = makeCanvas("Memory Footprint: RSS (MB)");

    float maxMem = 0.0f;
    for (const auto& s : samples) maxMem = std::max(maxMem, s.rssMb);
    const double memCeil = std::ceil(maxMem / 50.0) * 50.0 + 50.0;

    drawGridH(canvas, 6, 0.0, memCeil, " MB");
    drawXLabel(canvas, "Frame Number");

    std::vector<double> mem;
    for (const auto& s : samples) mem.push_back(s.rssMb);
    drawSeries(canvas, mem, 0.0, memCeil, COL_MEM, 2);

    // Average line
    const double avgMem = std::accumulate(mem.begin(), mem.end(), 0.0) / mem.size();
    {
        const int y = toY(avgMem, 0.0, memCeil);
        cv::line(canvas, cv::Point(MARGIN_L, y), cv::Point(MARGIN_L + PLOT_W, y),
                 cv::Scalar(80, 180, 180), 1, cv::LINE_4);
        std::ostringstream oss;
        oss << "Avg: " << std::fixed << std::setprecision(1) << avgMem << " MB";
        cv::putText(canvas, oss.str(), cv::Point(MARGIN_L + 10, y - 6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, COL_MEM, 1);
    }

    drawLegendItem(canvas, MARGIN_L + PLOT_W - 160, MARGIN_T + 20, COL_MEM, "RSS Memory");

    cv::imwrite(outDir + "/6_memory_footprint.png", canvas);
    std::cout << "  [SAVED] 6_memory_footprint.png\n";
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Summary report (text)
 * ═══════════════════════════════════════════════════════════════════════════ */

static void printAndSaveReport(const std::vector<FrameSample>& samples,
                                double totalSec, const std::string& source,
                                const std::string& yoloModel,
                                const std::string& midasModel,
                                int frameW, int frameH,
                                const std::string& outDir) {
    // Collect latencies (skip warmup)
    std::vector<double> yoloLat, midasLat, totalLat;
    int midasRuns = 0;
    float maxTemp = 0.0f, minTemp = 999.0f;
    float maxRss = 0.0f;
    int maxStride = 1;

    const int start = std::min(WARMUP_FRAMES, static_cast<int>(samples.size()));
    for (int i = start; i < static_cast<int>(samples.size()); ++i) {
        const auto& s = samples[i];
        yoloLat.push_back(s.yoloMs);
        totalLat.push_back(s.totalMs);
        if (s.ranMidas) { midasLat.push_back(s.midasMs); midasRuns++; }
        if (s.tempC > 0.0f) { maxTemp = std::max(maxTemp, s.tempC); minTemp = std::min(minTemp, s.tempC); }
        maxRss = std::max(maxRss, s.rssMb);
        maxStride = std::max(maxStride, s.stride);
    }

    auto percentile = [](std::vector<double> v, double p) -> double {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        const std::size_t idx = std::min(v.size() - 1, static_cast<std::size_t>(v.size() * p));
        return v[idx];
    };

    auto avg = [](const std::vector<double>& v) -> double {
        return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    };

    auto stddev = [&avg](const std::vector<double>& v) -> double {
        if (v.size() < 2) return 0.0;
        const double m = avg(v);
        return std::sqrt(std::accumulate(v.begin(), v.end(), 0.0,
            [m](double acc, double x) { return acc + (x - m) * (x - m); }) / v.size());
    };

    const double stableFps = samples.empty() ? 0.0 : samples.back().smoothFps;
    double peakFps = 0.0;
    for (const auto& s : samples) peakFps = std::max(peakFps, s.fps);

    // Average CPU per core
    std::array<double, 4> avgCpu{};
    for (int c = 0; c < 4; ++c) {
        double sum = 0.0;
        for (int i = start; i < static_cast<int>(samples.size()); ++i)
            sum += samples[i].cpuPct[c];
        avgCpu[c] = (samples.size() - start > 0) ? sum / (samples.size() - start) : 0.0;
    }

    const int N = static_cast<int>(samples.size());

    std::ostringstream rpt;
    rpt << "\n"
        << "==================================================================\n"
        << "          VIGIA PERFORMANCE BENCHMARK REPORT\n"
        << "==================================================================\n"
        << "\n"
        << "  Source       : " << source << "\n"
        << "  Resolution   : " << frameW << "x" << frameH << "\n"
        << "  YOLO model   : " << yoloModel << "\n"
        << "  MiDaS model  : " << midasModel << "\n"
        << "  Total frames : " << N << " (" << WARMUP_FRAMES << " warmup excluded from stats)\n"
        << "  Total time   : " << std::fixed << std::setprecision(2) << totalSec << " s\n"
        << "\n"
        << "──────────────────────────────────────────────────────────────────\n"
        << "  1. INFERENCE LATENCY (ms)\n"
        << "──────────────────────────────────────────────────────────────────\n"
        << "\n"
        << "  ┌──────────────┬──────────┬──────────┬──────────┬──────────┬──────────┐\n"
        << "  │  Stage       │   Avg    │   P50    │   P95    │   P99    │  StdDev  │\n"
        << "  ├──────────────┼──────────┼──────────┼──────────┼──────────┼──────────┤\n"
        << std::fixed << std::setprecision(2)
        << "  │  YOLO26      │ " << std::setw(7) << avg(yoloLat) << " │ " << std::setw(7) << percentile(yoloLat, 0.50) << " │ " << std::setw(7) << percentile(yoloLat, 0.95) << " │ " << std::setw(7) << percentile(yoloLat, 0.99) << " │ " << std::setw(7) << stddev(yoloLat) << " │\n"
        << "  │  MiDaS v2.1  │ " << std::setw(7) << avg(midasLat) << " │ " << std::setw(7) << percentile(midasLat, 0.50) << " │ " << std::setw(7) << percentile(midasLat, 0.95) << " │ " << std::setw(7) << percentile(midasLat, 0.99) << " │ " << std::setw(7) << stddev(midasLat) << " │\n"
        << "  │  Full frame  │ " << std::setw(7) << avg(totalLat) << " │ " << std::setw(7) << percentile(totalLat, 0.50) << " │ " << std::setw(7) << percentile(totalLat, 0.95) << " │ " << std::setw(7) << percentile(totalLat, 0.99) << " │ " << std::setw(7) << stddev(totalLat) << " │\n"
        << "  └──────────────┴──────────┴──────────┴──────────┴──────────┴──────────┘\n"
        << "\n"
        << "  MiDaS ran on " << midasRuns << " / " << (N - start) << " frames"
        << " (stride max = " << maxStride << ")\n"
        << "\n"
        << "──────────────────────────────────────────────────────────────────\n"
        << "  2. THROUGHPUT (FPS)\n"
        << "──────────────────────────────────────────────────────────────────\n"
        << "\n"
        << "  Stable FPS (EMA) : " << std::setprecision(2) << stableFps << "\n"
        << "  Peak FPS         : " << std::setprecision(2) << peakFps << "\n"
        << "  Average FPS      : " << std::setprecision(2) << (N / totalSec) << "\n"
        << "\n"
        << "──────────────────────────────────────────────────────────────────\n"
        << "  3. THERMAL THROTTLING\n"
        << "──────────────────────────────────────────────────────────────────\n"
        << "\n"
        << "  Temp range       : " << std::setprecision(1) << minTemp << " - " << maxTemp << " C\n"
        << "  Max stride       : " << maxStride << "\n";

    // Count how many frames were at each stride level
    std::array<int, 6> strideCounts{};  // stride 0..5
    for (int i = start; i < static_cast<int>(samples.size()); ++i)
        strideCounts[std::min(5, samples[i].stride)]++;
    rpt << "  Stride histogram :\n";
    for (int s = 1; s <= 5; ++s) {
        if (strideCounts[s] > 0)
            rpt << "    stride=" << s << " : " << strideCounts[s] << " frames ("
                << (strideCounts[s] * 100 / (N - start)) << "%)\n";
    }

    rpt << "\n"
        << "──────────────────────────────────────────────────────────────────\n"
        << "  4. RESOURCE UTILIZATION\n"
        << "──────────────────────────────────────────────────────────────────\n"
        << "\n"
        << std::setprecision(1)
        << "  CPU Core 0 (Capture) : " << avgCpu[0] << " % avg\n"
        << "  CPU Core 1 (Process) : " << avgCpu[1] << " % avg\n"
        << "  CPU Core 2 (TBB)     : " << avgCpu[2] << " % avg\n"
        << "  CPU Core 3 (UI)      : " << avgCpu[3] << " % avg\n"
        << "\n"
        << "  Peak RSS             : " << std::setprecision(1) << maxRss << " MB\n"
        << "\n"
        << "==================================================================\n"
        << "\n";

    const std::string report = rpt.str();
    std::cout << report;

    // Save to file
    std::ofstream outFile(outDir + "/benchmark_report.txt");
    if (outFile.is_open()) {
        outFile << report;
        std::cout << "  [SAVED] benchmark_report.txt\n";
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Adaptive stride logic (mirrors coordinator.cpp)
 * ═══════════════════════════════════════════════════════════════════════════ */

static constexpr float TEMP_WARN_C     = 75.0f;
static constexpr float TEMP_CRITICAL_C = 85.0f;

static int adaptiveStride(int currentStride, float tempC, long elapsedMs, long targetMs) {
    if (tempC > TEMP_CRITICAL_C)
        return 5;
    if (tempC > TEMP_WARN_C)
        return 3;
    if (elapsedMs > targetMs)
        return std::min(currentStride + 1, 5);
    return std::max(1, currentStride - 1);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  main()
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: performance_benchmark_test (--video <file.mp4> | --cam [index]) [yolo.xml] [midas.xml]\n";
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
        std::cerr << "Unknown mode: " << mode << "\n";
        return 1;
    }

    const std::string yoloModel = (argIdx < argc)
        ? argv[argIdx++]
        : "models/yolo26/yolo26_model.xml";
    const std::string midasModel = (argIdx < argc)
        ? argv[argIdx++]
        : "models/midasv21/openvino_midas_v21_small_256.xml";

    const long targetFrameTimeMs = 33; // ~30 fps target for benchmark

    // ── Output directory ────────────────────────────────────────────
    const std::string outDir = "benchmark_results";
#ifdef __linux__
    mkdir(outDir.c_str(), 0755);
#else
    std::system(("mkdir -p " + outDir).c_str());
#endif

    // ── OpenCV setup ────────────────────────────────────────────────
    cv::setNumThreads(0);
    cv::ocl::setUseOpenCL(false);

    // ── OpenVINO Core ───────────────────────────────────────────────
    std::cout << "[BENCH] Initializing OpenVINO...\n";
    auto corePtr = std::make_shared<ov::Core>();
    ov::Core& core = *corePtr;

    try { core.set_property("CPU", ov::hint::inference_precision(ov::element::f32)); }
    catch (...) {}
    try { core.set_property("CPU", ov::enable_mmap(false)); }
    catch (...) {}

    // Log device
    try {
        std::cout << "[BENCH] Device: "
                  << core.get_property("CPU", ov::device::full_name) << '\n';
    } catch (...) {}

#if (defined(__aarch64__) || defined(__ARM_NEON)) && !defined(__APPLE__)
    std::cout << "[BENCH] ARM backend: ACTIVE (KleidiAI + NEON)\n";
#endif

    // ── Load models ─────────────────────────────────────────────────
    std::cout << "[BENCH] Loading YOLO model: " << yoloModel << '\n';
    PerceptionAgent perception(core, yoloModel, "CPU");
    if (!perception.isModelLoaded()) {
        std::cerr << "[BENCH] FATAL: YOLO model failed to compile\n";
        return 1;
    }
    std::cout << "[BENCH] YOLO compiled OK\n";

    std::cout << "[BENCH] Loading MiDaS model: " << midasModel << '\n';
    AnalyticalAgent analytical(core, midasModel, "CPU");
    if (!analytical.isModelLoaded()) {
        std::cerr << "[BENCH] FATAL: MiDaS model failed to compile\n";
        return 1;
    }
    std::cout << "[BENCH] MiDaS compiled OK\n";

    TemporalAnalyzer temporal(10);
    FusionEngine fusion;

    // ── Open video / camera ─────────────────────────────────────────
    cv::VideoCapture cap;
    if (useCamera) {
        if (!cap.open(cameraIndex)) {
            std::cerr << "[BENCH] Failed to open camera " << cameraIndex << '\n';
            return 1;
        }
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    } else {
        if (!cap.open(videoPath)) {
            std::cerr << "[BENCH] Failed to open: " << videoPath << '\n';
            return 1;
        }
    }

    const int totalFrames = useCamera ? 0 : static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    const int frameW = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int frameH = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    std::cout << "[BENCH] Source: " << (useCamera ? "camera" : videoPath) << '\n';
    if (!useCamera)
        std::cout << "[BENCH] Frames: " << totalFrames
                  << "  Resolution: " << frameW << "x" << frameH << '\n';

    // ── Benchmark loop ──────────────────────────────────────────────
    std::vector<FrameSample> samples;
    samples.reserve(useCamera ? 1000 : static_cast<std::size_t>(totalFrames));

    int frameCount = 0;
    int midasStride = 1;
    double smoothFps = 0.0;
    constexpr double FPS_ALPHA = 0.1;

    CpuTimes prevCpu = readCpuTimes();

    const auto benchStart = Clock::now();

    std::cout << "[BENCH] Running full pipeline benchmark...\n\n";

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty())
            break;

        frameCount++;

        FrameSample sample;
        sample.frameNum = frameCount;
        sample.stride = midasStride;

        // ── System metrics (sampled every frame) ────────────────────
        sample.tempC  = readTemperature();
        sample.rssMb  = readRssMb();

        CpuTimes currCpu = readCpuTimes();
        sample.cpuPct = computeCpuPct(prevCpu, currCpu);
        prevCpu = currCpu;

        // ── Full pipeline timing ────────────────────────────────────
        const auto tFrameStart = Clock::now();

        // YOLO inference (timed)
        const auto tYolo0 = Clock::now();
        auto detections = perception.runInference(frame);
        const auto tYolo1 = Clock::now();
        sample.yoloMs = std::chrono::duration<double, std::milli>(tYolo1 - tYolo0).count();

        // MiDaS inference (timed, adaptive stride)
        const bool runMidas = (frameCount % midasStride == 0);
        sample.ranMidas = runMidas;
        cv::Mat depthMap;

        if (runMidas) {
            const auto tMidas0 = Clock::now();
            depthMap = analytical.runInference(frame);
            const auto tMidas1 = Clock::now();
            sample.midasMs = std::chrono::duration<double, std::milli>(tMidas1 - tMidas0).count();
        }

        // Fusion pass (for realistic workload)
        for (const auto& det : detections) {
            if (det.classId != POTHOLE_CLASS_ID) continue;
            sample.potholes++;

            if (runMidas && !depthMap.empty()) {
                cv::Rect roi = analytical.scaleROIToDepth(
                    det.boundingBox, frame.size(), depthMap.size());
                cv::Mat roiDepth = analytical.extractDepthROI(depthMap, roi);
                if (!roiDepth.empty()) {
                    auto residuals = analytical.computeDepthResiduals(roiDepth);
                    auto geom = analytical.computeGeometryMetrics(roiDepth, residuals);
                    auto tempMetrics = temporal.update(geom.depressionScore, geom.roughness);

                    FusionInput fin{};
                    fin.yoloConfidence  = det.confidence;
                    fin.depressionScore = geom.depressionScore;
                    fin.roughness       = geom.roughness;
                    fin.persistence     = tempMetrics.persistence;
                    fin.stability       = tempMetrics.stability;
                    (void)fusion.fuse(fin);
                }
            }
        }

        const auto tFrameEnd = Clock::now();
        sample.totalMs = std::chrono::duration<double, std::milli>(tFrameEnd - tFrameStart).count();

        // FPS
        sample.fps = (sample.totalMs > 0.0) ? (1000.0 / sample.totalMs) : 0.0;
        smoothFps = (frameCount == 1)
            ? sample.fps
            : (FPS_ALPHA * sample.fps + (1.0 - FPS_ALPHA) * smoothFps);
        sample.smoothFps = smoothFps;

        // Detection stats
        sample.detections = static_cast<int>(detections.size());
        for (const auto& d : detections)
            sample.maxConf = std::max(sample.maxConf, d.confidence);

        samples.push_back(sample);

        // Adaptive stride (mirrors coordinator logic)
        const long elapsedMs = static_cast<long>(sample.totalMs);
        midasStride = adaptiveStride(midasStride, sample.tempC, elapsedMs, targetFrameTimeMs);

        // Progress
        if (frameCount % 25 == 0 || frameCount == 1) {
            std::cout << "\r  Frame " << std::setw(5) << frameCount;
            if (totalFrames > 0)
                std::cout << " / " << totalFrames
                          << " (" << std::setw(3) << (frameCount * 100 / totalFrames) << "%)";
            std::cout << std::fixed << std::setprecision(1)
                      << "  | YOLO " << sample.yoloMs << "ms";
            if (runMidas)
                std::cout << "  MiDaS " << sample.midasMs << "ms";
            std::cout << "  | " << std::setprecision(2) << smoothFps << " fps"
                      << "  | s=" << midasStride
                      << "  | " << std::setprecision(1) << sample.tempC << "C"
                      << std::flush;
        }
    }

    const auto benchEnd = Clock::now();
    const double totalSec = std::chrono::duration<double>(benchEnd - benchStart).count();
    cap.release();

    if (samples.empty()) {
        std::cerr << "\n[BENCH] No frames processed!\n";
        return 1;
    }

    // ── Generate graphs ─────────────────────────────────────────────
    std::cout << "\n\n[BENCH] Generating charts...\n";

    generateLatencyChart(samples, outDir);
    generateFpsChart(samples, outDir);
    generateThermalChart(samples, outDir);
    generateCpuChart(samples, outDir);
    generateMemoryChart(samples, outDir);

    // ── Print summary report ────────────────────────────────────────
    printAndSaveReport(samples, totalSec,
                       useCamera ? "camera" : videoPath,
                       yoloModel, midasModel,
                       frameW, frameH, outDir);

    std::cout << "[BENCH] All results saved to: " << outDir << "/\n";
    return 0;
}

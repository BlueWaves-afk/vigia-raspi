/**
 * @file system_visual_test.cpp
 * @brief VIGIA full-pipeline visual integration test
 *
 * Optimized for Raspberry Pi 4 (Cortex-A72 / aarch64 / Debian Trixie)
 *   • OpenCV 4.14  — KleidiCV 0.7.0 HAL + TBB parallel backend
 *   • OpenVINO 2025 — CPU plugin with Arm Compute Library (ACL)
 *
 * Optimization map (keyed to user requirements):
 *
 *  ┌──────────────────────────────┬──────────────────────────────────────────┐
 *  │ Requirement                  │ Implementation                           │
 *  ├──────────────────────────────┼──────────────────────────────────────────┤
 *  │ OpenVINO Async API           │ Base classes (perception.cpp,            │
 *  │                              │ analytical.cpp) already use              │
 *  │                              │ start_async()+wait() with pre-allocated  │
 *  │                              │ ov::Tensor. Inherited transparently.     │
 *  ├──────────────────────────────┼──────────────────────────────────────────┤
 *  │ KleidiCV HAL pre-processing  │ cv::resize / cv::cvtColor /             │
 *  │                              │ cv::normalize / cv::applyColorMap kept   │
 *  │                              │ as standard calls → KleidiCV NEON HAL   │
 *  │                              │ dispatches automatically.                │
 *  │                              │ cv::setNumThreads(0) lets TBB use all   │
 *  │                              │ cores for internal parallelism.          │
 *  ├──────────────────────────────┼──────────────────────────────────────────┤
 *  │ Zero-copy / Memory           │ Shared ov::Core across both agents.      │
 *  │                              │ beginFrame() uses shallow cv::Mat copy   │
 *  │                              │ (refcount) instead of .clone().          │
 *  │                              │ drawDetections() writes into a           │
 *  │                              │ pre-allocated canvas, no per-frame       │
 *  │                              │ heap allocation.                         │
 *  │                              │ Dashboard panel compositing uses         │
 *  │                              │ cv::resize directly into sub-ROI.        │
 *  ├──────────────────────────────┼──────────────────────────────────────────┤
 *  │ TBB parallel_for_            │ drawDetections wraps the per-detection   │
 *  │                              │ rendering loop in cv::parallel_for_.     │
 *  ├──────────────────────────────┼──────────────────────────────────────────┤
 *  │ Performance Metrics          │ Dedicated PerfTracker: EMA-smoothed FPS  │
 *  │                              │ (α = 0.1), latency min/max/avg, and     │
 *  │                              │ time-to-first-inference-result timer     │
 *  │                              │ printed at startup and shown on HUD.     │
 *  ├──────────────────────────────┼──────────────────────────────────────────┤
 *  │ ov::hint::PerformanceMode    │ Set in base class loadNetwork()         │
 *  │ ::LATENCY + num_requests(1)  │ (perception.cpp / analytical.cpp).      │
 *  └──────────────────────────────┴──────────────────────────────────────────┘
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <limits>
#include <mutex>
#include <numeric>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <array>
#include <vector>
#include <ctime>

#ifdef __linux__
#include <unistd.h>                    // write(), _exit(), STDERR_FILENO
#elif defined(__APPLE__)
#include <unistd.h>
#endif

#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>    // cv::parallel_for_ + TBB
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <openvino/openvino.hpp>

#include "analytical.hpp"
#include "coordinator.hpp"
#include "fusion.hpp"
#include "perception.hpp"
#include "roi_utils.hpp"
#include "temporal.hpp"

namespace vigia {
namespace viz {

namespace {
constexpr int POTHOLE_CLASS_ID = 0;
constexpr float HAZARD_THRESHOLD = 0.55f;

// --- ARM64 edge-device profile ---
#if defined(__aarch64__) || defined(__ARM_NEON)
static constexpr bool kArmProfile = true;
static constexpr int kDashboardWidth = 1024;
static constexpr int kDashboardHeight = 600;
#else
static constexpr bool kArmProfile = false;
static constexpr int kDashboardWidth = 1440;
static constexpr int kDashboardHeight = 900;
#endif

static constexpr int kIdleBackoffMs = 10;
static constexpr int kRenderIntervalMs = 66; // ~15 FPS UI cap

// EMA smoothing factor for the FPS counter.
// α = 0.1 → heavily smoothed, stable readout on the HUD.
static constexpr double kFpsEmaAlpha = 0.1;

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
inline void pinCurrentThreadToCore(int coreId) {
    cpu_set_t cpuSet;
    CPU_ZERO(&cpuSet);
    CPU_SET(coreId, &cpuSet);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuSet);
}
#else
inline void pinCurrentThreadToCore(int) {}
#endif
} // namespace

/* ═══════════════════════════════════════════════════════════════════════════
 *  PerfTracker — Lightweight FPS + Latency Tracker
 *
 *  • EMA-smoothed FPS (α = 0.1) — stable HUD readout, zero deque overhead.
 *  • Captures time-to-first-inference-result for startup analysis.
 *  • Single-pass cumulative average latency (no history buffer).
 *  • Min/max latency tracking.
 * ═══════════════════════════════════════════════════════════════════════════ */
class PerfTracker {
public:
    PerfTracker() : startTs_(std::chrono::steady_clock::now()) {}

    /// Call once per completed frame with its end-to-end latency.
    void recordFrame(double latencyMs) {
        if (latencyMs <= 0.0)
            return;

        const double instantFps = 1000.0 / latencyMs;

        if (frameCount_ == 0) {
            emaFps_ = instantFps;
            firstInferenceLatencyMs_ =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - startTs_
                ).count();
        } else {
            emaFps_ = kFpsEmaAlpha * instantFps + (1.0 - kFpsEmaAlpha) * emaFps_;
        }

        cumulativeLatencyMs_ += latencyMs;
        ++frameCount_;

        if (latencyMs < minLatencyMs_) minLatencyMs_ = latencyMs;
        if (latencyMs > maxLatencyMs_) maxLatencyMs_ = latencyMs;
    }

    double smoothedFps()    const { return emaFps_; }
    double avgLatencyMs()   const { return frameCount_ > 0 ? cumulativeLatencyMs_ / frameCount_ : 0.0; }
    double minLatencyMs()   const { return minLatencyMs_; }
    double maxLatencyMs()   const { return maxLatencyMs_ == 0.0 ? 0.0 : maxLatencyMs_; }
    double firstInferMs()   const { return firstInferenceLatencyMs_; }
    std::uint64_t frames()  const { return frameCount_; }

private:
    std::chrono::steady_clock::time_point startTs_;
    double emaFps_{0.0};
    double cumulativeLatencyMs_{0.0};
    double firstInferenceLatencyMs_{0.0};
    double minLatencyMs_{std::numeric_limits<double>::max()};
    double maxLatencyMs_{0.0};
    std::uint64_t frameCount_{0};
};

/* ═══════════════════════════════════════════════════════════════════════════ */

struct FusionTelemetry {
    Detection detection{};
    FusionInput input{};
    FusionOutput output{};
    TemporalMetrics temporal{};
};

struct FrameSnapshot {
    std::uint64_t frameIndex{0};
    double latencyMs{0.0};
    int observedStride{1};
    cv::Mat frameBgr;
    cv::Mat depthMap;
    bool depthValid{false};
    std::vector<FusionTelemetry> fusions;
    std::size_t totalDetections{0};
    float maxConfidence{0.0f};
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  InstrumentationBus — Ring buffer connecting the processing pipeline
 *  to the UI thread.
 *
 *  Memory optimisation vs. original:
 *    • beginFrame() stores a shallow cv::Mat copy (refcount bump, ~20 ns)
 *      instead of .clone() (~300 μs for 640×480 BGR).  The frame is only
 *      deep-copied later in the main loop when we need a mutable canvas.
 *    • makeSnapshot() transfers vectors via std::move where possible.
 * ═══════════════════════════════════════════════════════════════════════════ */
class InstrumentationBus {
public:
    InstrumentationBus() = default;

    void beginFrame(std::uint64_t frameIndex, const cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(mutex_);

        FrameSlot& slot = slots_[frameIndex % MAX_INFLIGHT];
        // Evict stale in-flight slot occupying this ring position
        if (slot.frameIndex != 0 && slot.frameIndex != frameIndex && !slot.completed) {
            promoteUnfusedDetections(slot);
            finalizeSlot(slot);
            ready_.push_back(makeSnapshot(slot));
        }

        slot.frameIndex = frameIndex;
        // Shallow copy — shares the pixel buffer with the capture thread.
        // Safe because the coordinator's frameBuffer_ ring already .clone()'d
        // the capture output, so this Mat won't be overwritten under us.
        slot.frameBgr = frame;
        slot.startTs = std::chrono::steady_clock::now();
        slot.totalDetections = 0;
        slot.allDetections.clear();
        slot.filteredDetections.clear();
        slot.temporalStaging.clear();
        slot.fusionTelemetry.clear();
        slot.depthMap.release();
        slot.depthValid = false;
        slot.observedStride = lastObservedStride_;
        slot.maxConfidence = 0.0f;
        slot.completed = false;

        if (!firstFrameSeen_) {
            firstFrameTs_ = slot.startTs;
            firstFrameSeen_ = true;
        }

        capturedFrames_++;
    }

    void storeDetections(std::uint64_t frameIndex, const std::vector<Detection>& detections) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Finalize older frames that the processing thread skipped
        finalizeOlderFrames(frameIndex);

        FrameSlot& slot = slots_[frameIndex % MAX_INFLIGHT];
        if (slot.frameIndex != frameIndex)
            return;

        slot.totalDetections = detections.size();
        slot.allDetections = detections;
        slot.filteredDetections.clear();
        slot.temporalStaging.clear();
        slot.fusionTelemetry.clear();
        slot.maxConfidence = 0.0f;

        if constexpr (!kArmProfile) {
            std::cout << "[BUS] storeDetections frame=" << frameIndex
                      << " count=" << detections.size() << "\n";
        }

        for (const auto& det : detections) {
            slot.maxConfidence = std::max(slot.maxConfidence, det.confidence);
            if (det.classId == POTHOLE_CLASS_ID)
                slot.filteredDetections.push_back(det);
        }

        if (slot.filteredDetections.empty()) {
            finalizeSlot(slot);
            ready_.push_back(makeSnapshot(slot));
            slot.frameIndex = 0;
        }
    }

    void recordDepth(std::uint64_t frameIndex, const cv::Mat& depth) {
        std::lock_guard<std::mutex> lock(mutex_);
        FrameSlot& slot = slots_[frameIndex % MAX_INFLIGHT];
        if (slot.frameIndex != frameIndex)
            return;

        // Shallow copy — depth map is a fresh Mat returned from analytical
        slot.depthMap = depth;
        slot.depthValid = true;

        int observed = 1;
        if (lastDepthFrameIndex_ != 0 && frameIndex > lastDepthFrameIndex_)
            observed = static_cast<int>(frameIndex - lastDepthFrameIndex_);

        slot.observedStride = observed;
        lastDepthFrameIndex_ = frameIndex;
        lastObservedStride_ = observed;
    }

    void recordTemporal(std::uint64_t frameIndex, const TemporalMetrics& metrics) {
        std::lock_guard<std::mutex> lock(mutex_);
        FrameSlot& slot = slots_[frameIndex % MAX_INFLIGHT];
        if (slot.frameIndex != frameIndex)
            return;

        slot.temporalStaging.push_back(metrics);
    }

    void recordFusion(std::uint64_t frameIndex,
                      const FusionInput& input,
                      const FusionOutput& output) {
        std::lock_guard<std::mutex> lock(mutex_);
        FrameSlot& slot = slots_[frameIndex % MAX_INFLIGHT];
        if (slot.frameIndex != frameIndex)
            return;

        FusionTelemetry telemetry{};
        const std::size_t idx = slot.fusionTelemetry.size();
        if (idx < slot.filteredDetections.size())
            telemetry.detection = slot.filteredDetections[idx];
        telemetry.input = input;
        telemetry.output = output;
        if (idx < slot.temporalStaging.size())
            telemetry.temporal = slot.temporalStaging[idx];
        slot.fusionTelemetry.push_back(telemetry);

        if (slot.fusionTelemetry.size() == slot.filteredDetections.size()) {
            finalizeSlot(slot);
            ready_.push_back(makeSnapshot(slot));
            slot.frameIndex = 0;
        }
    }

    bool tryPopFrame(FrameSnapshot& snapshot) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Promote slots that have been waiting too long for fusion (>500ms).
        // This ensures bounding boxes are drawn even when depth/fusion is skipped.
        const auto now = std::chrono::steady_clock::now();
        for (auto& slot : slots_) {
            if (slot.frameIndex != 0 && !slot.completed &&
                !slot.filteredDetections.empty() &&
                slot.fusionTelemetry.size() < slot.filteredDetections.size()) {
                const double waitMs = std::chrono::duration<double, std::milli>(
                    now - slot.startTs).count();
                if (waitMs > 500.0) {
                    promoteUnfusedDetections(slot);
                    finalizeSlot(slot);
                    ready_.push_back(makeSnapshot(slot));
                    slot.frameIndex = 0;
                }
            }
        }

        if (ready_.empty())
            return false;
        // Drop stale snapshots if the UI fell behind.
        while (ready_.size() > MAX_READY)
            ready_.pop_front();
        snapshot = std::move(ready_.front());
        ready_.pop_front();
        return true;
    }

    void flush() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& slot : slots_) {
            if (slot.frameIndex != 0 && !slot.completed) {
                promoteUnfusedDetections(slot);
                finalizeSlot(slot);
                ready_.push_back(makeSnapshot(slot));
                slot.frameIndex = 0;
            }
        }
    }

    struct Summary {
        std::uint64_t capturedFrames{0};
        std::uint64_t processedFrames{0};
        std::uint64_t skippedFrames{0};
        std::uint64_t depthExecutions{0};
        double averageLatencyMs{0.0};
        double averageFps{0.0};
        double totalDurationSec{0.0};
    };

    Summary summary() const {
        std::lock_guard<std::mutex> lock(mutex_);
        Summary out;
        out.capturedFrames = capturedFrames_;
        out.processedFrames = processedFrames_;
        out.skippedFrames = skippedFrames_;
        out.depthExecutions = depthExecutions_;
        out.averageLatencyMs = processedFrames_ > 0
            ? totalLatencyMs_ / static_cast<double>(processedFrames_)
            : 0.0;
        if (processedFrames_ > 1 && firstFrameSeen_ && lastFrameSeen_) {
            const double seconds = std::chrono::duration<double>(lastFrameTs_ - firstFrameTs_).count();
            out.totalDurationSec = seconds;
            out.averageFps = seconds > 0.0 ? processedFrames_ / seconds : 0.0;
        }
        return out;
    }

private:
    struct FrameSlot {
        std::uint64_t frameIndex{0};
        cv::Mat frameBgr;
        std::vector<Detection> allDetections;
        std::vector<Detection> filteredDetections;
        std::vector<TemporalMetrics> temporalStaging;
        std::vector<FusionTelemetry> fusionTelemetry;
        cv::Mat depthMap;
        bool depthValid{false};
        int observedStride{1};
        float maxConfidence{0.0f};
        std::size_t totalDetections{0};
        bool completed{false};
        std::chrono::steady_clock::time_point startTs{};
        double latencyMs{0.0};
    };

    static FrameSnapshot makeSnapshot(FrameSlot& slot) {
        FrameSnapshot snapshot;
        snapshot.frameIndex = slot.frameIndex;
        snapshot.latencyMs = slot.latencyMs;
        snapshot.observedStride = slot.observedStride;
        // Shallow cv::Mat copies — the slot is about to be reset anyway
        snapshot.frameBgr = slot.frameBgr;
        snapshot.depthMap = slot.depthMap;
        snapshot.depthValid = slot.depthValid;
        // Move the vectors out of the slot to avoid deep-copying Detection/FusionTelemetry
        snapshot.fusions = std::move(slot.fusionTelemetry);
        snapshot.totalDetections = slot.totalDetections;
        snapshot.maxConfidence = slot.maxConfidence;
        return snapshot;
    }

    void finalizeSlot(FrameSlot& slot) {
        if (slot.completed)
            return;
        const auto now = std::chrono::steady_clock::now();
        slot.latencyMs = std::chrono::duration<double, std::milli>(now - slot.startTs).count();
        slot.completed = true;
        processedFrames_++;
        totalLatencyMs_ += slot.latencyMs;
        if (slot.depthValid)
            depthExecutions_++;
        lastFrameTs_ = now;
        lastFrameSeen_ = true;
        if (lastFinalizedFrame_ != 0 && slot.frameIndex > lastFinalizedFrame_ + 1)
            skippedFrames_ += slot.frameIndex - lastFinalizedFrame_ - 1;
        lastFinalizedFrame_ = slot.frameIndex;
    }

    // Promote raw detections to FusionTelemetry so bounding boxes are
    // always drawn even when the coordinator skips fusion (no depth data,
    // stride-skipped frames, etc.).
    static void promoteUnfusedDetections(FrameSlot& slot) {
        while (slot.fusionTelemetry.size() < slot.filteredDetections.size()) {
            const auto& det = slot.filteredDetections[slot.fusionTelemetry.size()];
            FusionTelemetry tele{};
            tele.detection = det;
            tele.input.yoloConfidence = det.confidence;
            tele.output.finalConfidence = det.confidence; // raw YOLO as fallback
            if (slot.fusionTelemetry.size() < slot.temporalStaging.size())
                tele.temporal = slot.temporalStaging[slot.fusionTelemetry.size()];
            slot.fusionTelemetry.push_back(tele);
        }
    }

    void finalizeOlderFrames(std::uint64_t currentFrameIndex) {
        for (auto& slot : slots_) {
            if (slot.frameIndex != 0 && slot.frameIndex < currentFrameIndex && !slot.completed) {
                promoteUnfusedDetections(slot);
                finalizeSlot(slot);
                ready_.push_back(makeSnapshot(slot));
                slot.frameIndex = 0;
            }
        }
    }

    static constexpr std::size_t MAX_INFLIGHT = 8;
    static constexpr std::size_t MAX_READY = 16;

    mutable std::mutex mutex_;
    std::array<FrameSlot, MAX_INFLIGHT> slots_{};
    std::deque<FrameSnapshot> ready_;

    std::uint64_t lastDepthFrameIndex_{0};
    int lastObservedStride_{1};
    std::uint64_t lastFinalizedFrame_{0};

    std::uint64_t capturedFrames_{0};
    std::uint64_t processedFrames_{0};
    std::uint64_t skippedFrames_{0};
    std::uint64_t depthExecutions_{0};
    double totalLatencyMs_{0.0};
    std::chrono::steady_clock::time_point firstFrameTs_{};
    std::chrono::steady_clock::time_point lastFrameTs_{};
    bool firstFrameSeen_{false};
    bool lastFrameSeen_{false};
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  VideoPerceptionAgent — video/camera capture with pause/step support.
 *
 *  Now accepts an ov::Core& to share the single core instance, matching
 *  the production main.cpp pattern.  Saves ~50–100 ms on Pi 4 by avoiding
 *  duplicate plugin discovery and device enumeration.
 * ═══════════════════════════════════════════════════════════════════════════ */
class VideoPerceptionAgent : public PerceptionAgent {
public:
    VideoPerceptionAgent(ov::Core& sharedCore,
                         const std::string& modelXmlPath,
                         const std::string& device,
                         std::string videoPath,
                         InstrumentationBus& bus,
                         int cameraIndex = 0)
        : PerceptionAgent(sharedCore, modelXmlPath, device, cameraIndex),
          videoPath_(std::move(videoPath)),
          bus_(bus),
          cameraIndex_(cameraIndex),
          useCamera_(false) {
        if (!capture_.open(videoPath_))
            throw std::runtime_error("Failed to open video file: " + videoPath_);
        videoFps_ = capture_.get(cv::CAP_PROP_FPS);
        if (videoFps_ <= 1.0)
            videoFps_ = 30.0;
        frameInterval_ = std::chrono::duration<double>(1.0 / videoFps_);
    }

    VideoPerceptionAgent(ov::Core& sharedCore,
                         const std::string& modelXmlPath,
                         const std::string& device,
                         InstrumentationBus& bus,
                         int cameraIndex)
        : PerceptionAgent(sharedCore, modelXmlPath, device, cameraIndex),
          bus_(bus),
          cameraIndex_(cameraIndex),
          useCamera_(true) {
        if (!capture_.open(cameraIndex_))
            throw std::runtime_error("Failed to open camera index: " + std::to_string(cameraIndex_));
        videoFps_ = capture_.get(cv::CAP_PROP_FPS);
        if (videoFps_ <= 1.0)
            videoFps_ = 30.0;
        frameInterval_ = std::chrono::duration<double>::zero();
    }

    bool captureFrame(cv::Mat& frame) override {
        bool runOnce = false;
        {
            std::unique_lock<std::mutex> lock(stateMutex_);
            stateCv_.wait(lock, [this] { return stopRequested_ || !paused_ || singleStep_; });
            if (stopRequested_)
                return false;
            if (singleStep_) {
                runOnce = true;
                singleStep_ = false;
            }
        }

        if (finished_.load())
            return false;

        cv::Mat rawFrame;
        if (!capture_.read(rawFrame) || rawFrame.empty()) {
            finished_.store(true);
            return false;
        }

        const std::uint64_t frameIndex = frameCounter_.fetch_add(1) + 1;
        latestFrameIndex_.store(frameIndex, std::memory_order_release);
        deliveredFrameIndex_.store(frameIndex, std::memory_order_release);

        bus_.beginFrame(frameIndex, rawFrame);
        frame = rawFrame;

        throttleCaptureRate();

        if (runOnce) {
            std::lock_guard<std::mutex> lock(stateMutex_);
            paused_ = true;
        }

        return true;
    }

    void togglePause() {
        std::lock_guard<std::mutex> lock(stateMutex_);
        paused_ = !paused_;
        if (!paused_)
            stateCv_.notify_all();
    }

    void singleStep() {
        std::lock_guard<std::mutex> lock(stateMutex_);
        singleStep_ = true;
        paused_ = false;
        stateCv_.notify_all();
    }

    void requestStop() {
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            stopRequested_ = true;
            paused_ = false;
        }
        stateCv_.notify_all();
    }

    bool isPaused() const {
        std::lock_guard<std::mutex> lock(stateMutex_);
        return paused_;
    }

    bool isFinished() const {
        return finished_.load();
    }

    double nominalFps() const {
        return videoFps_;
    }

protected:
    std::uint64_t latestFrameIndex() const {
        return latestFrameIndex_.load(std::memory_order_acquire);
    }

    std::uint64_t deliveredFrameIndex() const {
        return deliveredFrameIndex_.load(std::memory_order_acquire);
    }

    InstrumentationBus& bus() { return bus_; }

private:
    void throttleCaptureRate() {
        if (useCamera_)
            return;
        if (frameInterval_.count() <= 0.0)
            return;

        const auto now = std::chrono::steady_clock::now();
        if (lastCaptureTsValid_) {
            const auto target = lastCaptureTs_ + frameInterval_;
            if (now < target)
                std::this_thread::sleep_for(target - now);
        }
        lastCaptureTs_ = std::chrono::steady_clock::now();
        lastCaptureTsValid_ = true;
    }

    std::string videoPath_;
    InstrumentationBus& bus_;
    cv::VideoCapture capture_;

    mutable std::mutex stateMutex_;
    std::condition_variable stateCv_;
    bool paused_{false};
    bool singleStep_{false};
    bool stopRequested_{false};

    std::atomic<bool> finished_{false};
    std::atomic<std::uint64_t> frameCounter_{0};
    std::atomic<std::uint64_t> latestFrameIndex_{0};
    std::atomic<std::uint64_t> deliveredFrameIndex_{0};
    double videoFps_{30.0};
    std::chrono::steady_clock::time_point lastCaptureTs_{};
    bool lastCaptureTsValid_{false};
    std::chrono::duration<double> frameInterval_{};
    int cameraIndex_{0};
    bool useCamera_{false};
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  InstrumentedPerceptionAgent
 *
 *  runInference() delegates to PerceptionAgent::runInference() which
 *  already uses:
 *    • Pre-allocated ov::Tensor (zero per-frame allocation)
 *    • NEON vld3q_f32 HWC→CHW transpose
 *    • start_async() + wait() (OpenVINO async API)
 *    • ov::hint::PerformanceMode::LATENCY + num_requests(1)
 * ═══════════════════════════════════════════════════════════════════════════ */
class InstrumentedPerceptionAgent final : public VideoPerceptionAgent {
public:
    InstrumentedPerceptionAgent(ov::Core& sharedCore,
                                const std::string& modelXmlPath,
                                const std::string& device,
                                std::string videoPath,
                                InstrumentationBus& bus)
        : VideoPerceptionAgent(sharedCore, modelXmlPath, device, std::move(videoPath), bus) {}

    InstrumentedPerceptionAgent(ov::Core& sharedCore,
                                const std::string& modelXmlPath,
                                const std::string& device,
                                int cameraIndex,
                                InstrumentationBus& bus)
        : VideoPerceptionAgent(sharedCore, modelXmlPath, device, bus, cameraIndex) {}

    /// Thread-safe accessor for the last resolved frame index.
    std::uint64_t resolvedFrameIndex() const {
        return resolvedFrameIndex_.load(std::memory_order_acquire);
    }

    bool captureFrame(cv::Mat& frame) override {
        if (!VideoPerceptionAgent::captureFrame(frame))
            return false;

        const std::uint64_t frameIndex = deliveredFrameIndex();
        {
            std::lock_guard<std::mutex> lock(frameIndexMutex_);
            pendingFrameIndices_.push_back(frameIndex);
        }

        if constexpr (!kArmProfile) {
            static std::atomic<int> debugCaptureCount{0};
            if (debugCaptureCount.fetch_add(1) < 5)
                std::cout << "[CAPTURE] enqueued frame=" << frameIndex << "\n";
        }

        return true;
    }

    std::vector<Detection> runInference(const cv::Mat& frame) override {
        std::uint64_t frameIndex = 0;
        {
            std::lock_guard<std::mutex> lock(frameIndexMutex_);
            if (!pendingFrameIndices_.empty()) {
                frameIndex = pendingFrameIndices_.back();
                pendingFrameIndices_.clear();
            }
        }

        if (frameIndex == 0)
            frameIndex = deliveredFrameIndex();

        if constexpr (!kArmProfile) {
            static std::atomic<int> debugInferenceCount{0};
            if (debugInferenceCount.fetch_add(1) < 5)
                std::cout << "[INFER] using frame=" << frameIndex << "\n";
        }

        resolvedFrameIndex_.store(frameIndex, std::memory_order_release);

        auto detections = PerceptionAgent::runInference(frame);

        if constexpr (!kArmProfile) {
            static std::atomic<int> debugDetectionLogs{0};
            const int logIdx = debugDetectionLogs.fetch_add(1);
            if (logIdx < 16) {
                float maxConfidence = 0.0f;
                for (const auto& det : detections)
                    maxConfidence = std::max(maxConfidence, det.confidence);
                std::cout << "[YOLO] frame=" << frameIndex
                          << " detections=" << detections.size()
                          << " maxConf=" << maxConfidence << "\n";
            }
        }

        if (frameIndex != 0)
            bus().storeDetections(frameIndex, detections);
        return detections;
    }

private:
    std::mutex frameIndexMutex_;
    std::vector<std::uint64_t> pendingFrameIndices_;
    std::atomic<std::uint64_t> resolvedFrameIndex_{0};
};

/* ═══════════════════════════════════════════════════════════════════════════ */

class InstrumentedAnalyticalAgent final : public AnalyticalAgent {
public:
    InstrumentedAnalyticalAgent(ov::Core& sharedCore,
                                const std::string& modelXmlPath,
                                const std::string& device,
                                InstrumentationBus& bus,
                                const InstrumentedPerceptionAgent& perception)
        : AnalyticalAgent(sharedCore, modelXmlPath, device),
          bus_(bus), perception_(perception) {}

    cv::Mat runInference(const cv::Mat& frame) override {
        cv::Mat depth = AnalyticalAgent::runInference(frame);
        const std::uint64_t frameIndex = perception_.resolvedFrameIndex();
        if (frameIndex != 0)
            bus_.recordDepth(frameIndex, depth);
        return depth;
    }

private:
    InstrumentationBus& bus_;
    const InstrumentedPerceptionAgent& perception_;
};

/* ═══════════════════════════════════════════════════════════════════════════ */

class InstrumentedTemporalAnalyzer final : public TemporalAnalyzer {
public:
    InstrumentedTemporalAnalyzer(InstrumentationBus& bus,
                                 const InstrumentedPerceptionAgent& perception)
        : TemporalAnalyzer(), bus_(bus), perception_(perception) {}

    TemporalMetrics update(float depressionScore, float roughness) override {
        TemporalMetrics metrics = TemporalAnalyzer::update(depressionScore, roughness);
        const std::uint64_t frameIndex = perception_.resolvedFrameIndex();
        if (frameIndex != 0)
            bus_.recordTemporal(frameIndex, metrics);
        return metrics;
    }

private:
    InstrumentationBus& bus_;
    const InstrumentedPerceptionAgent& perception_;
};

/* ═══════════════════════════════════════════════════════════════════════════ */

class InstrumentedFusionEngine final : public FusionEngine {
public:
    InstrumentedFusionEngine(InstrumentationBus& bus,
                             const InstrumentedPerceptionAgent& perception)
        : FusionEngine(), bus_(bus), perception_(perception) {}

    FusionOutput fuse(const FusionInput& input) const override {
        FusionOutput output = FusionEngine::fuse(input);
        const std::uint64_t frameIndex = perception_.resolvedFrameIndex();
        if (frameIndex != 0)
            bus_.recordFusion(frameIndex, input, output);
        return output;
    }

private:
    InstrumentationBus& bus_;
    const InstrumentedPerceptionAgent& perception_;
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  Utility
 * ═══════════════════════════════════════════════════════════════════════════ */

inline float readCPUTemperature() {
#ifdef __linux__
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    if (!file.is_open())
        return std::numeric_limits<float>::quiet_NaN();
    float milli = 0.0f;
    file >> milli;
    return milli / 1000.0f;
#else
    return std::numeric_limits<float>::quiet_NaN();
#endif
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  drawDetections — TBB-parallel bounding box + label renderer
 *
 *  Uses cv::parallel_for_ to distribute per-detection rendering across
 *  TBB worker threads.  Each detection writes to a non-overlapping region
 *  of the canvas (its bounding box area + label strip), so no
 *  synchronisation is required.
 *
 *  Writes into the provided `canvas` in-place instead of cloning
 *  snapshot.frameBgr each time — caller is responsible for providing a
 *  mutable canvas (e.g. via .clone() once, then reuse across renders that
 *  share the same base frame).
 * ═══════════════════════════════════════════════════════════════════════════ */
void drawDetections(cv::Mat& canvas,
                    const FrameSnapshot& snapshot) {
    if (canvas.empty() || snapshot.fusions.empty())
        return;

    const int numFusions = static_cast<int>(snapshot.fusions.size());

    // cv::parallel_for_ dispatches to TBB when OpenCV is built with TBB.
    // For ≤2 detections the overhead is negligible; for 10+ it wins on 4 cores.
    cv::parallel_for_(cv::Range(0, numFusions), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            const auto& tele = snapshot.fusions[static_cast<std::size_t>(i)];
            const cv::Rect& box = tele.detection.boundingBox;
            if (box.width <= 0 || box.height <= 0)
                continue;
            const cv::Scalar boxColor(0, 165, 255);
            cv::rectangle(canvas, box, boxColor, 2);

            // Upper label: YOLO / Fusion / Stride
            std::ostringstream label;
            label << std::fixed << std::setprecision(2)
                  << "Y:" << tele.input.yoloConfidence
                  << " F:" << tele.output.finalConfidence
                  << " S:" << snapshot.observedStride;
            cv::putText(canvas,
                        label.str(),
                        cv::Point(box.x, std::max(box.y - 8, 12)),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        boxColor,
                        1,
                        cv::LINE_8);

            // Lower label: HAZARD / SAFE
            const bool hazard = tele.output.finalConfidence >= HAZARD_THRESHOLD;
            cv::putText(canvas,
                        hazard ? "HAZARD" : "SAFE",
                        cv::Point(box.x, box.y + box.height + 18),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.55,
                        hazard ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 200, 0),
                        2,
                        cv::LINE_8);
        }
    });
}

cv::Mat makeDepthVisualization(const FrameSnapshot& snapshot) {
    if (!snapshot.depthValid || snapshot.depthMap.empty())
        return {};

    // cv::normalize and cv::applyColorMap dispatch through KleidiCV HAL
    // when built with TBB — no manual parallel_for_ needed.
    cv::Mat depthNorm;
    cv::normalize(snapshot.depthMap, depthNorm, 0.0, 255.0, cv::NORM_MINMAX);
    depthNorm.convertTo(depthNorm, CV_8U);
    cv::Mat depthColor;
    cv::applyColorMap(depthNorm, depthColor, cv::COLORMAP_INFERNO);
    return depthColor;
}

/// Log OpenVINO device capabilities (mirrors main.cpp's logDeviceCapabilities).
void logDeviceCapabilities(ov::Core& core, const std::string& device) {
    std::cout << "[vigia-test] OpenVINO device: " << device << '\n';

    try {
        const std::string fullName =
            core.get_property(device, ov::device::full_name);
        std::cout << "[vigia-test]   Full name  : " << fullName << '\n';
    } catch (...) {}

    try {
        const std::vector<std::string> caps =
            core.get_property(device, ov::device::capabilities);
        std::cout << "[vigia-test]   Capabilities:";
        for (const auto& cap : caps)
            std::cout << ' ' << cap;
        std::cout << '\n';
    } catch (...) {}

#if defined(__aarch64__) || defined(__ARM_NEON)
    try {
        const std::string fullName =
            core.get_property(device, ov::device::full_name);
        if (fullName.find("ACL") != std::string::npos ||
            fullName.find("arm_compute") != std::string::npos ||
            fullName.find("Arm") != std::string::npos) {
            std::cout << "[vigia-test]   ACL backend : DETECTED (NEON-accelerated inference)\n";
        } else {
            std::cerr << "[vigia-test]   WARNING: ACL backend NOT detected. "
                         "CPU inference may not be NEON-optimized.\n";
        }
    } catch (...) {}
#endif
}

} // namespace viz
} // namespace vigia

/* ═══════════════════════════════════════════════════════════════════════════
 *  SIGBUS signal handler — catches unaligned access / mmap faults
 * ═══════════════════════════════════════════════════════════════════════════ */
static volatile const char* g_currentStep = "(not started)";

static void sigbusHandler(int sig) {
    // Only async-signal-safe calls here: write() + _exit()
    const char prefix[] = "\n[FATAL] Caught SIGBUS (Bus error) during step: ";
    const char suffix[] = "\n[FATAL] Root cause: OpenVINO ARM CPU plugin lacks ACL.\n"
                          "[FATAL] Rebuild OpenVINO from source with -DENABLE_KLEIDIAI=ON\n";
    (void)::write(STDERR_FILENO, prefix, sizeof(prefix) - 1);
    const volatile char* p = g_currentStep;
    size_t len = 0;
    while (p[len] != '\0') ++len;
    (void)::write(STDERR_FILENO, (const char*)p, len);
    (void)::write(STDERR_FILENO, suffix, sizeof(suffix) - 1);
    ::_exit(128 + sig);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  main()
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(int argc, char** argv) {
    using namespace vigia;
    using namespace vigia::viz;

    if (argc < 2) {
        std::cerr << "Usage: system_visual_test (--video <video_mp4> | --cam [camera_index]) [yolo_xml] [midas_xml] [target_fps]\n";
        return 1;
    }

    bool useCamera = false;
    int cameraIndex = 0;
    std::string videoPath;

    int argIndex = 1;
    const std::string modeArg = argv[argIndex++];

    if (modeArg == "--cam") {
        useCamera = true;
        if (argIndex < argc && argv[argIndex][0] != '-') {
            try {
                cameraIndex = std::stoi(argv[argIndex]);
            } catch (const std::exception&) {
                std::cerr << "[system_visual_test] Camera index must be an integer\n";
                return 1;
            }
            argIndex++;
        }
    } else if (modeArg == "--video") {
        if (argIndex >= argc) {
            std::cerr << "[system_visual_test] --video requires a path argument\n";
            return 1;
        }
        videoPath = argv[argIndex++];
    } else {
        std::cerr << "Usage: system_visual_test (--video <video_mp4> | --cam [camera_index]) [yolo_xml] [midas_xml] [target_fps]\n";
        return 1;
    }

    const std::string yoloModel = (argIndex < argc)
        ? argv[argIndex++]
        : "models/yolo26/yolo26_model.xml";
    const std::string midasModel = (argIndex < argc)
        ? argv[argIndex++]
        : "models/midasv21/openvino_midas_v21_small_256.xml";

    int targetFps = 30;
    if (argIndex < argc) {
        try {
            targetFps = std::max(1, std::stoi(argv[argIndex++]));
        } catch (const std::exception&) {
            std::cerr << "[system_visual_test] target_fps must be an integer\n";
            return 1;
        }
    }

    const std::string sourceLabel = useCamera
        ? ("camera:" + std::to_string(cameraIndex))
        : videoPath;

    try {
        // ── Install SIGBUS handler ─────────────────────────────────────
        // Catches unaligned access / mmap page faults on ARM64.
        // Prints which step was in progress before crashing.
        std::signal(SIGBUS, sigbusHandler);
        std::cout << "[TRACE] SIGBUS handler installed" << std::flush << std::endl;

        // ── OpenCV threading configuration ──────────────────────────────
        // cv::setNumThreads(0) = let TBB decide (uses all available cores).
        // This unlocks KleidiCV HAL multi-threaded dispatch for cv::resize,
        // cv::cvtColor, cv::GaussianBlur etc.  The old value of 1 was
        // crippling the TBB backend to a single core.
        cv::setNumThreads(0);
        cv::ocl::setUseOpenCL(false);

        // Pin UI/main thread to Core 3 (Linux/Raspberry Pi)
        pinCurrentThreadToCore(3);

        // ── Shared ov::Core (heap-allocated for guaranteed alignment) ───
        // std::make_shared ensures the ov::Core object lives on the heap
        // with proper alignment, avoiding potential stack alignment issues
        // on ARM64.  Also avoids duplicate plugin discovery (~50–100 ms
        // saved on Pi 4).
        g_currentStep = "ov::Core construction";
        std::cout << "[TRACE] >>> ov::Core construction BEGIN (heap via make_shared)" << std::flush << std::endl;
        auto corePtr = std::make_shared<ov::Core>();
        std::cout << "[TRACE] <<< ov::Core construction END (success, addr=" << static_cast<void*>(corePtr.get()) << ")" << std::flush << std::endl;
        ov::Core& core = *corePtr;
        const std::string device = "CPU";

        // ── Force FP32 precision ────────────────────────────────────────
        // Prevents FP16 alignment traps on Cortex-A72 (Pi 4).
        // Some OpenVINO builds default to FP16 on ARM which can trigger
        // SIGBUS on unaligned 16-bit float access.
        g_currentStep = "Setting FP32 inference precision";
        std::cout << "[TRACE] >>> Setting inference_precision to FP32" << std::flush << std::endl;
        try {
            core.set_property("CPU", ov::hint::inference_precision(ov::element::f32));
            std::cout << "[TRACE] <<< inference_precision(FP32) set successfully" << std::flush << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[TRACE] WARNING: Could not set FP32 precision: " << e.what() << std::flush << std::endl;
        }

        // ── Disable mmap at ov::Core level ──────────────────────────────
        // Prevents SIGBUS from SD-card mmap page faults.
        g_currentStep = "Disabling mmap on shared ov::Core";
        std::cout << "[TRACE] >>> Disabling mmap on shared ov::Core" << std::flush << std::endl;
        try {
            core.set_property("CPU", ov::enable_mmap(false));
            std::cout << "[TRACE] <<< ov::enable_mmap(false) set on shared core" << std::flush << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[TRACE] WARNING: Could not disable mmap on shared core: " << e.what() << std::flush << std::endl;
        }

        g_currentStep = "logDeviceCapabilities";
        std::cout << "[TRACE] >>> logDeviceCapabilities BEGIN" << std::flush << std::endl;
        logDeviceCapabilities(core, device);
        std::cout << "[TRACE] <<< logDeviceCapabilities END" << std::flush << std::endl;

        InstrumentationBus bus;
        std::unique_ptr<InstrumentedPerceptionAgent> perceptionHolder;
        g_currentStep = "PerceptionAgent construction (YOLO26 model load + compile)";
        std::cout << "[TRACE] >>> PerceptionAgent construction BEGIN (model=" << yoloModel << ")" << std::flush << std::endl;
        try {
            if (useCamera)
                perceptionHolder = std::make_unique<InstrumentedPerceptionAgent>(
                    core, yoloModel, device, cameraIndex, bus);
            else
                perceptionHolder = std::make_unique<InstrumentedPerceptionAgent>(
                    core, yoloModel, device, videoPath, bus);
        } catch (const ov::Exception& e) {
            std::cerr << "[TRACE] !!! ov::Exception during PerceptionAgent construction: " << e.what() << std::flush << std::endl;
            return 1;
        } catch (const std::exception& e) {
            std::cerr << "[TRACE] !!! std::exception during PerceptionAgent construction: " << e.what() << std::flush << std::endl;
            return 1;
        }

        const bool perceptionOk = perceptionHolder && perceptionHolder->isModelLoaded();
        if (perceptionOk) {
            std::cout << "[TRACE] <<< PerceptionAgent construction END (model compiled OK)" << std::flush << std::endl;
        } else {
            std::cerr << "[TRACE] <<< PerceptionAgent construction END (DEGRADED — model NOT compiled)\n"
                      << "[TRACE]     Perception inference will return empty detections.\n"
                      << "[TRACE]     Rebuild OpenVINO with ACL to fix." << std::flush << std::endl;
        }

        InstrumentedPerceptionAgent& perception = *perceptionHolder;
        g_currentStep = "AnalyticalAgent construction (MiDaS model load + compile)";
        std::cout << "[TRACE] >>> AnalyticalAgent construction BEGIN (model=" << midasModel << ")" << std::flush << std::endl;
        std::unique_ptr<InstrumentedAnalyticalAgent> analyticalHolder;
        try {
            analyticalHolder = std::make_unique<InstrumentedAnalyticalAgent>(
                core, midasModel, device, bus, perception);
        } catch (const ov::Exception& e) {
            std::cerr << "[TRACE] !!! ov::Exception during AnalyticalAgent construction: " << e.what() << std::flush << std::endl;
            return 1;
        } catch (const std::exception& e) {
            std::cerr << "[TRACE] !!! std::exception during AnalyticalAgent construction: " << e.what() << std::flush << std::endl;
            return 1;
        }

        const bool analyticalOk = analyticalHolder && analyticalHolder->isModelLoaded();
        if (analyticalOk) {
            std::cout << "[TRACE] <<< AnalyticalAgent construction END (model compiled OK)" << std::flush << std::endl;
        } else {
            std::cerr << "[TRACE] <<< AnalyticalAgent construction END (DEGRADED — model NOT compiled)\n"
                      << "[TRACE]     Depth estimation will return empty cv::Mat.\n"
                      << "[TRACE]     Rebuild OpenVINO with ACL to fix." << std::flush << std::endl;
        }

        // Print degraded-mode summary
        if (!perceptionOk || !analyticalOk) {
            std::cerr << "\n"
                      << "╔══════════════════════════════════════════════════════════════╗\n"
                      << "║               ⚠  DEGRADED MODE — ACL MISSING  ⚠            ║\n"
                      << "╠══════════════════════════════════════════════════════════════╣\n"
                      << "║  Perception (YOLO26) : " << (perceptionOk  ? "OK " : "OFF") << "                                    ║\n"
                      << "║  Analytical (MiDaS)  : " << (analyticalOk  ? "OK " : "OFF") << "                                    ║\n"
                      << "╠══════════════════════════════════════════════════════════════╣\n"
                      << "║  The pre-compiled OpenVINO archive does not include the     ║\n"
                      << "║  Arm Compute Library (ACL). Without ACL, the reference      ║\n"
                      << "║  CPU plugin emits unaligned accesses → SIGBUS on ARM64.     ║\n"
                      << "║                                                              ║\n"
                      << "║  Fix: rebuild OpenVINO from source:                         ║\n"
                      << "║    cmake -DENABLE_KLEIDIAI=ON ..                            ║\n"
                      << "║  See CONTRIBUTING.md Option B for full instructions.        ║\n"
                      << "╚══════════════════════════════════════════════════════════════╝\n"
                      << std::flush << std::endl;
        }

        g_currentStep = "Post-construction (coordinator + main loop)";
        InstrumentedAnalyticalAgent& analytical = *analyticalHolder;
        InstrumentedTemporalAnalyzer temporal(bus, perception);
        InstrumentedFusionEngine fusion(bus, perception);

        Coordinator coordinator(perception, analytical, temporal, fusion, targetFps);
        coordinator.start();

        // ── Performance tracker ─────────────────────────────────────────
        PerfTracker perf;

        bool shouldQuit = false;
        bool paused = false;

        cv::Mat currentDetectionsCanvas;
        cv::Mat lastDepthCanvas;
        cv::Mat lastPotholeCanvas;
        double lastSmoothedFps = 0.0;
        double lastAvgLatency = 0.0;
        int lastObservedStride = 0;
        float lastMaxConfidence = 0.0f;
        bool lastDepthValid = false;
        bool lastHazardTriggered = false;
        double lastFusionPeak = 0.0;
        double lastYoloPeak = 0.0;
        double lastCpuTemp = 0.0;

        std::deque<std::string> potholeLog;
        constexpr std::size_t MAX_EVENT_LOG = 24;

        const cv::Size dashboardSize(kDashboardWidth, kDashboardHeight);
        const int headerHeight = 72;
        const int panelHeaderHeight = 52;
        const int margin = 24;
        const int gap = 20;
        const int contentWidth = dashboardSize.width - margin * 2;
        const int contentHeight = dashboardSize.height - headerHeight - margin * 2;
        const int topRowHeight = (contentHeight - gap) / 2;
        const int bottomRowHeight = contentHeight - topRowHeight - gap;
        const int topPanelWidth = (contentWidth - 2 * gap) / 3;
        const int bottomPanelWidth = (contentWidth - gap) / 2;

        const cv::Rect headerRect(0, 0, dashboardSize.width, headerHeight);
        const int topY = headerHeight + margin;
        const int bottomY = topY + topRowHeight + gap;

        const cv::Rect detRect(margin, topY, topPanelWidth, topRowHeight);
        const cv::Rect depthRect(margin + topPanelWidth + gap, topY, topPanelWidth, topRowHeight);
        const cv::Rect snapshotRect(margin + 2 * (topPanelWidth + gap), topY, topPanelWidth, topRowHeight);
        const cv::Rect insightsRect(margin, bottomY, bottomPanelWidth, bottomRowHeight);
        const cv::Rect logRect(margin + bottomPanelWidth + gap, bottomY, bottomPanelWidth, bottomRowHeight);

        const cv::Scalar backgroundColor(18, 18, 26);
        const cv::Scalar panelColor(30, 28, 44);
        const cv::Scalar panelOverlay(48, 46, 68);
        const cv::Scalar headerColor(42, 40, 64);
        const cv::Scalar accentColor(186, 236, 255);
        const cv::Scalar subtitleColor(140, 200, 220);
        const cv::Scalar borderColor(58, 82, 128);
        const cv::Scalar textColor(210, 238, 242);
        const cv::Scalar warningColor(112, 190, 255);
        const cv::Scalar hazardAccent(32, 92, 240);
        const cv::Scalar okAccent(120, 232, 188);

        auto appendPotholeLog = [&](const FrameSnapshot& snap) {
            if (snap.fusions.empty())
                return;
            const auto now = std::chrono::system_clock::now();
            const std::time_t tt = std::chrono::system_clock::to_time_t(now);
            const std::tm tm = *std::localtime(&tt);
            for (const auto& tele : snap.fusions) {
                std::ostringstream line;
                line << '[' << std::put_time(&tm, "%H:%M:%S") << "] | "
                     << "Conf: " << std::fixed << std::setprecision(2) << tele.input.yoloConfidence
                     << " | Fusion: " << std::fixed << std::setprecision(2) << tele.output.finalConfidence
                     << " | Status: " << (tele.output.finalConfidence >= HAZARD_THRESHOLD ? "HAZARD" : "SAFE");
                potholeLog.push_back(line.str());
            }
            while (potholeLog.size() > MAX_EVENT_LOG)
                potholeLog.pop_front();
        };

        auto renderTextBlock = [panelHeaderHeight](cv::Mat& panel,
                                                  const std::vector<std::string>& lines,
                                                  const cv::Scalar& color,
                                                  int startY = -1,
                                                  int lineStep = 26) {
            int baseY = startY >= 0 ? startY : panelHeaderHeight + 32;
            for (std::size_t i = 0; i < lines.size(); ++i) {
                cv::putText(panel,
                            lines[i],
                            cv::Point(20, baseY + static_cast<int>(i) * lineStep),
                            cv::FONT_HERSHEY_DUPLEX,
                            0.55,
                            color,
                            1,
                            cv::LINE_8);
            }
        };

        auto formatDouble = [](double value) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << value;
            return oss.str();
        };

        cv::namedWindow("VIGIA Dashboard", cv::WINDOW_NORMAL);
        cv::resizeWindow("VIGIA Dashboard", dashboardSize.width, dashboardSize.height);

        auto lastRenderTs = std::chrono::steady_clock::now();
        bool needsRender = true;

        while (!shouldQuit) {
            FrameSnapshot snapshot;
            if (bus.tryPopFrame(snapshot)) {
                needsRender = true;

                // ── Feed PerfTracker ────────────────────────────────────
                perf.recordFrame(snapshot.latencyMs);
                lastSmoothedFps = perf.smoothedFps();
                lastAvgLatency  = perf.avgLatencyMs();

                // Print time-to-first-inference once
                if (perf.frames() == 1) {
                    std::cout << "[vigia-test] Time to first inference result: "
                              << std::fixed << std::setprecision(1)
                              << perf.firstInferMs() << " ms\n";
                }

                lastObservedStride = snapshot.observedStride;
                lastMaxConfidence = snapshot.maxConfidence;
                lastDepthValid = snapshot.depthValid && !snapshot.depthMap.empty();
                lastHazardTriggered = false;
                lastFusionPeak = 0.0;
                lastYoloPeak = 0.0;

                for (const auto& tele : snapshot.fusions) {
                    lastFusionPeak = std::max(lastFusionPeak, static_cast<double>(tele.output.finalConfidence));
                    lastYoloPeak = std::max(lastYoloPeak, static_cast<double>(tele.input.yoloConfidence));
                    if (tele.output.finalConfidence >= HAZARD_THRESHOLD)
                        lastHazardTriggered = true;
                }

                // Throttle CPU temperature reads to once per second
                {
                    static auto lastTempReadTs = std::chrono::steady_clock::now() - std::chrono::seconds(2);
                    const auto tempNow = std::chrono::steady_clock::now();
                    if (tempNow - lastTempReadTs >= std::chrono::seconds(1)) {
                        lastCpuTemp = static_cast<double>(readCPUTemperature());
                        lastTempReadTs = tempNow;
                    }
                }

                // Deep-copy the base frame ONCE, then draw detections in-place.
                // Replaces the original pattern of cloning inside drawDetections()
                // every call — saving one full-frame memcpy per pop.
                if (!snapshot.frameBgr.empty()) {
                    currentDetectionsCanvas = snapshot.frameBgr.clone();
                    drawDetections(currentDetectionsCanvas, snapshot);
                    if (!snapshot.fusions.empty())
                        lastPotholeCanvas = currentDetectionsCanvas;
                }

                if constexpr (!kArmProfile) {
                    cv::Mat depthCanvas = makeDepthVisualization(snapshot);
                    if (!depthCanvas.empty())
                        lastDepthCanvas = depthCanvas;
                }

                appendPotholeLog(snapshot);

                if constexpr (!kArmProfile) {
                    std::cout << std::fixed << std::setprecision(2)
                              << "[FRAME " << snapshot.frameIndex << "] latency=" << snapshot.latencyMs
                              << "ms stride=" << snapshot.observedStride
                              << " fps=" << lastSmoothedFps
                              << " detections=" << snapshot.fusions.size()
                              << " maxY=" << snapshot.maxConfidence
                              << '\n';
                }
            } else if (perception.isFinished()) {
                shouldQuit = true;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(kIdleBackoffMs));
            }

            // Render-throttle: cap dashboard rebuilds to ~15 FPS
            const auto renderNow = std::chrono::steady_clock::now();
            const bool renderDue = needsRender &&
                (renderNow - lastRenderTs >= std::chrono::milliseconds(kRenderIntervalMs));

            if (renderDue) {

            if constexpr (kArmProfile) {
                // ── Lightweight ARM display: detection canvas + minimal HUD ──
                if (!currentDetectionsCanvas.empty()) {
                    cv::Mat& display = currentDetectionsCanvas;
                    // Dark HUD bar for readability
                    if (display.rows > 52) {
                        cv::Mat hudBar = display(cv::Rect(0, 0, display.cols, 52));
                        hudBar.setTo(cv::Scalar(18, 18, 26));
                    }
                    const std::string hud =
                        "FPS " + formatDouble(lastSmoothedFps) +
                        "  |  Lat " + formatDouble(lastAvgLatency) + "ms"
                        "  |  Stride " + std::to_string(lastObservedStride) +
                        "  |  " + (lastHazardTriggered ? "HAZARD" : "OK");
                    cv::putText(display, hud, cv::Point(12, 20),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                lastHazardTriggered ? cv::Scalar(64, 64, 240)
                                                    : cv::Scalar(200, 230, 200),
                                1, cv::LINE_8);
                    const std::string detLine =
                        "Det " + std::to_string(static_cast<int>(lastMaxConfidence > 0)) +
                        "  |  MaxY " + formatDouble(static_cast<double>(lastMaxConfidence)) +
                        "  |  Fusion " + formatDouble(lastFusionPeak) +
                        "  |  " + sourceLabel;
                    cv::putText(display, detLine, cv::Point(12, 42),
                                cv::FONT_HERSHEY_SIMPLEX, 0.45,
                                cv::Scalar(140, 200, 220),
                                1, cv::LINE_8);
                    if (paused) {
                        cv::putText(display, "PAUSED",
                                    cv::Point(display.cols / 2 - 60, display.rows / 2),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                                    cv::Scalar(112, 190, 255), 2, cv::LINE_8);
                    }
                    cv::imshow("VIGIA Dashboard", display);
                }
            } else {
            // ── Full desktop 5-panel dashboard ──
            cv::Mat dashboard(dashboardSize, CV_8UC3);
            dashboard.setTo(backgroundColor);

            cv::rectangle(dashboard, headerRect, headerColor, cv::FILLED);
            cv::circle(dashboard, cv::Point(margin + 18, headerHeight / 2), 7, cv::Scalar(82, 82, 230), cv::FILLED);
            cv::circle(dashboard, cv::Point(margin + 38, headerHeight / 2), 7, cv::Scalar(82, 198, 240), cv::FILLED);
            cv::circle(dashboard, cv::Point(margin + 58, headerHeight / 2), 7, cv::Scalar(120, 220, 160), cv::FILLED);

            std::ostringstream headerLabel;
            headerLabel << "VIGIA RasPi Module";
            cv::putText(dashboard,
                        headerLabel.str(),
                        cv::Point(margin + 90, headerHeight - 20),
                        cv::FONT_HERSHEY_DUPLEX,
                        0.8,
                        accentColor,
                        1,
                        cv::LINE_8);

            const auto headerNow = std::chrono::system_clock::now();
            const std::time_t headerT = std::chrono::system_clock::to_time_t(headerNow);
            const std::tm headerTm = *std::localtime(&headerT);
            std::ostringstream headerTime;
            headerTime << std::put_time(&headerTm, "%b %d  %H:%M");
            cv::putText(dashboard,
                        headerTime.str(),
                        cv::Point(dashboardSize.width - 200, headerHeight - 20),
                        cv::FONT_HERSHEY_DUPLEX,
                        0.6,
                        subtitleColor,
                        1,
                        cv::LINE_8);

            auto setupPanel = [&](const cv::Rect& roi, const std::string& title) {
                cv::Mat panel = dashboard(roi);
                panel.setTo(panelColor);
                if (panel.rows > panelHeaderHeight) {
                    cv::Mat panelHeader = panel(cv::Rect(0, 0, panel.cols, panelHeaderHeight));
                    panelHeader.setTo(panelOverlay);
                }
                cv::putText(panel,
                            title,
                            cv::Point(22, panelHeaderHeight - 14),
                            cv::FONT_HERSHEY_DUPLEX,
                            0.7,
                            accentColor,
                            1,
                            cv::LINE_8);
                cv::line(panel,
                         cv::Point(0, panelHeaderHeight),
                         cv::Point(panel.cols, panelHeaderHeight),
                         borderColor,
                         1,
                         cv::LINE_8);
                cv::rectangle(dashboard, roi, borderColor, 1, cv::LINE_8);
                return panel;
            };

            // ── Detection Feed panel ──
            cv::Mat detPanel = setupPanel(detRect, "Detection Feed");
            const int detContentHeight = std::max(0, detPanel.rows - panelHeaderHeight);
            if (detContentHeight > 0) {
                cv::Mat detContent = detPanel(cv::Rect(0, panelHeaderHeight, detPanel.cols, detContentHeight));
                if (!currentDetectionsCanvas.empty()) {
                    // Resize directly into the dashboard sub-ROI — no intermediate
                    // allocation.  KleidiCV HAL accelerates cv::resize via NEON.
                    cv::resize(currentDetectionsCanvas, detContent, detContent.size());
                    if constexpr (!kArmProfile) {
                        cv::Mat tint(detContent.size(), detContent.type(), cv::Scalar(14, 14, 22));
                        cv::addWeighted(detContent, 0.88, tint, 0.12, 0.0, detContent);
                    }
                } else {
                    const int fallbackY = panelHeaderHeight + detContentHeight / 2;
                    renderTextBlock(detPanel,
                                    {"Awaiting detections..."},
                                    warningColor,
                                    fallbackY,
                                    28);
                }
            }

            const std::string hazardLabel = lastHazardTriggered ? "HAZARD" : "Nominal";
            const cv::Scalar hazardColorScalar = lastHazardTriggered ? cv::Scalar(64, 64, 240) : okAccent;
            cv::putText(detPanel,
                        hazardLabel,
                        cv::Point(detPanel.cols - 160, panelHeaderHeight - 14),
                        cv::FONT_HERSHEY_DUPLEX,
                        0.6,
                        hazardColorScalar,
                        1,
                        cv::LINE_8);

            if (detPanel.rows > panelHeaderHeight + 12) {
                const int statusHeight = std::min(60, detPanel.rows - panelHeaderHeight);
                cv::Rect statusRect(0, detPanel.rows - statusHeight, detPanel.cols, statusHeight);
                cv::Mat statusBar = detPanel(statusRect);
                statusBar.setTo(cv::Scalar(18, 18, 28));

                const std::string fpsLabel = "FPS " + (lastSmoothedFps > 0.0 ? formatDouble(lastSmoothedFps) : std::string("n/a"));
                const std::string latencyLabel = "Latency " + (lastAvgLatency > 0.0 ? formatDouble(lastAvgLatency) + " ms" : std::string("n/a"));
                const std::string srcLabel = "Source " + sourceLabel;

                cv::putText(detPanel,
                            srcLabel,
                            cv::Point(24, detPanel.rows - 24),
                            cv::FONT_HERSHEY_DUPLEX,
                            0.5,
                            subtitleColor,
                            1,
                            cv::LINE_8);
                cv::putText(detPanel,
                            fpsLabel,
                            cv::Point(detPanel.cols / 2 - 80, detPanel.rows - 24),
                            cv::FONT_HERSHEY_DUPLEX,
                            0.5,
                            subtitleColor,
                            1,
                            cv::LINE_8);
                cv::putText(detPanel,
                            latencyLabel,
                            cv::Point(detPanel.cols - 220, detPanel.rows - 24),
                            cv::FONT_HERSHEY_DUPLEX,
                            0.5,
                            subtitleColor,
                            1,
                            cv::LINE_8);
            }

            if (paused && detContentHeight > 0) {
                const int overlayWidth = std::min(280, detPanel.cols - 40);
                const int overlayHeight = 80;
                const int overlayX = (detPanel.cols - overlayWidth) / 2;
                const int overlayY = panelHeaderHeight + detContentHeight / 2 - overlayHeight / 2;
                cv::rectangle(detPanel,
                              cv::Rect(overlayX, overlayY, overlayWidth, overlayHeight),
                              cv::Scalar(16, 18, 26),
                              cv::FILLED);
                cv::putText(detPanel,
                            "PAUSED",
                            cv::Point(overlayX + 40, overlayY + overlayHeight / 2 + 12),
                            cv::FONT_HERSHEY_DUPLEX,
                            0.75,
                            warningColor,
                            1,
                            cv::LINE_8);
            }

            // ── Depth Map panel ──
            cv::Mat depthPanel = setupPanel(depthRect, lastDepthValid ? "Depth Map" : "Depth Map (pending)");
            const int depthContentHeight = std::max(0, depthPanel.rows - panelHeaderHeight);
            if (depthContentHeight > 0) {
                cv::Mat depthContent = depthPanel(cv::Rect(0, panelHeaderHeight, depthPanel.cols, depthContentHeight));
                if (!lastDepthCanvas.empty()) {
                    // Resize directly into dashboard sub-ROI — zero intermediate alloc
                    cv::resize(lastDepthCanvas, depthContent, depthContent.size());
                } else {
                    const int fallbackY = panelHeaderHeight + depthContentHeight / 2;
                    renderTextBlock(depthPanel,
                                    {"Depth data pending"},
                                    warningColor,
                                    fallbackY,
                                    26);
                }
            }

            // ── Last Pothole Frame panel ──
            cv::Mat snapshotPanel = setupPanel(snapshotRect, "Last Pothole Frame");
            const int snapshotContentHeight = std::max(0, snapshotPanel.rows - panelHeaderHeight);
            if (snapshotContentHeight > 0) {
                cv::Mat snapshotContent = snapshotPanel(cv::Rect(0, panelHeaderHeight, snapshotPanel.cols, snapshotContentHeight));
                if (!lastPotholeCanvas.empty()) {
                    // Resize directly into dashboard sub-ROI — zero intermediate alloc
                    cv::resize(lastPotholeCanvas, snapshotContent, snapshotContent.size());
                    if constexpr (!kArmProfile) {
                        cv::Mat tint(snapshotContent.size(), snapshotContent.type(), cv::Scalar(18, 18, 32));
                        cv::addWeighted(snapshotContent, 0.9, tint, 0.1, 0.0, snapshotContent);
                    }
                } else {
                    const int placeholderY = panelHeaderHeight + snapshotContentHeight / 2;
                    renderTextBlock(snapshotPanel,
                                    {"No pothole frame recorded"},
                                    warningColor,
                                    placeholderY,
                                    26);
                }
            }

            // ── Operational Insights panel (with PerfTracker metrics) ──
            cv::Mat insightsPanel = setupPanel(insightsRect, "Operational Insights");
            std::vector<std::string> insightLines;
            insightLines.emplace_back("Latency avg: " + (lastAvgLatency > 0.0 ? formatDouble(lastAvgLatency) + " ms" : std::string("n/a")));
            insightLines.emplace_back("Latency min: " + (perf.frames() > 0 ? formatDouble(perf.minLatencyMs()) + " ms" : std::string("n/a")));
            insightLines.emplace_back("Latency max: " + (perf.frames() > 0 ? formatDouble(perf.maxLatencyMs()) + " ms" : std::string("n/a")));
            insightLines.emplace_back("FPS (EMA): " + (lastSmoothedFps > 0.0 ? formatDouble(lastSmoothedFps) : std::string("n/a")));
            insightLines.emplace_back("MiDaS stride: " + std::to_string(lastObservedStride));
            insightLines.emplace_back("YOLO max: " + formatDouble(static_cast<double>(lastMaxConfidence)));
            insightLines.emplace_back("Fusion peak: " + formatDouble(lastFusionPeak));
            insightLines.emplace_back("Hazard state: " + std::string(lastHazardTriggered ? "ACTIVE" : "nominal"));
            insightLines.emplace_back("CPU temp: " + (std::isnan(lastCpuTemp) ? std::string("n/a") : formatDouble(lastCpuTemp) + " C"));
            insightLines.emplace_back("1st infer: " + formatDouble(perf.firstInferMs()) + " ms");
            renderTextBlock(insightsPanel, insightLines, textColor, panelHeaderHeight + 32, 26);
            if (lastHazardTriggered) {
                cv::putText(insightsPanel,
                            "HAZARD DETECTED",
                            cv::Point(22, insightsPanel.rows - 32),
                            cv::FONT_HERSHEY_DUPLEX,
                            0.64,
                            hazardAccent,
                            1,
                            cv::LINE_8);
            }

            // ── Pothole Event Log panel ──
            cv::Mat logPanel = setupPanel(logRect, "Pothole Event Log");
            const int maxVisibleEvents = std::max(1, (logRect.height - (panelHeaderHeight + 28)) / 22);
            std::vector<std::string> logLines;
            logLines.reserve(static_cast<std::size_t>(maxVisibleEvents));
            if (potholeLog.empty()) {
                logLines.emplace_back("No pothole events yet");
            } else {
                int added = 0;
                for (auto it = potholeLog.rbegin(); it != potholeLog.rend() && added < maxVisibleEvents; ++it, ++added)
                    logLines.push_back(*it);
            }
            renderTextBlock(logPanel, logLines, subtitleColor, panelHeaderHeight + 32, 22);

            cv::imshow("VIGIA Dashboard", dashboard);
            } // desktop dashboard

            lastRenderTs = renderNow;
            needsRender = false;
            } // render-throttle gate

            const int delay = paused ? 50 : 1;
            const int key = cv::waitKey(delay);
            if (key < 0)
                continue;

            switch (key) {
            case 'q':
            case 'Q':
                shouldQuit = true;
                break;
            case ' ':
                perception.togglePause();
                paused = perception.isPaused();
                break;
            case 's':
            case 'S':
                if (perception.isPaused()) {
                    perception.singleStep();
                    paused = true;
                }
                break;
            default:
                break;
            }
        }

        perception.requestStop();
        coordinator.stop();
        bus.flush();

        cv::destroyAllWindows();

        const auto summary = bus.summary();
        std::cout << "\n===== System Visual Test Summary =====\n";
        std::cout << "Source: " << sourceLabel << "\n";
        std::cout << "Frames captured: " << summary.capturedFrames << "\n";
        std::cout << "Frames processed: " << summary.processedFrames << "\n";
        std::cout << "Frames skipped (buffer): " << summary.skippedFrames << "\n";
        std::cout << "MiDaS executions: " << summary.depthExecutions << "\n";
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Average latency: " << summary.averageLatencyMs << " ms\n";
        std::cout << "Average FPS: " << summary.averageFps << "\n";
        if (summary.processedFrames > 0)
            std::cout << "MiDaS frequency: "
                      << (static_cast<double>(summary.depthExecutions) /
                          static_cast<double>(summary.processedFrames))
                      << "\n";
        std::cout << "──────── Performance Metrics ────────\n";
        std::cout << "FPS (EMA-smoothed): " << perf.smoothedFps() << "\n";
        std::cout << "Latency avg: " << perf.avgLatencyMs() << " ms\n";
        std::cout << "Latency min: " << perf.minLatencyMs() << " ms\n";
        std::cout << "Latency max: " << perf.maxLatencyMs() << " ms\n";
        std::cout << "Time to 1st inference: " << perf.firstInferMs() << " ms\n";
        std::cout << "======================================\n";

    } catch (const std::exception& ex) {
        std::cerr << "[system_visual_test] " << ex.what() << "\n";
        return 1;
    }

    return 0;
}

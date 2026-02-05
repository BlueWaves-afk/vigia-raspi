#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
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
#include <unordered_map>
#include <vector>
#include <ctime>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

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
static std::atomic<std::uint64_t> g_activeFrameIndex{0};
}

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

class InstrumentationBus {
public:
    InstrumentationBus() = default;

    void beginFrame(std::uint64_t frameIndex, const cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(mutex_);

        FrameSlot& slot = slots_[frameIndex];
        slot.frameIndex = frameIndex;
        slot.frameBgr = frame.clone();
        slot.startTs = std::chrono::steady_clock::now();
        slot.totalDetections = 0;
        slot.filteredDetections.clear();
        slot.temporalStaging.clear();
        slot.fusionTelemetry.clear();
        slot.depthMap.release();
        slot.depthValid = false;
        slot.observedStride = lastObservedStride_;
        slot.maxConfidence = 0.0f;
        slot.completed = false;

        if (!firstFrameTs_.time_since_epoch().count())
            firstFrameTs_ = slot.startTs;

        capturedFrames_++;
    }

    void storeDetections(std::uint64_t frameIndex, const std::vector<Detection>& detections) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = slots_.find(frameIndex);
        if (it == slots_.end())
            return;

        FrameSlot& slot = it->second;
        slot.totalDetections = detections.size();
        slot.allDetections = detections;
        slot.filteredDetections.clear();
        slot.temporalStaging.clear();
        slot.fusionTelemetry.clear();
        slot.maxConfidence = 0.0f;

        std::cout << "[BUS] storeDetections frame=" << frameIndex
                  << " count=" << detections.size() << "\n";

        for (const auto& det : detections) {
            slot.maxConfidence = std::max(slot.maxConfidence, det.confidence);
            if (det.classId == POTHOLE_CLASS_ID)
                slot.filteredDetections.push_back(det);
        }

        if (slot.filteredDetections.empty()) {
            finalizeSlot(slot);
            ready_.push_back(makeSnapshot(slot));
            slots_.erase(it);
        }
    }

    void recordDepth(std::uint64_t frameIndex, const cv::Mat& depth) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = slots_.find(frameIndex);
        if (it == slots_.end())
            return;

        FrameSlot& slot = it->second;
        slot.depthMap = depth.clone();
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
        auto it = slots_.find(frameIndex);
        if (it == slots_.end())
            return;

        it->second.temporalStaging.push_back(metrics);
    }

    void recordFusion(std::uint64_t frameIndex,
                      const FusionInput& input,
                      const FusionOutput& output) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = slots_.find(frameIndex);
        if (it == slots_.end())
            return;

        FrameSlot& slot = it->second;
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
            slots_.erase(it);
        }
    }

    bool tryPopFrame(FrameSnapshot& snapshot) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (ready_.empty())
            return false;
        snapshot = ready_.front();
        ready_.pop_front();
        return true;
    }

    void flush() {
        std::lock_guard<std::mutex> lock(mutex_);
        finalizeOlderFrames(std::numeric_limits<std::uint64_t>::max());
        std::vector<std::uint64_t> remaining;
        remaining.reserve(slots_.size());
        for (const auto& kv : slots_)
            remaining.push_back(kv.first);
        for (auto idx : remaining) {
            FrameSlot& slot = slots_.at(idx);
            finalizeSlot(slot);
            ready_.push_back(makeSnapshot(slot));
            slots_.erase(idx);
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
        if (processedFrames_ > 1 && firstFrameTs_.time_since_epoch().count() &&
            lastFrameTs_.time_since_epoch().count()) {
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

    static FrameSnapshot makeSnapshot(const FrameSlot& slot) {
        FrameSnapshot snapshot;
        snapshot.frameIndex = slot.frameIndex;
        snapshot.latencyMs = slot.latencyMs;
        snapshot.observedStride = slot.observedStride;
        snapshot.frameBgr = slot.frameBgr;
        snapshot.depthMap = slot.depthMap;
        snapshot.depthValid = slot.depthValid;
        snapshot.fusions = slot.fusionTelemetry;
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
        if (lastFrameTs_.time_since_epoch().count())
            lastFrameTs_ = now;
        else
            lastFrameTs_ = now;
        if (lastFinalizedFrame_ != 0 && slot.frameIndex > lastFinalizedFrame_ + 1)
            skippedFrames_ += slot.frameIndex - lastFinalizedFrame_ - 1;
        lastFinalizedFrame_ = slot.frameIndex;
    }

    void finalizeOlderFrames(std::uint64_t currentFrameIndex) {
        std::vector<std::uint64_t> stale;
        stale.reserve(slots_.size());
        for (const auto& kv : slots_) {
            if (kv.first < currentFrameIndex && !kv.second.completed)
                stale.push_back(kv.first);
        }
        for (auto idx : stale) {
            FrameSlot& slot = slots_.at(idx);
            finalizeSlot(slot);
            ready_.push_back(makeSnapshot(slot));
            slots_.erase(idx);
        }
    }

    mutable std::mutex mutex_;
    std::unordered_map<std::uint64_t, FrameSlot> slots_;
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
};

class VideoPerceptionAgent : public PerceptionAgent {
public:
    VideoPerceptionAgent(const std::string& modelXmlPath,
                         const std::string& device,
                         std::string videoPath,
                         InstrumentationBus& bus,
                         int cameraIndex = 0)
        : PerceptionAgent(modelXmlPath, device, cameraIndex),
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

    VideoPerceptionAgent(const std::string& modelXmlPath,
                         const std::string& device,
                         InstrumentationBus& bus,
                         int cameraIndex)
        : PerceptionAgent(modelXmlPath, device, cameraIndex),
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
        if (lastCaptureTs_.time_since_epoch().count()) {
            const auto target = lastCaptureTs_ + frameInterval_;
            if (now < target)
                std::this_thread::sleep_for(target - now);
        }
        lastCaptureTs_ = std::chrono::steady_clock::now();
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
    std::chrono::duration<double> frameInterval_{};
    int cameraIndex_{0};
    bool useCamera_{false};
};

class InstrumentedPerceptionAgent final : public VideoPerceptionAgent {
public:
    InstrumentedPerceptionAgent(const std::string& modelXmlPath,
                                const std::string& device,
                                std::string videoPath,
                                InstrumentationBus& bus)
        : VideoPerceptionAgent(modelXmlPath, device, std::move(videoPath), bus) {}

    InstrumentedPerceptionAgent(const std::string& modelXmlPath,
                                const std::string& device,
                                int cameraIndex,
                                InstrumentationBus& bus)
        : VideoPerceptionAgent(modelXmlPath, device, bus, cameraIndex) {}

    bool captureFrame(cv::Mat& frame) override {
        if (!VideoPerceptionAgent::captureFrame(frame))
            return false;

        const std::uint64_t frameIndex = deliveredFrameIndex();
        {
            std::lock_guard<std::mutex> lock(frameIndexMutex_);
            pendingFrameIndices_.push_back(frameIndex);
        }

        static std::atomic<int> debugCaptureCount{0};
        if (debugCaptureCount.fetch_add(1) < 5)
            std::cout << "[CAPTURE] enqueued frame=" << frameIndex << "\n";

        return true;
    }

    std::vector<Detection> runInference(const cv::Mat& frame) override {
        std::uint64_t frameIndex = 0;
        {
            std::lock_guard<std::mutex> lock(frameIndexMutex_);
            if (!pendingFrameIndices_.empty()) {
                frameIndex = pendingFrameIndices_.back();
                pendingFrameIndices_.clear(); // drop stale indices to stay aligned with latest frame
            }
        }

        if (frameIndex == 0)
            frameIndex = deliveredFrameIndex();

        static std::atomic<int> debugInferenceCount{0};
        if (debugInferenceCount.fetch_add(1) < 5)
            std::cout << "[INFER] using frame=" << frameIndex << "\n";

        g_activeFrameIndex.store(frameIndex, std::memory_order_release);

        auto detections = PerceptionAgent::runInference(frame);

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

        if (frameIndex != 0)
            bus().storeDetections(frameIndex, detections);
        return detections;
    }
private:
    std::mutex frameIndexMutex_;
    std::vector<std::uint64_t> pendingFrameIndices_;
};

class InstrumentedAnalyticalAgent final : public AnalyticalAgent {
public:
    InstrumentedAnalyticalAgent(const std::string& modelXmlPath,
                                const std::string& device,
                                InstrumentationBus& bus)
        : AnalyticalAgent(modelXmlPath, device), bus_(bus) {}

    cv::Mat runInference(const cv::Mat& frame) override {
        cv::Mat depth = AnalyticalAgent::runInference(frame);
        const std::uint64_t frameIndex = g_activeFrameIndex.load(std::memory_order_acquire);
        if (frameIndex != 0)
            bus_.recordDepth(frameIndex, depth);
        return depth;
    }

private:
    InstrumentationBus& bus_;
};

class InstrumentedTemporalAnalyzer final : public TemporalAnalyzer {
public:
    explicit InstrumentedTemporalAnalyzer(InstrumentationBus& bus)
        : TemporalAnalyzer(), bus_(bus) {}

    TemporalMetrics update(float depressionScore, float roughness) override {
        TemporalMetrics metrics = TemporalAnalyzer::update(depressionScore, roughness);
        const std::uint64_t frameIndex = g_activeFrameIndex.load(std::memory_order_acquire);
        if (frameIndex != 0)
            bus_.recordTemporal(frameIndex, metrics);
        return metrics;
    }

private:
    InstrumentationBus& bus_;
};

class InstrumentedFusionEngine final : public FusionEngine {
public:
    explicit InstrumentedFusionEngine(InstrumentationBus& bus)
        : FusionEngine(), bus_(bus) {}

    FusionOutput fuse(const FusionInput& input) const override {
        FusionOutput output = FusionEngine::fuse(input);
        const std::uint64_t frameIndex = g_activeFrameIndex.load(std::memory_order_acquire);
        if (frameIndex != 0)
            bus_.recordFusion(frameIndex, input, output);
        return output;
    }

private:
    InstrumentationBus& bus_;
};

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

cv::Mat drawDetections(const FrameSnapshot& snapshot) {
    cv::Mat canvas = snapshot.frameBgr.empty() ? cv::Mat{} : snapshot.frameBgr.clone();
    if (canvas.empty())
        return canvas;

    for (const auto& tele : snapshot.fusions) {
        const cv::Rect& box = tele.detection.boundingBox;
        const cv::Scalar boxColor(0, 165, 255);
        cv::rectangle(canvas, box, boxColor, 2);

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
                    cv::LINE_AA);

        const bool hazard = tele.output.finalConfidence >= HAZARD_THRESHOLD;
        cv::putText(canvas,
                    hazard ? "HAZARD" : "SAFE",
                    cv::Point(box.x, box.y + box.height + 18),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.55,
                    hazard ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 200, 0),
                    2,
                    cv::LINE_AA);
    }

    return canvas;
}

cv::Mat makeDepthVisualization(const FrameSnapshot& snapshot) {
    if (!snapshot.depthValid || snapshot.depthMap.empty())
        return {};

    cv::Mat depthNorm;
    cv::normalize(snapshot.depthMap, depthNorm, 0.0, 255.0, cv::NORM_MINMAX);
    depthNorm.convertTo(depthNorm, CV_8U);
    cv::Mat depthColor;
    cv::applyColorMap(depthNorm, depthColor, cv::COLORMAP_INFERNO);
    return depthColor;
}

cv::Mat buildInsightsCanvas(const FrameSnapshot& snapshot,
                            double smoothedFps,
                            double avgLatencyMs,
                            float cpuTempC) {
    cv::Mat canvas(420, 600, CV_8UC3, cv::Scalar::all(0));
    int y = 32;
    const auto putLine = [&](const std::string& text, double scale = 0.6) {
        cv::putText(canvas,
                    text,
                    cv::Point(20, y),
                    cv::FONT_HERSHEY_SIMPLEX,
                    scale,
                    cv::Scalar(220, 220, 220),
                    1,
                    cv::LINE_AA);
        y += static_cast<int>(30 * scale) + 4;
    };

    std::ostringstream header;
    header << "Frame " << snapshot.frameIndex;
    putLine(header.str(), 0.7);

    std::ostringstream latencyLine;
    latencyLine << std::fixed << std::setprecision(2)
                << "Latency: " << snapshot.latencyMs << " ms";
    putLine(latencyLine.str());

    std::ostringstream fpsLine;
    fpsLine << std::fixed << std::setprecision(2)
            << "FPS (smooth): " << smoothedFps;
    putLine(fpsLine.str());

    std::ostringstream avgLine;
    avgLine << std::fixed << std::setprecision(2)
            << "Latency (avg): " << avgLatencyMs << " ms";
    putLine(avgLine.str());

    std::ostringstream strideLine;
    strideLine << "MiDaS stride: " << snapshot.observedStride;
    putLine(strideLine.str());

    if (!std::isnan(cpuTempC)) {
        std::ostringstream tempLine;
        tempLine << std::fixed << std::setprecision(2)
                 << "CPU temp: " << cpuTempC << " C";
        putLine(tempLine.str());
    } else {
        putLine("CPU temp: n/a");
    }

    std::ostringstream detLine;
    detLine << "Detections: " << snapshot.totalDetections
            << " | Potholes: " << snapshot.fusions.size();
    putLine(detLine.str());

    std::size_t idx = 0;
    bool hazard = false;
    for (const auto& tele : snapshot.fusions) {
        std::ostringstream line;
        line << "#" << idx++
             << " Y=" << std::fixed << std::setprecision(2) << tele.input.yoloConfidence
             << " G=" << tele.output.geometryConfidence
             << " T=" << tele.output.temporalConfidence
             << " F=" << tele.output.finalConfidence;
        putLine(line.str(), 0.55);
        hazard = hazard || tele.output.finalConfidence >= HAZARD_THRESHOLD;
    }

    putLine(std::string("Hazard decision: ") + (hazard ? "TRUE" : "FALSE"), 0.65);

    return canvas;
}

} // namespace viz
} // namespace vigia

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
        InstrumentationBus bus;
        std::unique_ptr<InstrumentedPerceptionAgent> perceptionHolder;
        if (useCamera)
            perceptionHolder = std::make_unique<InstrumentedPerceptionAgent>(yoloModel, "CPU", cameraIndex, bus);
        else
            perceptionHolder = std::make_unique<InstrumentedPerceptionAgent>(yoloModel, "CPU", videoPath, bus);

        InstrumentedPerceptionAgent& perception = *perceptionHolder;
        InstrumentedAnalyticalAgent analytical(midasModel, "CPU", bus);
        InstrumentedTemporalAnalyzer temporal(bus);
        InstrumentedFusionEngine fusion(bus);

        Coordinator coordinator(perception, analytical, temporal, fusion, targetFps);
        coordinator.start();

        std::deque<double> latencyHistory;
        constexpr std::size_t MAX_HISTORY = 32;
        double cumulativeLatency = 0.0;
        std::size_t latencySamples = 0;

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

        const cv::Size dashboardSize(1440, 900);
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
                            cv::LINE_AA);
            }
        };

        auto formatDouble = [](double value) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << value;
            return oss.str();
        };

        cv::namedWindow("VIGIA Dashboard", cv::WINDOW_NORMAL);
        cv::resizeWindow("VIGIA Dashboard", dashboardSize.width, dashboardSize.height);

        while (!shouldQuit) {
            FrameSnapshot snapshot;
            bool updatedFrame = false;
            if (bus.tryPopFrame(snapshot)) {
                updatedFrame = true;
                const double latency = snapshot.latencyMs;
                latencyHistory.push_back(latency);
                if (latencyHistory.size() > MAX_HISTORY)
                    latencyHistory.pop_front();

                double latencyAvgWindow = std::accumulate(latencyHistory.begin(), latencyHistory.end(), 0.0) /
                                          static_cast<double>(latencyHistory.size());
                const double smoothedFps = latencyAvgWindow > 0.0 ? 1000.0 / latencyAvgWindow : 0.0;

                cumulativeLatency += latency;
                latencySamples++;
                const double avgLatency = latencySamples > 0
                    ? cumulativeLatency / static_cast<double>(latencySamples)
                    : latency;

                lastSmoothedFps = smoothedFps;
                lastAvgLatency = avgLatency;
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

                lastCpuTemp = static_cast<double>(readCPUTemperature());

                cv::Mat detCanvas = drawDetections(snapshot);
                if (!detCanvas.empty()) {
                    currentDetectionsCanvas = detCanvas;
                    if (!snapshot.fusions.empty())
                        lastPotholeCanvas = detCanvas.clone();
                }

                cv::Mat depthCanvas = makeDepthVisualization(snapshot);
                if (!depthCanvas.empty())
                    lastDepthCanvas = depthCanvas;

                appendPotholeLog(snapshot);

                std::cout << std::fixed << std::setprecision(2)
                          << "[FRAME " << snapshot.frameIndex << "] latency=" << snapshot.latencyMs
                          << "ms stride=" << snapshot.observedStride
                          << " fps=" << smoothedFps
                          << " detections=" << snapshot.fusions.size()
                          << " maxY=" << snapshot.maxConfidence
                          << '\n';
            } else if (perception.isFinished()) {
                shouldQuit = true;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(4));
            }

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
                        cv::LINE_AA);

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
                        cv::LINE_AA);

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
                            cv::LINE_AA);
                cv::line(panel,
                         cv::Point(0, panelHeaderHeight),
                         cv::Point(panel.cols, panelHeaderHeight),
                         borderColor,
                         1,
                         cv::LINE_AA);
                cv::rectangle(dashboard, roi, borderColor, 1, cv::LINE_AA);
                return panel;
            };

            cv::Mat detPanel = setupPanel(detRect, "Detection Feed");
            const int detContentHeight = std::max(0, detPanel.rows - panelHeaderHeight);
            if (detContentHeight > 0) {
                cv::Mat detContent = detPanel(cv::Rect(0, panelHeaderHeight, detPanel.cols, detContentHeight));
                if (!currentDetectionsCanvas.empty()) {
                    cv::Mat resized;
                    cv::resize(currentDetectionsCanvas, resized, detContent.size());
                    resized.copyTo(detContent);
                    cv::Mat tint(detContent.size(), detContent.type(), cv::Scalar(14, 14, 22));
                    cv::addWeighted(detContent, 0.88, tint, 0.12, 0.0, detContent);
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
                        cv::LINE_AA);

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
                            cv::LINE_AA);
                cv::putText(detPanel,
                            fpsLabel,
                            cv::Point(detPanel.cols / 2 - 80, detPanel.rows - 24),
                            cv::FONT_HERSHEY_DUPLEX,
                            0.5,
                            subtitleColor,
                            1,
                            cv::LINE_AA);
                cv::putText(detPanel,
                            latencyLabel,
                            cv::Point(detPanel.cols - 220, detPanel.rows - 24),
                            cv::FONT_HERSHEY_DUPLEX,
                            0.5,
                            subtitleColor,
                            1,
                            cv::LINE_AA);
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
                            cv::LINE_AA);
            }

            cv::Mat depthPanel = setupPanel(depthRect, lastDepthValid ? "Depth Map" : "Depth Map (pending)");
            const int depthContentHeight = std::max(0, depthPanel.rows - panelHeaderHeight);
            if (depthContentHeight > 0) {
                cv::Mat depthContent = depthPanel(cv::Rect(0, panelHeaderHeight, depthPanel.cols, depthContentHeight));
                if (!lastDepthCanvas.empty()) {
                    cv::Mat resized;
                    cv::resize(lastDepthCanvas, resized, depthContent.size());
                    resized.copyTo(depthContent);
                } else {
                    const int fallbackY = panelHeaderHeight + depthContentHeight / 2;
                    renderTextBlock(depthPanel,
                                    {"Depth data pending"},
                                    warningColor,
                                    fallbackY,
                                    26);
                }
            }

            cv::Mat snapshotPanel = setupPanel(snapshotRect, "Last Pothole Frame");
            const int snapshotContentHeight = std::max(0, snapshotPanel.rows - panelHeaderHeight);
            if (snapshotContentHeight > 0) {
                cv::Mat snapshotContent = snapshotPanel(cv::Rect(0, panelHeaderHeight, snapshotPanel.cols, snapshotContentHeight));
                if (!lastPotholeCanvas.empty()) {
                    cv::Mat resized;
                    cv::resize(lastPotholeCanvas, resized, snapshotContent.size());
                    resized.copyTo(snapshotContent);
                    cv::Mat tint(snapshotContent.size(), snapshotContent.type(), cv::Scalar(18, 18, 32));
                    cv::addWeighted(snapshotContent, 0.9, tint, 0.1, 0.0, snapshotContent);
                } else {
                    const int placeholderY = panelHeaderHeight + snapshotContentHeight / 2;
                    renderTextBlock(snapshotPanel,
                                    {"No pothole frame recorded"},
                                    warningColor,
                                    placeholderY,
                                    26);
                }
            }

            cv::Mat insightsPanel = setupPanel(insightsRect, "Operational Insights");
            std::vector<std::string> insightLines;
            insightLines.emplace_back("Latency avg: " + (lastAvgLatency > 0.0 ? formatDouble(lastAvgLatency) + " ms" : std::string("n/a")));
            insightLines.emplace_back("FPS smooth: " + (lastSmoothedFps > 0.0 ? formatDouble(lastSmoothedFps) : std::string("n/a")));
            insightLines.emplace_back("MiDaS stride: " + std::to_string(lastObservedStride));
            insightLines.emplace_back("YOLO max: " + formatDouble(static_cast<double>(lastMaxConfidence)));
            insightLines.emplace_back("YOLO peak: " + formatDouble(lastYoloPeak));
            insightLines.emplace_back("Fusion peak: " + formatDouble(lastFusionPeak));
            insightLines.emplace_back("Hazard state: " + std::string(lastHazardTriggered ? "ACTIVE" : "nominal"));
            insightLines.emplace_back("CPU temp: " + (std::isnan(lastCpuTemp) ? std::string("n/a") : formatDouble(lastCpuTemp) + " C"));
            insightLines.emplace_back("Events buffered: " + std::to_string(potholeLog.size()));
            renderTextBlock(insightsPanel, insightLines, textColor, panelHeaderHeight + 32, 26);
            if (lastHazardTriggered) {
                cv::putText(insightsPanel,
                            "HAZARD DETECTED",
                            cv::Point(22, insightsPanel.rows - 32),
                            cv::FONT_HERSHEY_DUPLEX,
                            0.64,
                            hazardAccent,
                            1,
                            cv::LINE_AA);
            }

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
        std::cout << "======================================\n";

    } catch (const std::exception& ex) {
        std::cerr << "[system_visual_test] " << ex.what() << "\n";
        return 1;
    }

    return 0;
}

#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <openvino/openvino.hpp>

#include "safe_queue.hpp"   // adjust to your project's include path

namespace vigia {

// ── Forward-declared so RequestWrap's OpenVINO members don't leak
//    into every translation unit that includes this header.
struct RequestWrap;

/* ─── Public data types ───────────────────────────────────────────── */

struct Detection {
    cv::Rect boundingBox;
    float    confidence{0.0f};
    int      classId{-1};
};

struct FramePacket {
    std::uint64_t frameId{0};
    cv::Mat       frame;
    double        timestamp{0.0};
};

struct PerceptionResult {
    std::uint64_t          frameId{0};
    std::vector<Detection> detections;
    float                  greatestConfidence{0.0f};
    double                 timestamp{0.0};
};

/* ═══════════════════════════════════════════════════════════════════
 *  PerceptionAgent
 * ═══════════════════════════════════════════════════════════════════ */
class PerceptionAgent {
public:
    // Owning-Core constructor (creates its own ov::Core)
    PerceptionAgent(const std::string& modelXmlPath,
                    const std::string& device,
                    int cameraIndex = 0);

    // Shared-Core constructor (avoids duplicate plugin discovery on Pi 4)
    PerceptionAgent(ov::Core& sharedCore,
                    const std::string& modelXmlPath,
                    const std::string& device,
                    int cameraIndex = 0);

    virtual ~PerceptionAgent();

    // H-HMAS agent-loop entry point
    void run(SafeQueue<FramePacket>&     inputQueue,
             SafeQueue<PerceptionResult>& outputQueue,
             std::atomic<bool>&          running);

    virtual bool captureFrame(cv::Mat& frame);
    virtual std::vector<Detection> runInference(const cv::Mat& frame);
    virtual void notifyProcessingComplete(std::uint64_t /*frameIndex*/) {}

    bool isModelLoaded() const { return modelLoaded_; }

protected:
    struct Letterbox {
        float scale{1.0f};
        int   pad_w{0};
        int   pad_h{0};
    };

    cv::Mat              preprocess(const cv::Mat& frame, Letterbox& lb);
    std::vector<Detection> postprocess(const ov::Tensor& output,
                                       const Letterbox&  lb,
                                       const cv::Size&   origSize);
    float aggregateConfidence(const std::vector<Detection>& detections) const;

    // ── OpenVINO core ──────────────────────────────────────────────
    ov::Core  ownedCore_;       // used when no external Core is supplied
    ov::Core* core_;            // always points to the active Core

    ov::CompiledModel          compiledModel_;
    ov::Output<const ov::Node> outputTensor_;   // read-only graph metadata, safe to share

    // ── Model metadata ─────────────────────────────────────────────
    int  inputWidth_{640};
    int  inputHeight_{640};
    bool isInt8Model_{false};
    bool modelLoaded_{false};

    // ── Thresholds ─────────────────────────────────────────────────
    static constexpr float kConfThresholdFp32 = 0.25f;
    static constexpr float kConfThresholdInt8 = 0.20f;
    static constexpr float kIouThreshold      = 0.45f;
    float confThreshold_{kConfThresholdFp32};

    // ── Camera (fallback capture) ──────────────────────────────────
    cv::VideoCapture camera_;
    bool             cameraInitialized_{false};
    int              cameraIndex_{0};

private:
    void loadNetwork(const std::string& modelXmlPath,
                     const std::string& device);

    // ════════════════════════════════════════════════════════════════
    //  Thread-safe InferRequest pool
    //
    //  kPoolSize MUST equal the num_requests hint in compile_model().
    //
    //  Each RequestWrap owns one InferRequest + one pre-allocated
    //  ov::Tensor. Callers write into wrap->inputTensor and run
    //  wrap->inferRequest — no shared state between concurrent calls.
    //
    //  getRequest()    – blocks on poolCv_ until a slot is free.
    //  returnRequest() – pushes the slot back and notifies one waiter.
    //                    Called inside try/catch in runInference so an
    //                    exception can never permanently drain the pool.
    // ════════════════════════════════════════════════════════════════
    static constexpr std::size_t kPoolSize = 2;  // matches num_requests(2)

    std::queue<std::unique_ptr<RequestWrap>> idleRequests_;
    mutable std::mutex                       poolMutex_;
    std::condition_variable                  poolCv_;

    std::unique_ptr<RequestWrap> getRequest();
    void returnRequest(std::unique_ptr<RequestWrap> wrap);
};

} // namespace vigia
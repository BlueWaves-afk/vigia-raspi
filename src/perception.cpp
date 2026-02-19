#include "perception.hpp"

#include <algorithm>
#include <numeric>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <csetjmp>
#include <csignal>
#include <cstring>    // std::memcpy

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <openvino/runtime/intel_cpu/properties.hpp>
#include <openvino/core/preprocess/pre_post_process.hpp>

#if defined(__aarch64__) || defined(__ARM_NEON)
#  include <arm_neon.h>
#endif

#include <unistd.h>   // write(), _exit(), STDERR_FILENO

/* ═══════════════════════════════════════════════════════════════════════════
 *  SIGBUS recovery for compile_model()
 *
 *  On ARM64 without KleidiAI/ACL, the OpenVINO reference CPU plugin may
 *  emit unaligned memory accesses during JIT compilation → SIGBUS.
 *  We catch it with sigsetjmp/siglongjmp and continue in degraded mode.
 * ═══════════════════════════════════════════════════════════════════════════ */
static thread_local sigjmp_buf            g_compileJmpBuf;
static thread_local volatile sig_atomic_t g_compileJmpActive = 0;

static void compileModelSigbusHandler(int sig) {
    if (g_compileJmpActive) {
        g_compileJmpActive = 0;
        siglongjmp(g_compileJmpBuf, sig);
    }
#ifdef __linux__
    static const char msg[] = "\n[FATAL] SIGBUS outside guarded compile_model region\n";
    (void)::write(STDERR_FILENO, msg, sizeof(msg) - 1);
#endif
    ::_exit(128 + sig);
}

namespace vigia {

/* ═══════════════════════════════════════════════════════════════════════════
 *  RequestWrap — full definition (forward-declared in perception.hpp)
 *
 *  Owns exactly ONE InferRequest and ONE pre-allocated input ov::Tensor.
 *  The tensor is bound to the request once in loadNetwork() and reused
 *  every frame — zero per-frame heap allocation.
 *
 *  WHY a separate tensor per wrap?
 *    ov::InferRequest is NOT thread-safe.  If two callers shared a single
 *    inputTensor_ and wrote their preprocessed frames concurrently, the
 *    second write would corrupt the first frame's data mid-inference.
 *    Giving each wrap its own tensor eliminates this race entirely.
 * ═══════════════════════════════════════════════════════════════════════════ */
struct RequestWrap {
    ov::InferRequest inferRequest;
    ov::Tensor       inputTensor;   // pre-allocated, bound once to inferRequest
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  NMS helper — fixes merged boxes produced by INT8 quantization
 * ═══════════════════════════════════════════════════════════════════════════ */
namespace {

std::vector<int> nmsBoxes(const std::vector<cv::Rect>& boxes,
                           const std::vector<float>&    scores,
                           float scoreThreshold,
                           float iouThreshold) {
    std::vector<int> indices;
    if (boxes.empty()) return indices;

    std::vector<int> order(boxes.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&scores](int a, int b) { return scores[a] > scores[b]; });

    std::vector<bool> suppressed(boxes.size(), false);

    for (std::size_t i = 0; i < order.size(); ++i) {
        const int idx = order[i];
        if (suppressed[idx] || scores[idx] < scoreThreshold) continue;

        indices.push_back(idx);
        const cv::Rect& boxA  = boxes[idx];
        const float     areaA = static_cast<float>(boxA.area());

        for (std::size_t j = i + 1; j < order.size(); ++j) {
            const int jdx = order[j];
            if (suppressed[jdx]) continue;

            const cv::Rect& boxB      = boxes[jdx];
            const cv::Rect  inter     = boxA & boxB;
            const float     interArea = static_cast<float>(inter.area());
            const float     areaB     = static_cast<float>(boxB.area());
            const float     iou       = interArea / (areaA + areaB - interArea + 1e-6f);

            if (iou > iouThreshold)
                suppressed[jdx] = true;
        }
    }
    return indices;
}

} // anonymous namespace

/* ═══════════════════════════════════════════════════════════════════════════
 *  Pool helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

std::unique_ptr<RequestWrap> PerceptionAgent::getRequest() {
    std::unique_lock<std::mutex> lock(poolMutex_);
    poolCv_.wait(lock, [this] { return !idleRequests_.empty(); });
    auto wrap = std::move(idleRequests_.front());
    idleRequests_.pop();
    return wrap;
}

void PerceptionAgent::returnRequest(std::unique_ptr<RequestWrap> wrap) {
    {
        std::lock_guard<std::mutex> lock(poolMutex_);
        idleRequests_.push(std::move(wrap));
    }
    poolCv_.notify_one();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Constructors / Destructor
 * ═══════════════════════════════════════════════════════════════════════════ */

PerceptionAgent::PerceptionAgent(const std::string& modelXmlPath,
                                 const std::string& device,
                                 int                cameraIndex)
    : core_(&ownedCore_), cameraIndex_(cameraIndex) {
    loadNetwork(modelXmlPath, device);
}

PerceptionAgent::PerceptionAgent(ov::Core&          sharedCore,
                                 const std::string& modelXmlPath,
                                 const std::string& device,
                                 int                cameraIndex)
    : core_(&sharedCore), cameraIndex_(cameraIndex) {
    loadNetwork(modelXmlPath, device);
}

PerceptionAgent::~PerceptionAgent() {
    if (camera_.isOpened())
        camera_.release();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  H-HMAS Agent Loop
 * ═══════════════════════════════════════════════════════════════════════════ */

void PerceptionAgent::run(SafeQueue<FramePacket>&     inputQueue,
                          SafeQueue<PerceptionResult>& outputQueue,
                          std::atomic<bool>&           running) {
    while (running.load(std::memory_order_relaxed)) {
        auto packetOpt = inputQueue.try_pop();
        if (!packetOpt) continue;

        FramePacket packet = std::move(*packetOpt);
        auto detections   = runInference(packet.frame);

        PerceptionResult result;
        result.frameId            = packet.frameId;
        result.detections         = detections;
        result.greatestConfidence = aggregateConfidence(detections);
        result.timestamp          = packet.timestamp;

        outputQueue.push(std::move(result));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  captureFrame  (default camera path)
 * ═══════════════════════════════════════════════════════════════════════════ */

bool PerceptionAgent::captureFrame(cv::Mat& frame) {
    if (!cameraInitialized_) {
        cameraInitialized_ = camera_.open(cameraIndex_);
        if (!cameraInitialized_) return false;
        camera_.set(cv::CAP_PROP_FRAME_WIDTH,  640.0);
        camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 480.0);
    }
    if (!camera_.isOpened()) return false;
    return camera_.read(frame);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  loadNetwork
 *
 *  Steps:
 *    1. Validate .xml / .bin files
 *    2. Log /proc/meminfo (Linux only)
 *    3. read_model()
 *    4. Detect INT8 (FakeQuantize ops) and choose threshold / layout
 *    5. PrePostProcessor for INT8 path
 *    6. Configure CPU plugin properties
 *    7. compile_model() with SIGBUS recovery
 *    8. Resolve input dimensions
 *    9. Populate idleRequests_ pool   ← THE FIX
 * ═══════════════════════════════════════════════════════════════════════════ */

void PerceptionAgent::loadNetwork(const std::string& modelXmlPath,
                                  const std::string& device) {
    // ── 1. Derive .bin path ──────────────────────────────────────────────
    std::string modelBinPath = modelXmlPath;
    if (auto pos = modelBinPath.rfind(".xml"); pos != std::string::npos)
        modelBinPath.replace(pos, 4, ".bin");

    // ── 2. Validate model files ──────────────────────────────────────────
    auto validateFile = [](const std::string& path, const std::string& label) {
        std::ifstream f(path, std::ios::binary | std::ios::ate);
        if (!f.is_open()) {
            std::cerr << "[YOLO26] FATAL: Cannot open " << label << ": " << path << std::flush << std::endl;
            throw std::runtime_error("Cannot open model file: " + path);
        }
        const auto sz = f.tellg();
        if (sz <= 0) {
            std::cerr << "[YOLO26] FATAL: " << label << " is empty: " << path << std::flush << std::endl;
            throw std::runtime_error("Model file is empty: " + path);
        }
        std::cout << "[YOLO26] " << label << " OK  size=" << sz << " bytes  path=" << path << std::flush << std::endl;
    };
    validateFile(modelXmlPath, ".xml");
    validateFile(modelBinPath, ".bin");

    // ── 3. /proc/meminfo snapshot ────────────────────────────────────────
#ifdef __linux__
    {
        std::ifstream mem("/proc/meminfo");
        if (mem.is_open()) {
            std::string line;
            std::cout << "[YOLO26] /proc/meminfo before model load:" << std::flush << std::endl;
            while (std::getline(mem, line)) {
                if (line.rfind("MemTotal", 0) == 0 || line.rfind("MemFree", 0) == 0 ||
                    line.rfind("MemAvailable", 0) == 0 || line.rfind("SwapTotal", 0) == 0 ||
                    line.rfind("SwapFree", 0) == 0)
                    std::cout << "[YOLO26]   " << line << std::flush << std::endl;
            }
        }
    }
#endif

    // ── 4. read_model ────────────────────────────────────────────────────
    std::cout << "[TRACE] >>> read_model() BEGIN  xml=" << modelXmlPath << std::flush << std::endl;
    std::shared_ptr<ov::Model> model;
    try {
        model = core_->read_model(modelXmlPath);
    } catch (const ov::Exception& e) {
        std::cerr << "[TRACE] !!! ov::Exception in read_model: " << e.what() << std::flush << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "[TRACE] !!! std::exception in read_model: " << e.what() << std::flush << std::endl;
        throw;
    }
    std::cout << "[TRACE] <<< read_model() END" << std::flush << std::endl;

    // ── Detect INT8 (FakeQuantize ops) ───────────────────────────────────
    isInt8Model_ = false;
    for (const auto& op : model->get_ops()) {
        if (op->get_type_name() == std::string("FakeQuantize")) {
            isInt8Model_ = true;
            break;
        }
    }
    confThreshold_ = isInt8Model_ ? kConfThresholdInt8 : kConfThresholdFp32;
    std::cout << "[YOLO26] Model type: " << (isInt8Model_ ? "INT8" : "FP32")
              << "  conf=" << confThreshold_ << "  iou=" << kIouThreshold << std::flush << std::endl;

    // ── 5. PrePostProcessor (INT8 path only) ─────────────────────────────
    if (isInt8Model_) {
        ov::preprocess::PrePostProcessor ppp(model);
        ppp.input().tensor()
            .set_element_type(ov::element::u8)
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::RGB);
        ppp.input().preprocess()
            .convert_element_type(ov::element::f32)
            .scale(255.0f);
        ppp.input().model().set_layout("NCHW");
        model = ppp.build();
        std::cout << "[YOLO26] PrePostProcessor: U8 NHWC RGB -> F32 NCHW" << std::flush << std::endl;
    } else {
        model->get_parameters()[0]->set_layout("NCHW");
    }
    ov::set_batch(model, 1);

    // ── 6. CPU plugin properties ─────────────────────────────────────────
    std::cout << "[TRACE] >>> Configuring CPU plugin properties" << std::flush << std::endl;
    try {
        core_->set_property("CPU", ov::enable_mmap(false));
        core_->set_property("CPU", ov::num_streams(1));
        core_->set_property("CPU", ov::inference_num_threads(4));
        std::cout << "[YOLO26] num_streams=1  inference_num_threads=4  mmap=off" << std::flush << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[YOLO26] WARNING: Could not set CPU properties: " << e.what()
                  << " (continuing with defaults)" << std::flush << std::endl;
    }

    // ── 7. compile_model (with SIGBUS recovery) ──────────────────────────
    std::cout << "[TRACE] >>> compile_model() BEGIN (device=" << device << ")" << std::flush << std::endl;

    {
        struct sigaction sa{}, oldSa{};
        sa.sa_handler = compileModelSigbusHandler;
        sa.sa_flags   = 0;
        sigemptyset(&sa.sa_mask);
        ::sigaction(SIGBUS, &sa, &oldSa);

        g_compileJmpActive = 1;
        const int jmpVal   = sigsetjmp(g_compileJmpBuf, 1);

        if (jmpVal != 0) {
            g_compileJmpActive = 0;
            ::sigaction(SIGBUS, &oldSa, nullptr);
            std::cerr << "\n[YOLO26] SIGBUS caught during compile_model() -- entering DEGRADED mode.\n"
                      << "[YOLO26] Fix: rebuild OpenVINO with -DENABLE_KLEIDIAI=ON\n"
                      << std::flush << std::endl;
            modelLoaded_ = false;
            return;
        }

        try {
            compiledModel_ = core_->compile_model(
                model,
                device,
                {
                    ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
                    ov::hint::num_requests(static_cast<uint32_t>(kPoolSize)),
                    ov::hint::inference_precision(ov::element::dynamic)
                }
            );
        } catch (const ov::Exception& e) {
            g_compileJmpActive = 0;
            ::sigaction(SIGBUS, &oldSa, nullptr);
            std::cerr << "[TRACE] !!! ov::Exception in compile_model: " << e.what() << std::flush << std::endl;
            modelLoaded_ = false;
            return;
        } catch (const std::exception& e) {
            g_compileJmpActive = 0;
            ::sigaction(SIGBUS, &oldSa, nullptr);
            std::cerr << "[TRACE] !!! std::exception in compile_model: " << e.what() << std::flush << std::endl;
            modelLoaded_ = false;
            return;
        }

        g_compileJmpActive = 0;
        ::sigaction(SIGBUS, &oldSa, nullptr);
    }
    std::cout << "[TRACE] <<< compile_model() END (success)" << std::flush << std::endl;

    // ── 8. Resolve input dimensions ──────────────────────────────────────
    outputTensor_ = compiledModel_.output(0);
    const auto& inputShape = compiledModel_.input().get_shape();
    if (isInt8Model_) {
        inputHeight_ = static_cast<int>(inputShape[1]);   // NHWC: [1,H,W,C]
        inputWidth_  = static_cast<int>(inputShape[2]);
    } else {
        inputHeight_ = static_cast<int>(inputShape[2]);   // NCHW: [1,C,H,W]
        inputWidth_  = static_cast<int>(inputShape[3]);
    }
    std::cout << "[YOLO26] Input: " << inputWidth_ << "x" << inputHeight_ << std::flush << std::endl;

    // ── 9. Populate the request pool ─────────────────────────────────────
    //
    //  ROOT CAUSE FIX:
    //  The original code created inferRequest_ and inputTensor_ as single
    //  class members.  When system_visual_test.cpp compiled the model with
    //  num_requests(2), OpenVINO allocated internal resources for two
    //  pipeline slots.  The Coordinator's processing thread then called
    //  runInference() while the previous request was still in-flight,
    //  causing a pop from an empty idleRequests_ queue -> assertion failure.
    //
    //  Fix: create kPoolSize RequestWrap objects, each with its own
    //  InferRequest AND its own pre-allocated ov::Tensor bound at startup.
    //  getRequest()/returnRequest() guard the queue with poolMutex_ +
    //  poolCv_ so callers block rather than crash on concurrent access.
    //
    std::cout << "[TRACE] >>> Building request pool (kPoolSize=" << kPoolSize << ")" << std::flush << std::endl;
    {
        std::lock_guard<std::mutex> lock(poolMutex_);
        for (std::size_t i = 0; i < kPoolSize; ++i) {
            auto wrap = std::make_unique<RequestWrap>();

            wrap->inferRequest = compiledModel_.create_infer_request();

            // Each RequestWrap MUST have its own tensor.
            // Sharing a single inputTensor_ across two concurrent inference
            // calls corrupts whichever frame was written second.
            if (isInt8Model_) {
                wrap->inputTensor = ov::Tensor(
                    ov::element::u8,
                    {1,
                     static_cast<std::size_t>(inputHeight_),
                     static_cast<std::size_t>(inputWidth_),
                     3});   // NHWC
            } else {
                wrap->inputTensor = ov::Tensor(
                    ov::element::f32,
                    {1, 3,
                     static_cast<std::size_t>(inputHeight_),
                     static_cast<std::size_t>(inputWidth_)});  // NCHW
            }

            // Bind tensor to this request once -- never rebound per frame.
            wrap->inferRequest.set_input_tensor(wrap->inputTensor);

            idleRequests_.push(std::move(wrap));
        }
    }
    std::cout << "[TRACE] <<< Request pool ready  layout="
              << (isInt8Model_ ? "U8 NHWC" : "F32 NCHW") << std::flush << std::endl;

    modelLoaded_ = true;
    std::cout << "[YOLO26] Model loaded OK.  Input: "
              << inputWidth_ << "x" << inputHeight_ << std::flush << std::endl;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  preprocess -- letterbox resize + BGR->RGB + optional float conversion
 * ═══════════════════════════════════════════════════════════════════════════ */

cv::Mat PerceptionAgent::preprocess(const cv::Mat& frame, Letterbox& lb) {
    cv::Mat rgb;
    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);

    const float r = std::min(static_cast<float>(inputWidth_)  / frame.cols,
                             static_cast<float>(inputHeight_) / frame.rows);
    const int newW = static_cast<int>(std::round(frame.cols * r));
    const int newH = static_cast<int>(std::round(frame.rows * r));

    lb.scale = r;
    lb.pad_w = (inputWidth_  - newW) / 2;
    lb.pad_h = (inputHeight_ - newH) / 2;

    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(newW, newH));

    cv::Mat padded;
    cv::copyMakeBorder(resized, padded,
                       lb.pad_h, inputHeight_ - newH - lb.pad_h,
                       lb.pad_w, inputWidth_  - newW - lb.pad_w,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    if (isInt8Model_)
        return padded;          // U8, NHWC -- OpenVINO PPP handles the rest

    cv::Mat blob;
    padded.convertTo(blob, CV_32F, 1.0 / 255.0);
    return blob;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  runInference -- acquire request -> fill tensor -> async infer -> return
 * ═══════════════════════════════════════════════════════════════════════════ */

std::vector<Detection> PerceptionAgent::runInference(const cv::Mat& frame) {
    if (!modelLoaded_)
        return {};

    // ── Acquire an idle request from the pool ────────────────────────────
    //  Blocks via condition_variable if all slots are in use.
    auto wrap = getRequest();

    // ── Preprocess ───────────────────────────────────────────────────────
    Letterbox lb;
    cv::Mat   blob      = preprocess(frame, lb);
    const int planeSize = inputHeight_ * inputWidth_;

    // ── Copy pixels into THIS request's own tensor ───────────────────────
    if (isInt8Model_) {
        uint8_t*       dst        = wrap->inputTensor.data<uint8_t>();
        const uint8_t* src        = blob.ptr<uint8_t>();
        const int      totalBytes = planeSize * 3;
#if defined(__aarch64__) || defined(__ARM_NEON)
        int i = 0;
        for (int end = totalBytes - (totalBytes % 16); i < end; i += 16)
            vst1q_u8(dst + i, vld1q_u8(src + i));
        for (; i < totalBytes; ++i) dst[i] = src[i];
#else
        std::memcpy(dst, src, static_cast<std::size_t>(totalBytes));
#endif
    } else {
        float*       dst   = wrap->inputTensor.data<float>();
        const float* src   = blob.ptr<float>();
        float*       dst_r = dst;
        float*       dst_g = dst + planeSize;
        float*       dst_b = dst + planeSize * 2;
#if defined(__aarch64__) || defined(__ARM_NEON)
        int i = 0;
        for (int end = planeSize - (planeSize % 4); i < end; i += 4) {
            float32x4x3_t px = vld3q_f32(src + i * 3);
            vst1q_f32(dst_r + i, px.val[0]);
            vst1q_f32(dst_g + i, px.val[1]);
            vst1q_f32(dst_b + i, px.val[2]);
        }
        for (; i < planeSize; ++i) {
            dst_r[i] = src[i * 3 + 0];
            dst_g[i] = src[i * 3 + 1];
            dst_b[i] = src[i * 3 + 2];
        }
#else
        for (int i = 0; i < planeSize; ++i) {
            dst_r[i] = src[i * 3 + 0];
            dst_g[i] = src[i * 3 + 1];
            dst_b[i] = src[i * 3 + 2];
        }
#endif
    }

    // ── Async inference on this request's private InferRequest ───────────
    std::vector<Detection> detections;
    try {
        wrap->inferRequest.start_async();
        wrap->inferRequest.wait();
        detections = postprocess(
            wrap->inferRequest.get_tensor(outputTensor_), lb, frame.size());
    } catch (const ov::Exception& e) {
        std::cerr << "[YOLO] ov::Exception during inference: " << e.what() << std::endl;
        returnRequest(std::move(wrap));   // always return -- never leak a slot
        throw;
    } catch (const std::exception& e) {
        std::cerr << "[YOLO] std::exception during inference: " << e.what() << std::endl;
        returnRequest(std::move(wrap));
        throw;
    }

    // ── Return the request AFTER results are copied out ──────────────────
    returnRequest(std::move(wrap));
    return detections;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  postprocess -- decode [1,300,6] YOLO output + NMS
 * ═══════════════════════════════════════════════════════════════════════════ */

std::vector<Detection> PerceptionAgent::postprocess(const ov::Tensor& output,
                                                     const Letterbox&  lb,
                                                     const cv::Size&   origSize) {
#if defined(__clang__) || defined(__GNUC__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
    const float* data = output.data<float>();
#if defined(__clang__) || defined(__GNUC__)
#  pragma GCC diagnostic pop
#endif

    const auto& shape          = output.get_shape();
    const int   num_detections = static_cast<int>(shape[1]);

    std::vector<cv::Rect> boxes;
    std::vector<float>    scores;
    std::vector<int>      classIds;
    boxes.reserve(128);
    scores.reserve(128);
    classIds.reserve(128);

    for (int i = 0; i < num_detections; ++i) {
        const float* row        = data + i * 6;
        const float  confidence = row[4];
        if (confidence < confThreshold_) continue;

        float x1 = (row[0] - lb.pad_w) / lb.scale;
        float y1 = (row[1] - lb.pad_h) / lb.scale;
        float x2 = (row[2] - lb.pad_w) / lb.scale;
        float y2 = (row[3] - lb.pad_h) / lb.scale;

        x1 = std::clamp(x1, 0.0f, static_cast<float>(origSize.width));
        y1 = std::clamp(y1, 0.0f, static_cast<float>(origSize.height));
        x2 = std::clamp(x2, 0.0f, static_cast<float>(origSize.width));
        y2 = std::clamp(y2, 0.0f, static_cast<float>(origSize.height));

        const int bw = static_cast<int>(x2 - x1);
        const int bh = static_cast<int>(y2 - y1);
        if (bw > 0 && bh > 0) {
            boxes.emplace_back(static_cast<int>(x1), static_cast<int>(y1), bw, bh);
            scores.push_back(confidence);
            classIds.push_back(static_cast<int>(row[5]));
        }
    }

    const std::vector<int> keep = nmsBoxes(boxes, scores, confThreshold_, kIouThreshold);

    std::vector<Detection> detections;
    detections.reserve(keep.size());
    for (int idx : keep) {
        Detection det;
        det.boundingBox = boxes[idx];
        det.confidence  = scores[idx];
        det.classId     = classIds[idx];
        detections.push_back(det);
    }
    return detections;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  aggregateConfidence
 * ═══════════════════════════════════════════════════════════════════════════ */

float PerceptionAgent::aggregateConfidence(const std::vector<Detection>& detections) const {
    float best = 0.0f;
    for (const auto& d : detections)
        best = std::max(best, d.confidence);
    return best;
}

} // namespace vigia
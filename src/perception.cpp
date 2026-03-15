#include "perception.hpp"
#include <algorithm>
#include <numeric>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <csetjmp>
#include <csignal>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <openvino/runtime/intel_cpu/properties.hpp>
#include <openvino/core/preprocess/pre_post_process.hpp>

#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#endif

#include <unistd.h>

/* ── SIGBUS recovery for compile_model() ─────────────────────────────────
 * On ARM64 without ACL, OpenVINO's reference CPU plugin may emit
 * unaligned memory accesses during JIT compilation, raising SIGBUS.
 * We use sigsetjmp/siglongjmp to recover instead of crashing.
 * ────────────────────────────────────────────────────────────────────── */
static thread_local sigjmp_buf g_compileJmpBuf;
static thread_local volatile sig_atomic_t g_compileJmpActive = 0;

static void compileModelSigbusHandler(int sig) {
    if (g_compileJmpActive) {
        g_compileJmpActive = 0;
        siglongjmp(g_compileJmpBuf, sig);
    }
    // If not in a guarded region, use default behavior
    const char msg[] = "\n[FATAL] SIGBUS outside guarded compile_model region\n";
#ifdef __linux__
    (void)::write(STDERR_FILENO, msg, sizeof(msg) - 1);
#endif
    ::_exit(128 + sig);
}

namespace vigia {

namespace {
// ── NMS to fix merged boxes in INT8 quantized models ────────────────
// INT8 quantization can cause adjacent detections to merge; this separates them.
std::vector<int> nmsBoxes(const std::vector<cv::Rect>& boxes,
                          const std::vector<float>& scores,
                          float scoreThreshold,
                          float iouThreshold) {
    std::vector<int> indices;
    if (boxes.empty()) return indices;
    
    // Sort by score descending
    std::vector<int> order(boxes.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&scores](int a, int b) {
        return scores[a] > scores[b];
    });
    
    std::vector<bool> suppressed(boxes.size(), false);
    
    for (size_t i = 0; i < order.size(); ++i) {
        const int idx = order[i];
        if (suppressed[idx] || scores[idx] < scoreThreshold) continue;
        
        indices.push_back(idx);
        const cv::Rect& boxA = boxes[idx];
        const float areaA = static_cast<float>(boxA.area());
        
        for (size_t j = i + 1; j < order.size(); ++j) {
            const int jdx = order[j];
            if (suppressed[jdx]) continue;
            
            const cv::Rect& boxB = boxes[jdx];
            const cv::Rect intersection = boxA & boxB;
            const float interArea = static_cast<float>(intersection.area());
            const float areaB = static_cast<float>(boxB.area());
            const float iou = interArea / (areaA + areaB - interArea + 1e-6f);
            
            if (iou > iouThreshold) {
                suppressed[jdx] = true;
            }
        }
    }
    
    return indices;
}
} // anonymous namespace

PerceptionAgent::PerceptionAgent(const std::string& modelXmlPath,
                                 const std::string& device,
                                 int cameraIndex)
    : core_(&ownedCore_), cameraIndex_(cameraIndex) {
    loadNetwork(modelXmlPath, device);
}

PerceptionAgent::PerceptionAgent(ov::Core& sharedCore,
                                 const std::string& modelXmlPath,
                                 const std::string& device,
                                 int cameraIndex)
    : core_(&sharedCore), cameraIndex_(cameraIndex) {
    loadNetwork(modelXmlPath, device);
}

PerceptionAgent::~PerceptionAgent() {
    if (camera_.isOpened())
        camera_.release();
}

/* ===================== H-HMAS Agent Loop ===================== */

void PerceptionAgent::run(SafeQueue<FramePacket>& inputQueue,
                         SafeQueue<PerceptionResult>& outputQueue,
                         std::atomic<bool>& running) {
    while (running.load(std::memory_order_relaxed)) {
        auto packetOpt = inputQueue.try_pop();
        if (!packetOpt) {
            continue;
        }
        FramePacket packet = std::move(*packetOpt);

        auto detections = runInference(packet.frame);

        PerceptionResult result;
        result.frameId = packet.frameId;
        result.detections = detections;
        result.greatestConfidence = aggregateConfidence(detections);
        result.timestamp = packet.timestamp;

        outputQueue.push(std::move(result));
    }
}

bool PerceptionAgent::captureFrame(cv::Mat& frame) {
    if (!cameraInitialized_) {
        cameraInitialized_ = camera_.open(cameraIndex_);
        if (!cameraInitialized_)
            return false;

        camera_.set(cv::CAP_PROP_FRAME_WIDTH, 640.0);
        camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 480.0);
    }

    if (!camera_.isOpened())
        return false;

    if (!camera_.read(frame))
        return false;

    return true;
}

/* ===================== Network Loading ===================== */

void PerceptionAgent::loadNetwork(const std::string& modelXmlPath,
                                  const std::string& device) {
    // ── 1. Derive .bin path ─────────────────────────────────────────
    std::string modelBinPath = modelXmlPath;
    {
        auto pos = modelBinPath.rfind(".xml");
        if (pos != std::string::npos)
            modelBinPath.replace(pos, 4, ".bin");
    }

    // ── 2. Model Integrity: verify .xml and .bin are readable & non-empty
    auto validateFile = [](const std::string& path, const std::string& label) {
        std::ifstream f(path, std::ios::binary | std::ios::ate);
        if (!f.is_open()) {
            std::cerr << "[YOLO26] FATAL: Cannot open " << label << ": " << path << std::flush << std::endl;
            throw std::runtime_error("Cannot open model file: " + path);
        }
        const auto size = f.tellg();
        if (size <= 0) {
            std::cerr << "[YOLO26] FATAL: " << label << " is empty (0 bytes): " << path << std::flush << std::endl;
            throw std::runtime_error("Model file is empty: " + path);
        }
        std::cout << "[YOLO26] " << label << " OK  size=" << size << " bytes  path=" << path << std::flush << std::endl;
    };
    validateFile(modelXmlPath, ".xml");
    validateFile(modelBinPath, ".bin");

    // ── 3. RAM availability: read /proc/meminfo before heavy allocations
#ifdef __linux__
    {
        std::ifstream meminfo("/proc/meminfo");
        if (meminfo.is_open()) {
            std::string line;
            std::cout << "[YOLO26] /proc/meminfo snapshot before model load:" << std::flush << std::endl;
            while (std::getline(meminfo, line)) {
                if (line.rfind("MemTotal", 0) == 0 ||
                    line.rfind("MemFree", 0) == 0 ||
                    line.rfind("MemAvailable", 0) == 0 ||
                    line.rfind("SwapTotal", 0) == 0 ||
                    line.rfind("SwapFree", 0) == 0) {
                    std::cout << "[YOLO26]   " << line << std::flush << std::endl;
                }
            }
        } else {
            std::cerr << "[YOLO26] WARNING: Could not open /proc/meminfo" << std::flush << std::endl;
        }
    }
#endif

    // ── 4. read_model ───────────────────────────────────────────────
    std::cout << "[TRACE] >>> core_->read_model() BEGIN" << std::flush << std::endl;
    std::cout << "[TRACE]     .xml = " << modelXmlPath << std::flush << std::endl;
    std::cout << "[TRACE]     .bin = " << modelBinPath << std::flush << std::endl;

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
    std::cout << "[TRACE] <<< core_->read_model() END (success)" << std::flush << std::endl;

    // ── Detect INT8 quantized model ─────────────────────────────────
    // Check for FakeQuantize operations which indicate INT8 quantization
    isInt8Model_ = false;
    for (const auto& op : model->get_ops()) {
        if (op->get_type_name() == std::string("FakeQuantize")) {
            isInt8Model_ = true;
            break;
        }
    }
    
    // Set confidence threshold based on model type
    confThreshold_ = isInt8Model_ ? kConfThresholdInt8 : kConfThresholdFp32;
    std::cout << "[YOLO26] Model type: " << (isInt8Model_ ? "INT8 quantized" : "FP32")
              << " | conf threshold: " << confThreshold_
              << " | IOU threshold: " << kIouThreshold << std::flush << std::endl;

    // ── Configure preprocessing based on model type ─────────────────
    // INT8 models: Zero-copy U8 input (NHWC) → model (NCHW)
    // FP32 models: Standard F32 input with manual HWC→CHW transposition
    if (isInt8Model_) {
    ov::preprocess::PrePostProcessor ppp(model);
    ppp.input().tensor()
        .set_element_type(ov::element::u8)
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::RGB);

    // ONLY convert the type. Let the model's internal layers handle the division.
    ppp.input().preprocess()
        .convert_element_type(ov::element::f32)
        .scale(255.0f);

    ppp.input().model().set_layout("NCHW");
    model = ppp.build();
    std::cout << "[YOLO26] PrePostProcessor: U8 NHWC RGB -> F32 (Internal scaling expected)" << std::endl;
    } else {
        // FP32: Set layout directly, manual preprocessing in runInference
        model->get_parameters()[0]->set_layout("NCHW");
    }
    ov::set_batch(model, 1);

    // ── 5. Explicit plugin inspection: check CPU supported properties
    //    and log whether ACL backend is available.
    std::cout << "[TRACE] >>> Inspecting CPU plugin supported_properties" << std::flush << std::endl;
    try {
        auto props = core_->get_property(device, ov::supported_properties);
        bool foundAcl = false;
        for (const auto& prop : props) {
            const std::string name = prop;
            if (name.find("ACL") != std::string::npos ||
                name.find("arm_compute") != std::string::npos) {
                foundAcl = true;
            }
        }
        std::cout << "[YOLO26] CPU plugin properties (" << props.size() << " entries)."
                  << (foundAcl ? " ACL reference found." : " No explicit ACL property.")
                  << std::flush << std::endl;

        // Also log the full device name — this is the most reliable ACL indicator
        const std::string fullName = core_->get_property(device, ov::device::full_name);
        std::cout << "[YOLO26] Device full_name: " << fullName << std::flush << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[YOLO26] WARNING: Could not query CPU properties: " << e.what() << std::flush << std::endl;
    }

    // ── 6. Configure CPU plugin properties BEFORE compile_model ─────────
    //    These must be set via set_property() on the device, not passed
    //    to compile_model(), for proper ARM/KleidiAI backend selection.
    std::cout << "[TRACE] >>> Configuring CPU plugin properties" << std::flush << std::endl;
    try {
        // Disable mmap: forces standard RAM allocation instead of memory-mapping
        // the .bin weights file. Fixes SIGBUS on SD-card based systems (Raspberry Pi)
        // where mmap pages can fault if the filesystem returns EIO.
        core_->set_property("CPU", ov::enable_mmap(false));
        std::cout << "[TRACE] <<< ov::enable_mmap(false) set successfully" << std::flush << std::endl;

        // Threading config for Raspberry Pi 4 (4× Cortex-A72 @ 1.5GHz):
        // - num_streams(1): Single inference stream for minimum latency
        // - inference_num_threads(4): Use all 4 cores to parallelize ops
        // This prevents core saturation and context-switching overhead.
        core_->set_property("CPU", ov::num_streams(1));
        core_->set_property("CPU", ov::inference_num_threads(4));
        std::cout << "[YOLO26] Threading: num_streams=1, inference_num_threads=4" << std::flush << std::endl;
    } catch (const std::exception& e) {
        // Properties may not exist on older OpenVINO builds — non-fatal
        std::cerr << "[YOLO26] WARNING: Could not set CPU properties: " << e.what()
                  << " (continuing with defaults)" << std::flush << std::endl;
    }

    // ── 7. compile_model (with SIGBUS recovery) ──────────────────
    std::cout << "[TRACE] >>> core_->compile_model() BEGIN (device=" << device << ")" << std::flush << std::endl;

    // Print available RAM right before compilation
#ifdef __linux__
    {
        std::ifstream meminfo("/proc/meminfo");
        if (meminfo.is_open()) {
            std::string line;
            while (std::getline(meminfo, line)) {
                if (line.rfind("MemAvailable", 0) == 0) {
                    std::cout << "[YOLO26] PRE-COMPILE " << line << std::flush << std::endl;
                    break;
                }
            }
        }
    }
#endif

    // Install SIGBUS handler for compile_model() recovery.
    // On ARM64 without ACL, the reference CPU plugin may trigger SIGBUS
    // from unaligned JIT accesses.  We recover via siglongjmp so the
    // system can continue in degraded mode without this agent.
    {
        struct sigaction sa{}, oldSa{};
        sa.sa_handler = compileModelSigbusHandler;
        sa.sa_flags = 0;
        sigemptyset(&sa.sa_mask);
        ::sigaction(SIGBUS, &sa, &oldSa);

        g_compileJmpActive = 1;
        const int sigbusJmp = sigsetjmp(g_compileJmpBuf, 1);

        if (sigbusJmp != 0) {
            // We got here via siglongjmp — SIGBUS was caught
            g_compileJmpActive = 0;
            ::sigaction(SIGBUS, &oldSa, nullptr);  // restore old handler
            std::cerr << "\n[YOLO26] ══════════════════════════════════════════════════════\n"
                      << "[YOLO26] SIGBUS caught during compile_model()!\n"
                      << "[YOLO26] Root cause: OpenVINO ARM CPU plugin lacks ACL (Arm Compute Library).\n"
                      << "[YOLO26] The reference plugin uses unaligned memory accesses that\n"
                      << "[YOLO26] trigger SIGBUS on strict-alignment ARM64 hardware.\n"
                      << "[YOLO26] Fix: rebuild OpenVINO from source with -DENABLE_KLEIDIAI=ON\n"
                      << "[YOLO26] Continuing in DEGRADED mode (perception disabled).\n"
                      << "[YOLO26] ══════════════════════════════════════════════════════\n"
                      << std::flush << std::endl;
            modelLoaded_ = false;
            return;  // agent is alive but inference is disabled
        }

        try {
            // inference_precision = dynamic lets OpenVINO use native INT8
            // when the model contains FakeQuantize ops (INT8 quantized).
            // For FP32 models, it falls back to FP32 automatically.
            compiledModel_ = core_->compile_model(
                model,
                device,
                {
                    ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
                    ov::hint::num_requests(1),
                    ov::hint::inference_precision(ov::element::f32),
                    ov::inference_num_threads(4)
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
        ::sigaction(SIGBUS, &oldSa, nullptr);  // restore old handler
    }
    std::cout << "[TRACE] <<< core_->compile_model() END (success)" << std::flush << std::endl;

    inferRequest_ = compiledModel_.create_infer_request();
    outputTensor_ = compiledModel_.output(0);
    modelLoaded_ = true;

    const auto& inputShape = compiledModel_.input().get_shape();
    if (isInt8Model_) {
        // NHWC: [1, H, W, C]
        inputHeight_ = static_cast<int>(inputShape[1]);
        inputWidth_  = static_cast<int>(inputShape[2]);
    } else {
        // NCHW: [1, C, H, W]
        inputHeight_ = static_cast<int>(inputShape[2]);
        inputWidth_  = static_cast<int>(inputShape[3]);
    }

    std::cout << "[TRACE] >>> input tensor pre-allocation BEGIN (" << inputWidth_ << "x" << inputHeight_ << ")" << std::flush << std::endl;
    if (isInt8Model_) {
        inputTensor_ = ov::Tensor(ov::element::u8,
                      {1,
                       static_cast<std::size_t>(inputHeight_),
                       static_cast<std::size_t>(inputWidth_),
                       3});  // NHWC
    } else {
        inputTensor_ = ov::Tensor(ov::element::f32,
                                  {1, 3,
                                   static_cast<std::size_t>(inputHeight_),
                                   static_cast<std::size_t>(inputWidth_)});  // NCHW
    }
    inferRequest_.set_input_tensor(inputTensor_);
    std::cout << "[TRACE] <<< input tensor pre-allocation END (" 
              << (isInt8Model_ ? "U8 NHWC" : "F32 NCHW") << ")" << std::flush << std::endl;

    // Pre-allocate preprocessing buffers — eliminates per-frame heap alloc.
    // For INT8: preprocPadded_ wraps the tensor data directly (zero-copy write).
    preprocResized_.create(inputHeight_, inputWidth_, CV_8UC3);
    if (isInt8Model_) {
        // Header-only wrap of tensor buffer — copyMakeBorder writes straight into it
        preprocPadded_ = cv::Mat(inputHeight_, inputWidth_, CV_8UC3,
                                  inputTensor_.data<uint8_t>());
    } else {
        preprocPadded_.create(inputHeight_, inputWidth_, CV_8UC3);
        preprocBlob_.create(inputHeight_, inputWidth_, CV_32FC3);
    }

    // Pre-allocate NMS vectors — cleared each frame, never reallocated
    nmsBoxes_.reserve(128);
    nmsScores_.reserve(128);
    nmsClassIds_.reserve(128);

    std::cout << "[YOLO26] Model Loaded. Input: " << inputWidth_ << "x" << inputHeight_ << "\n";
}

/* ===================== Core Logic ===================== */

cv::Mat PerceptionAgent::preprocess(const cv::Mat& frame, Letterbox& lb) {
    // Letterbox scaling
    const float r = std::min(static_cast<float>(inputWidth_)  / frame.cols,
                             static_cast<float>(inputHeight_) / frame.rows);
    const int new_unpad_w = static_cast<int>(std::round(frame.cols * r));
    const int new_unpad_h = static_cast<int>(std::round(frame.rows * r));
    lb.scale = r;
    lb.pad_w = (inputWidth_  - new_unpad_w) / 2;
    lb.pad_h = (inputHeight_ - new_unpad_h) / 2;

    // Resize FIRST (on full-res BGR) — cvtColor then operates on small image only.
    // This reduces cvtColor work by ~9× for 1280×720 → 320×320.
    cv::resize(frame, preprocResized_, cv::Size(new_unpad_w, new_unpad_h));
    cv::cvtColor(preprocResized_, preprocResized_, cv::COLOR_BGR2RGB);

    // Letterbox pad into pre-allocated buffer.
    // INT8: preprocPadded_ wraps inputTensor_ data → zero-copy write into tensor.
    cv::copyMakeBorder(preprocResized_, preprocPadded_,
                       lb.pad_h, inputHeight_ - new_unpad_h - lb.pad_h,
                       lb.pad_w, inputWidth_  - new_unpad_w - lb.pad_w,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    if (isInt8Model_) {
        return preprocPadded_;  // already in tensor — caller skips memcpy
    }
    preprocPadded_.convertTo(preprocBlob_, CV_32F, 1.0 / 255.0);
    return preprocBlob_;
}

std::vector<Detection> PerceptionAgent::runInference(const cv::Mat& frame) {
    Letterbox lb;
    cv::Mat blob = preprocess(frame, lb);
    const int planeSize = inputHeight_ * inputWidth_;

    if (isInt8Model_) {
        // preprocPadded_ wraps inputTensor_ data — copyMakeBorder already wrote
        // directly into the tensor. Nothing to copy.
        (void)blob;
    } else {
        float* tensorData = inputTensor_.data<float>();
        const float* blobData = blob.ptr<float>();
        float* dst_r = tensorData;
        float* dst_g = tensorData + planeSize;
        float* dst_b = tensorData + planeSize * 2;
#if defined(__aarch64__) || defined(__ARM_NEON)
        int i = 0;
        const int simdEnd = planeSize - (planeSize % 4);
        for (; i < simdEnd; i += 4) {
            float32x4x3_t rgb = vld3q_f32(blobData + i * 3);
            vst1q_f32(dst_r + i, rgb.val[0]);
            vst1q_f32(dst_g + i, rgb.val[1]);
            vst1q_f32(dst_b + i, rgb.val[2]);
        }
        for (; i < planeSize; ++i) {
            dst_r[i] = blobData[i * 3 + 0];
            dst_g[i] = blobData[i * 3 + 1];
            dst_b[i] = blobData[i * 3 + 2];
        }
#else
        for (int i = 0; i < planeSize; ++i) {
            dst_r[i] = blobData[i * 3 + 0];
            dst_g[i] = blobData[i * 3 + 1];
            dst_b[i] = blobData[i * 3 + 2];
        }
#endif
    }

    // ── Async inference (OpenVINO 2025) ───────────────────────────
    // Input tensor is already bound via set_input_tensor in loadNetwork.
    try {
        inferRequest_.start_async();
        inferRequest_.wait();
    } catch (const ov::Exception& e) {
        std::cerr << "[YOLO] ov::Exception during inference: " << e.what() << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "[YOLO] std::exception during inference: " << e.what() << std::endl;
        throw;
    }

    return postprocess(inferRequest_.get_tensor(outputTensor_), lb, frame.size());
}

std::vector<Detection> PerceptionAgent::postprocess(const ov::Tensor& output, const Letterbox& lb, const cv::Size& origSize) {
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
    const float* data = output.data<float>();
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
    const auto& shape = output.get_shape(); 

    // YOLO26 NMS-Free output is typically [1, 300, 6]
    // Indices: 0:x1, 1:y1, 2:x2, 3:y2, 4:score, 5:class_id
    const int num_detections = static_cast<int>(shape[1]);

    // Use persistent vectors — clear() keeps capacity, avoids per-frame alloc
    nmsBoxes_.clear();
    nmsScores_.clear();
    nmsClassIds_.clear();

    for (int i = 0; i < num_detections; ++i) {
        const float* row = data + (i * 6);
        const float confidence = row[4];

        if (confidence < confThreshold_) continue;

        // ── Inverse letterbox scaling ───────────────────────────────
        // Map coordinates back to original frame (removing padding and scaling)
        float x1 = (row[0] - lb.pad_w) / lb.scale;
        float y1 = (row[1] - lb.pad_h) / lb.scale;
        float x2 = (row[2] - lb.pad_w) / lb.scale;
        float y2 = (row[3] - lb.pad_h) / lb.scale;

        // Clip to original frame boundaries
        x1 = std::clamp(x1, 0.0f, static_cast<float>(origSize.width));
        y1 = std::clamp(y1, 0.0f, static_cast<float>(origSize.height));
        x2 = std::clamp(x2, 0.0f, static_cast<float>(origSize.width));
        y2 = std::clamp(y2, 0.0f, static_cast<float>(origSize.height));

        const int bx = static_cast<int>(x1);
        const int by = static_cast<int>(y1);
        const int bw = static_cast<int>(x2 - x1);
        const int bh = static_cast<int>(y2 - y1);

        if (bw > 0 && bh > 0) {
            nmsBoxes_.emplace_back(bx, by, bw, bh);
            nmsScores_.push_back(confidence);
            nmsClassIds_.push_back(static_cast<int>(row[5]));
        }
    }

    std::vector<int> keepIndices = nmsBoxes(nmsBoxes_, nmsScores_, confThreshold_, kIouThreshold);

    std::vector<Detection> detections;
    detections.reserve(keepIndices.size());
    for (int idx : keepIndices) {
        Detection det;
        det.boundingBox = nmsBoxes_[idx];
        det.confidence  = nmsScores_[idx];
        det.classId     = nmsClassIds_[idx];
        detections.push_back(det);
    }
    return detections;
}

float PerceptionAgent::aggregateConfidence(const std::vector<Detection>& detections) const {
    float maxConfidence = 0.0F;
    for (const auto& det : detections)
        maxConfidence = std::max(maxConfidence, det.confidence);
    return maxConfidence;
}

} // namespace vigia
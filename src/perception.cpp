#include "perception.hpp"
#include <algorithm>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#endif

namespace vigia {

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

    // Set layout to NCHW as required by OpenVINO for this model type
    model->get_parameters()[0]->set_layout("NCHW");
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

    // ── 6. Disable mmap: forces standard RAM allocation instead of
    //    memory-mapping the .bin weights file.  Fixes SIGBUS on SD-card
    //    based systems (Raspberry Pi) where mmap pages can fault if the
    //    filesystem returns EIO.
    std::cout << "[TRACE] >>> Disabling mmap for CPU plugin" << std::flush << std::endl;
    try {
        core_->set_property("CPU", ov::enable_mmap(false));
        std::cout << "[TRACE] <<< ov::enable_mmap(false) set successfully" << std::flush << std::endl;
    } catch (const std::exception& e) {
        // enable_mmap may not exist on older OpenVINO builds — non-fatal
        std::cerr << "[YOLO26] WARNING: Could not set enable_mmap(false): " << e.what()
                  << " (continuing without it)" << std::flush << std::endl;
    }

    // ── 7. compile_model ────────────────────────────────────────────
    std::cout << "[TRACE] >>> core_->compile_model() BEGIN (device=" << device << ")" << std::flush << std::endl;

    // Print available RAM again right before compilation — this is where
    // SIGBUS typically fires on a memory-starved Pi.
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

    try {
        compiledModel_ = core_->compile_model(
            model,
            device,
            {
                ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
                ov::hint::num_requests(1)
            }
        );
    } catch (const ov::Exception& e) {
        std::cerr << "[TRACE] !!! ov::Exception in compile_model: " << e.what() << std::flush << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "[TRACE] !!! std::exception in compile_model: " << e.what() << std::flush << std::endl;
        throw;
    }
    std::cout << "[TRACE] <<< core_->compile_model() END (success)" << std::flush << std::endl;

    inferRequest_ = compiledModel_.create_infer_request();
    outputTensor_ = compiledModel_.output(0);

    const auto& inputShape = compiledModel_.input().get_shape();
    inputHeight_ = static_cast<int>(inputShape[2]);
    inputWidth_  = static_cast<int>(inputShape[3]);

    std::cout << "[TRACE] >>> input tensor pre-allocation BEGIN (" << inputWidth_ << "x" << inputHeight_ << ")" << std::flush << std::endl;
    // Pre-allocate a persistent input tensor — avoids malloc/free every frame
    inputTensor_ = ov::Tensor(ov::element::f32,
                              {1, 3,
                               static_cast<std::size_t>(inputHeight_),
                               static_cast<std::size_t>(inputWidth_)});
    inferRequest_.set_input_tensor(inputTensor_);
    std::cout << "[TRACE] <<< input tensor pre-allocation END (success)" << std::flush << std::endl;

    std::cout << "[YOLO26] Model Loaded. Input: " << inputWidth_ << "x" << inputHeight_ << "\n";
}

/* ===================== Core Logic ===================== */

cv::Mat PerceptionAgent::preprocess(const cv::Mat& frame, Letterbox& lb) {
    // 1. Convert BGR to RGB
    cv::Mat rgb;
    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);

    // 2. Calculate Letterbox (preserves aspect ratio)
    float r = std::min((float)inputWidth_ / frame.cols, (float)inputHeight_ / frame.rows);
    int new_unpad_w = std::round(frame.cols * r);
    int new_unpad_h = std::round(frame.rows * r);

    lb.scale = r;
    lb.pad_w = (inputWidth_ - new_unpad_w) / 2;
    lb.pad_h = (inputHeight_ - new_unpad_h) / 2;

    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(new_unpad_w, new_unpad_h));

    cv::Mat padded;
    cv::copyMakeBorder(resized, padded, lb.pad_h, inputHeight_ - new_unpad_h - lb.pad_h,
                       lb.pad_w, inputWidth_ - new_unpad_w - lb.pad_w,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    // 3. Normalize to [0.0, 1.0]
    cv::Mat blob;
    padded.convertTo(blob, CV_32F, 1.0 / 255.0);
    return blob;
}

std::vector<Detection> PerceptionAgent::runInference(const cv::Mat& frame) {
    Letterbox lb;
    cv::Mat blob = preprocess(frame, lb);

    // ── HWC → CHW transposition ──────────────────────────────────
    // Write directly into the pre-allocated input tensor.
    std::cout << "[TRACE] >>> inputTensor_.data<float>() BEGIN (tensor addr=" << static_cast<void*>(&inputTensor_) << ")" << std::flush << std::endl;
    float* tensorData = inputTensor_.data<float>();
    std::cout << "[TRACE]     tensorData ptr = " << static_cast<void*>(tensorData) << std::flush << std::endl;
    const float* blobData = blob.ptr<float>();
    std::cout << "[TRACE]     blobData   ptr = " << static_cast<const void*>(blobData) << std::flush << std::endl;
    const int planeSize = inputHeight_ * inputWidth_;
    std::cout << "[TRACE]     planeSize = " << planeSize << " (" << inputWidth_ << "x" << inputHeight_ << ")" << std::flush << std::endl;

    float* dst_r = tensorData;
    float* dst_g = tensorData + planeSize;
    float* dst_b = tensorData + planeSize * 2;

#if defined(__aarch64__) || defined(__ARM_NEON)
    // NEON vld3q deinterleave: loads 4 RGB pixels (12 floats) at once,
    // splits into 3 contiguous channel planes.  ~4× faster than scalar.
    int i = 0;
    const int simdEnd = planeSize - (planeSize % 4);
    for (; i < simdEnd; i += 4) {
        float32x4x3_t rgb = vld3q_f32(blobData + i * 3);
        vst1q_f32(dst_r + i, rgb.val[0]);
        vst1q_f32(dst_g + i, rgb.val[1]);
        vst1q_f32(dst_b + i, rgb.val[2]);
    }
    // Scalar tail for non-multiple-of-4 remainder
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

    // ── Async inference (OpenVINO 2025) ───────────────────────────
    // Input tensor is already bound via set_input_tensor in loadNetwork.
    std::cout << "[TRACE] >>> HWC->CHW transposition DONE, start_async() BEGIN" << std::flush << std::endl;
    try {
        inferRequest_.start_async();
        std::cout << "[TRACE]     start_async() returned, waiting..." << std::flush << std::endl;
        inferRequest_.wait();
        std::cout << "[TRACE] <<< inferRequest_.wait() returned (success)" << std::flush << std::endl;
    } catch (const ov::Exception& e) {
        std::cerr << "[TRACE] !!! ov::Exception during inference: " << e.what() << std::flush << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "[TRACE] !!! std::exception during inference: " << e.what() << std::flush << std::endl;
        throw;
    }

    return postprocess(inferRequest_.get_tensor(outputTensor_), lb, frame.size());
}

std::vector<Detection> PerceptionAgent::postprocess(const ov::Tensor& output, const Letterbox& lb, const cv::Size& origSize) {
    std::vector<Detection> detections;
    const float* data = output.data<const float>();
    const auto& shape = output.get_shape(); 

    // YOLO26 NMS-Free output is typically [1, 300, 6]
    // Indices: 0:x1, 1:y1, 2:x2, 3:y2, 4:score, 5:class_id
    int num_detections = static_cast<int>(shape[1]);

    for (int i = 0; i < num_detections; ++i) {
        const float* row = data + (i * 6);
        float confidence = row[4];

        if (confidence < confThreshold_) continue;

        // Map coordinates back to original frame (removing padding and scaling)
        float x1 = (row[0] - lb.pad_w) / lb.scale;
        float y1 = (row[1] - lb.pad_h) / lb.scale;
        float x2 = (row[2] - lb.pad_w) / lb.scale;
        float y2 = (row[3] - lb.pad_h) / lb.scale;

        // Clip to original frame boundaries
        x1 = std::clamp(x1, 0.0f, (float)origSize.width);
        y1 = std::clamp(y1, 0.0f, (float)origSize.height);
        x2 = std::clamp(x2, 0.0f, (float)origSize.width);
        y2 = std::clamp(y2, 0.0f, (float)origSize.height);

        Detection det;
        det.confidence = confidence;
        det.classId = static_cast<int>(row[5]);
        det.boundingBox = cv::Rect(cv::Point(static_cast<int>(x1), static_cast<int>(y1)), 
                                   cv::Point(static_cast<int>(x2), static_cast<int>(y2)));

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
#include "analytical.hpp"
#include "roi_utils.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <csetjmp>
#include <csignal>
#include <opencv2/imgproc.hpp>
#include <cstring>
#include <limits>
#include <cmath>
#include <numeric>

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
    const char msg[] = "\n[FATAL] SIGBUS outside guarded compile_model region\n";
#ifdef __linux__
    (void)::write(STDERR_FILENO, msg, sizeof(msg) - 1);
#endif
    ::_exit(128 + sig);
}

namespace vigia {

/* ===================== Constructor ===================== */

AnalyticalAgent::AnalyticalAgent(
    const std::string& modelXmlPath,
    const std::string& device
) : core_(&ownedCore_) {
    loadNetwork(modelXmlPath, device);
}

AnalyticalAgent::AnalyticalAgent(
    ov::Core& sharedCore,
    const std::string& modelXmlPath,
    const std::string& device
) : core_(&sharedCore) {
    loadNetwork(modelXmlPath, device);
}

/* ===================== Network Loading ===================== */

void AnalyticalAgent::loadNetwork(
    const std::string& modelXmlPath,
    const std::string& device
) {
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
            std::cerr << "[MiDaS] FATAL: Cannot open " << label << ": " << path << std::flush << std::endl;
            throw std::runtime_error("Cannot open model file: " + path);
        }
        const auto size = f.tellg();
        if (size <= 0) {
            std::cerr << "[MiDaS] FATAL: " << label << " is empty (0 bytes): " << path << std::flush << std::endl;
            throw std::runtime_error("Model file is empty: " + path);
        }
        std::cout << "[MiDaS] " << label << " OK  size=" << size << " bytes  path=" << path << std::flush << std::endl;
    };
    validateFile(modelXmlPath, ".xml");
    validateFile(modelBinPath, ".bin");

    // ── 3. RAM availability: read /proc/meminfo before heavy allocations
#ifdef __linux__
    {
        std::ifstream meminfo("/proc/meminfo");
        if (meminfo.is_open()) {
            std::string line;
            std::cout << "[MiDaS] /proc/meminfo snapshot before model load:" << std::flush << std::endl;
            while (std::getline(meminfo, line)) {
                if (line.rfind("MemTotal", 0) == 0 ||
                    line.rfind("MemFree", 0) == 0 ||
                    line.rfind("MemAvailable", 0) == 0 ||
                    line.rfind("SwapTotal", 0) == 0 ||
                    line.rfind("SwapFree", 0) == 0) {
                    std::cout << "[MiDaS]   " << line << std::flush << std::endl;
                }
            }
        } else {
            std::cerr << "[MiDaS] WARNING: Could not open /proc/meminfo" << std::flush << std::endl;
        }
    }
#endif

    // ── 4. read_model ───────────────────────────────────────────────
    std::cout << "[TRACE] >>> MiDaS core_->read_model() BEGIN" << std::flush << std::endl;
    std::cout << "[TRACE]     .xml = " << modelXmlPath << std::flush << std::endl;
    std::cout << "[TRACE]     .bin = " << modelBinPath << std::flush << std::endl;

    std::shared_ptr<ov::Model> model;
    try {
        model = core_->read_model(modelXmlPath);
    } catch (const ov::Exception& e) {
        std::cerr << "[TRACE] !!! ov::Exception in MiDaS read_model: " << e.what() << std::flush << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "[TRACE] !!! std::exception in MiDaS read_model: " << e.what() << std::flush << std::endl;
        throw;
    }
    std::cout << "[TRACE] <<< MiDaS core_->read_model() END (success)" << std::flush << std::endl;

    model->get_parameters()[0]->set_layout("NCHW");
    ov::set_batch(model, 1);

    // ── 5. Explicit plugin inspection: check CPU supported properties
    std::cout << "[TRACE] >>> Inspecting MiDaS CPU plugin supported_properties" << std::flush << std::endl;
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
        std::cout << "[MiDaS] CPU plugin properties (" << props.size() << " entries)."
                  << (foundAcl ? " ACL reference found." : " No explicit ACL property.")
                  << std::flush << std::endl;

        const std::string fullName = core_->get_property(device, ov::device::full_name);
        std::cout << "[MiDaS] Device full_name: " << fullName << std::flush << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[MiDaS] WARNING: Could not query CPU properties: " << e.what() << std::flush << std::endl;
    }

    // ── 6. Disable mmap: forces standard RAM allocation instead of
    //    memory-mapping the .bin weights file.  Fixes SIGBUS on SD-card
    //    based systems (Raspberry Pi).
    std::cout << "[TRACE] >>> Disabling mmap for MiDaS CPU plugin" << std::flush << std::endl;
    try {
        core_->set_property("CPU", ov::enable_mmap(false));
        std::cout << "[TRACE] <<< ov::enable_mmap(false) set successfully" << std::flush << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[MiDaS] WARNING: Could not set enable_mmap(false): " << e.what()
                  << " (continuing without it)" << std::flush << std::endl;
    }

    // ── 7. compile_model (with SIGBUS recovery) ──────────────────
    std::cout << "[TRACE] >>> MiDaS core_->compile_model() BEGIN (device=" << device << ")" << std::flush << std::endl;

#ifdef __linux__
    {
        std::ifstream meminfo("/proc/meminfo");
        if (meminfo.is_open()) {
            std::string line;
            while (std::getline(meminfo, line)) {
                if (line.rfind("MemAvailable", 0) == 0) {
                    std::cout << "[MiDaS] PRE-COMPILE " << line << std::flush << std::endl;
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
            ::sigaction(SIGBUS, &oldSa, nullptr);
            std::cerr << "\n[MiDaS] ══════════════════════════════════════════════════════\n"
                      << "[MiDaS] SIGBUS caught during compile_model()!\n"
                      << "[MiDaS] Root cause: OpenVINO ARM CPU plugin lacks ACL (Arm Compute Library).\n"
                      << "[MiDaS] The reference plugin uses unaligned memory accesses that\n"
                      << "[MiDaS] trigger SIGBUS on strict-alignment ARM64 hardware.\n"
                      << "[MiDaS] Fix: rebuild OpenVINO from source with -DENABLE_KLEIDIAI=ON\n"
                      << "[MiDaS] Continuing in DEGRADED mode (depth estimation disabled).\n"
                      << "[MiDaS] ══════════════════════════════════════════════════════\n"
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
                    ov::hint::num_requests(1),
                    ov::inference_num_threads(2),
                    ov::hint::inference_precision(ov::element::f32)
                }
            );
        } catch (const ov::Exception& e) {
            g_compileJmpActive = 0;
            ::sigaction(SIGBUS, &oldSa, nullptr);
            std::cerr << "[TRACE] !!! ov::Exception in MiDaS compile_model: " << e.what() << std::flush << std::endl;
            modelLoaded_ = false;
            return;
        } catch (const std::exception& e) {
            g_compileJmpActive = 0;
            ::sigaction(SIGBUS, &oldSa, nullptr);
            std::cerr << "[TRACE] !!! std::exception in MiDaS compile_model: " << e.what() << std::flush << std::endl;
            modelLoaded_ = false;
            return;
        }

        g_compileJmpActive = 0;
        ::sigaction(SIGBUS, &oldSa, nullptr);
    }
    std::cout << "[TRACE] <<< MiDaS core_->compile_model() END (success)" << std::flush << std::endl;

    inferRequest_ = compiledModel_.create_infer_request();
    outputTensor_ = compiledModel_.output(0);
    modelLoaded_ = true;

    const auto& inputShape = compiledModel_.input().get_shape();
    inputHeight_ = inputShape[2];
    inputWidth_ = inputShape[3];

    std::cout << "[TRACE]     MiDaS input: " << inputWidth_ << "x" << inputHeight_ << std::flush << std::endl;
    chwBuffer_.resize(inputWidth_ * inputHeight_ * 3);

    // Pre-allocate preprocessing buffers
    midasResized_.create(static_cast<int>(inputHeight_), static_cast<int>(inputWidth_), CV_8UC3);
    midasBlob_.create(static_cast<int>(inputHeight_), static_cast<int>(inputWidth_), CV_32FC3);

    // Wrap output tensor as a persistent cv::Mat header (zero-copy depth output)
    const ov::Tensor outTensor = inferRequest_.get_tensor(outputTensor_);
    depthOutput_ = cv::Mat(static_cast<int>(inputHeight_), static_cast<int>(inputWidth_),
                           CV_32F, const_cast<float*>(outTensor.data<float>()));
}

/* ===================== Phase 1: MiDaS Inference ===================== */

cv::Mat AnalyticalAgent::runInference(const cv::Mat& frame) {
    if (!modelLoaded_) return cv::Mat();

    // Resize into pre-allocated buffer, then convert in-place — no intermediate alloc
    cv::resize(frame, midasResized_, cv::Size(inputWidth_, inputHeight_));
    midasResized_.convertTo(midasBlob_, CV_32F, 1.0f / 255.0f);

    float* const chw = chwBuffer_.data();
    const float* blobData = midasBlob_.ptr<float>();
    const int planeSize = static_cast<int>(inputHeight_ * inputWidth_);

    float* dst_r = chw;
    float* dst_g = chw + planeSize;
    float* dst_b = chw + planeSize * 2;

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

    // ── Pre-allocated tensor wrapping chwBuffer_ ──────────────────
    // Wraps the existing buffer without copying.  set_input_tensor
    // is only called once (when inputTensorBound_ is false).
    if (!inputTensorBound_) {
        midasInputTensor_ = ov::Tensor(
            ov::element::f32,
            {1, 3, inputHeight_, inputWidth_},
            chw
        );
        inferRequest_.set_input_tensor(midasInputTensor_);
        inputTensorBound_ = true;
    }

    try {
        inferRequest_.infer();
    } catch (const ov::Exception& e) {
        std::cerr << "[MiDaS] ov::Exception during infer: " << e.what() << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "[MiDaS] std::exception during infer: " << e.what() << std::endl;
        throw;
    }

    // depthOutput_ is a cv::Mat header wrapping the output tensor buffer.
    // Return a clone so the caller owns stable data independent of the next infer().
    return depthOutput_.clone();
}

cv::Rect AnalyticalAgent::scaleROIToDepth(
    const cv::Rect& roi,
    const cv::Size& originalSize,
    const cv::Size& depthSize
) const {
    if (originalSize.width <= 0 || originalSize.height <= 0)
        return {};

    if (depthSize.width <= 0 || depthSize.height <= 0)
        return {};

    const float scaleX = static_cast<float>(depthSize.width) /
                         static_cast<float>(originalSize.width);
    const float scaleY = static_cast<float>(depthSize.height) /
                         static_cast<float>(originalSize.height);

    cv::Rect scaled(
        static_cast<int>(std::round(roi.x * scaleX)),
        static_cast<int>(std::round(roi.y * scaleY)),
        static_cast<int>(std::round(roi.width * scaleX)),
        static_cast<int>(std::round(roi.height * scaleY))
    );

    cv::Rect bounds(0, 0, depthSize.width, depthSize.height);
    return scaled & bounds;
}

/* ===================== Phase 2a: ROI Depth Prep ===================== */

cv::Mat AnalyticalAgent::extractDepthROI(
    const cv::Mat& depthMap,
    const cv::Rect& roi
) {
    const cv::Rect boundedROI = clampROIToMat(roi, depthMap);
    if (boundedROI.empty()) return {};

    // copyTo reuses roiScratch_ allocation when size matches — no heap alloc
    depthMap(boundedROI).copyTo(roiScratch_);
    cv::GaussianBlur(roiScratch_, roiScratch_, cv::Size(3, 3), 0);
    cv::normalize(roiScratch_, roiScratch_, 0.0f, 1.0f, cv::NORM_MINMAX);
    return roiScratch_;
}

/* ===================== Phase 2b: Plane Residual Analysis ===================== */

DepthResidualStats AnalyticalAgent::computeDepthResiduals(
    const cv::Mat& roiDepth
) const {
    DepthResidualStats stats{};

    if (roiDepth.empty() || roiDepth.type() != CV_32F)
        return stats;

    const int rows = roiDepth.rows;
    const int cols = roiDepth.cols;
    const int N = rows * cols;
    const float fN = static_cast<float>(N);

    float sumX = 0, sumY = 0, sumZ = 0;
    float sumXX = 0, sumYY = 0, sumXY = 0;
    float sumXZ = 0, sumYZ = 0;

    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x) {
            const float z = roiDepth.at<float>(y, x);
            const float fx = static_cast<float>(x);
            const float fy = static_cast<float>(y);
            sumX += fx; sumY += fy; sumZ += z;
            sumXX += fx * fx; sumYY += fy * fy;
            sumXY += fx * fy;
            sumXZ += fx * z; sumYZ += fy * z;
        }

    const float denom =
        (sumXX * sumYY * fN +
         2.0f * sumX * sumY * sumXY -
         sumXX * sumY * sumY -
         sumYY * sumX * sumX -
         sumXY * sumXY * fN);

    float a = 0, b = 0, c = 0;

    if (std::abs(denom) > 1e-6f) {
        a = (sumXZ * sumYY * fN +
             sumX * sumY * sumYZ +
             sumXY * sumY * sumZ -
             sumXZ * sumY * sumY -
             sumYY * sumX * sumZ -
             sumXY * sumYZ * fN) / denom;

        b = (sumXX * sumYZ * fN +
             sumX * sumY * sumXZ +
             sumX * sumXY * sumZ -
             sumXX * sumY * sumZ -
             sumYZ * sumX * sumX -
             sumXY * sumXZ * fN) / denom;

        c = (sumZ - a * sumX - b * sumY) / fN;
    }

    float mean = 0.0f;
    float minVal = std::numeric_limits<float>::max();
    float var = 0.0f;

    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x) {
            const float obs = roiDepth.at<float>(y, x);
            const float r = obs - (a * static_cast<float>(x) + b * static_cast<float>(y) + c);
            mean += r;
            if (r < minVal) minVal = r;
            var += r * r;
        }

    mean /= fN;
    var = (var / fN) - (mean * mean);

    stats.meanResidual = mean;
    stats.minResidual  = minVal;
    stats.stdResidual  = std::sqrt(std::max(0.0f, var));

    return stats;
}

/* ===================== Phase 3: Geometry + Temporal Metrics ===================== */

DepthGeometryMetrics AnalyticalAgent::computeGeometryMetrics(
    const cv::Mat& roiDepth,
    const DepthResidualStats& residuals
) {
    DepthGeometryMetrics metrics{};

    if (roiDepth.empty())
        return metrics;

    /* Instantaneous geometry */
    metrics.depressionScore =
        std::max(0.0f, -residuals.minResidual);

    metrics.roughness = residuals.stdResidual;

    return metrics;
}

} // namespace vigia

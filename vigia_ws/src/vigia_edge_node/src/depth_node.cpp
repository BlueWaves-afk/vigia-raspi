#include "depth_node.hpp"
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <chrono>
#if defined(__aarch64__) || defined(__ARM_NEON)
#  include <arm_neon.h>
#endif

using namespace std::chrono_literals;

static constexpr int kMidasSize = 256;

DepthNode::DepthNode(const rclcpp::NodeOptions & options)
: Node("depth_node", options),
  env_(ORT_LOGGING_LEVEL_WARNING, "vigia_depth"),
  mem_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
  last_temp_read_(std::chrono::steady_clock::now() - 2s)
{
    declare_parameter("model_path",               "models/midasv21/midas_v21_small_256.onnx");
    declare_parameter("midas_stride",             3);
    declare_parameter("temp_warn_threshold_c",    75.0);
    declare_parameter("temp_critical_threshold_c",85.0);

    model_path_   = get_parameter("model_path").as_string();
    midas_stride_ = get_parameter("midas_stride").as_int();
    temp_warn_c_  = get_parameter("temp_warn_threshold_c").as_double();
    temp_crit_c_  = get_parameter("temp_critical_threshold_c").as_double();

    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    opts.AddConfigEntry("session.disable_quant_qdq_cleanup", "1");  // keep FP32
    opts.SetIntraOpNumThreads(2);
    opts.SetInterOpNumThreads(1);
    opts.DisableMemPattern();

    session_ = std::make_unique<Ort::Session>(env_, model_path_.c_str(), opts);

    Ort::AllocatorWithDefaultOptions alloc;
    input_name_str_  = session_->GetInputNameAllocated(0, alloc).get();
    output_name_str_ = session_->GetOutputNameAllocated(0, alloc).get();
    input_name_      = input_name_str_.c_str();
    output_name_     = output_name_str_.c_str();

    // FP32 normalized input: 3 × 256 × 256.
    midas_input_buf_.resize(3 * kMidasSize * kMidasSize);
    midas_output_buf_.resize(kMidasSize * kMidasSize);

    pub_ = create_publisher<vigia_msgs::msg::DepthMap>(
        "/vigia/depth", vigia::qos::inference_results());
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/vigia/camera/image_raw", vigia::qos::camera_frames(),
        std::bind(&DepthNode::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "DepthNode ready: model=%s stride=%d",
                model_path_.c_str(), midas_stride_);
}

float DepthNode::read_cpu_temp_celsius()
{
    auto now = std::chrono::steady_clock::now();
    if (now - last_temp_read_ < std::chrono::seconds(1)) return cached_temp_c_;

    std::ifstream f("/sys/class/thermal/thermal_zone0/temp");
    if (!f.is_open()) return cached_temp_c_;
    int raw = 0;
    f >> raw;
    cached_temp_c_  = static_cast<float>(raw) / 1000.0f;
    last_temp_read_ = now;

    // Thermal adaptive stride.
    if (cached_temp_c_ >= static_cast<float>(temp_crit_c_)) {
        midas_stride_ = 6;
    } else if (cached_temp_c_ >= static_cast<float>(temp_warn_c_)) {
        midas_stride_ = 4;
    } else {
        midas_stride_ = get_parameter("midas_stride").as_int();
    }
    return cached_temp_c_;
}

void DepthNode::on_image(std::shared_ptr<const sensor_msgs::msg::Image> msg)
{
    ++frame_counter_;
    read_cpu_temp_celsius();

    if ((frame_counter_ % static_cast<uint64_t>(midas_stride_)) != 0) return;

    auto t0 = std::chrono::steady_clock::now();

    // Resize to 256×256 FP32 and normalize to [0,1].
    cv::Mat src(static_cast<int>(msg->height), static_cast<int>(msg->width),
                CV_8UC3, const_cast<uint8_t*>(msg->data.data()));
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(kMidasSize, kMidasSize), 0, 0, cv::INTER_LINEAR);

    // HWC uint8 BGR → CHW FP32 RGB, normalize [0,1].
    // NEON vld3q_f32 deinterleave: processes 4 pixels (12 floats) per iteration
    // (~4× faster than scalar on Cortex-A76). Ported from analytical.cpp:284-306.
    cv::Mat blob;
    resized.convertTo(blob, CV_32F, 1.0f / 255.0f);
    const int pixels = kMidasSize * kMidasSize;
    float* ch_r = midas_input_buf_.data();
    float* ch_g = midas_input_buf_.data() + pixels;
    float* ch_b = midas_input_buf_.data() + pixels * 2;
    const float* src = blob.ptr<float>();
#if defined(__aarch64__) || defined(__ARM_NEON)
    const int simd_end = pixels - (pixels % 4);
    int i = 0;
    for (; i < simd_end; i += 4) {
        float32x4x3_t bgr = vld3q_f32(src + i * 3);
        vst1q_f32(ch_r + i, bgr.val[2]);  // R
        vst1q_f32(ch_g + i, bgr.val[1]);  // G
        vst1q_f32(ch_b + i, bgr.val[0]);  // B
    }
    for (; i < pixels; ++i) {
        ch_r[i] = src[i * 3 + 2];
        ch_g[i] = src[i * 3 + 1];
        ch_b[i] = src[i * 3 + 0];
    }
#else
    for (int i = 0; i < pixels; ++i) {
        ch_r[i] = src[i * 3 + 2];
        ch_g[i] = src[i * 3 + 1];
        ch_b[i] = src[i * 3 + 0];
    }
#endif

    std::array<int64_t, 4> in_shape{1, 3, kMidasSize, kMidasSize};
    auto in_tensor = Ort::Value::CreateTensor<float>(
        mem_info_,
        midas_input_buf_.data(), midas_input_buf_.size(),
        in_shape.data(), in_shape.size());

    auto outputs = session_->Run(
        Ort::RunOptions{nullptr},
        &input_name_, &in_tensor, 1,
        &output_name_, 1);

    auto t1  = std::chrono::steady_clock::now();
    float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

    // Zero-copy tensor read via raw pointer — no clone().
    const float* depth_ptr = outputs[0].GetTensorData<float>();

    auto depth_msg = std::make_unique<vigia_msgs::msg::DepthMap>();
    depth_msg->header              = msg->header;
    depth_msg->frame_id            = static_cast<uint32_t>(frame_counter_);
    depth_msg->width               = kMidasSize;
    depth_msg->height              = kMidasSize;
    depth_msg->inference_latency_ms = ms;
    depth_msg->data.assign(depth_ptr, depth_ptr + static_cast<size_t>(pixels));

    pub_->publish(std::move(depth_msg));
}

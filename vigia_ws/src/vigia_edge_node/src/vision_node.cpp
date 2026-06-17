#include "vision_node.hpp"
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cstring>

VisionNode::VisionNode(const rclcpp::NodeOptions & options)
: Node("vision_node", options),
  env_(ORT_LOGGING_LEVEL_WARNING, "vigia_vision"),
  mem_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
    declare_parameter("model_path",         "models/yolov26/yolov26_nano_int8.onnx");
    declare_parameter("latent_layer_name",  "");
    declare_parameter("conf_threshold",     0.25);
    declare_parameter("nms_iou_threshold",  0.45);
    declare_parameter("input_width",        320);
    declare_parameter("input_height",       320);

    model_path_         = get_parameter("model_path").as_string();
    latent_layer_name_  = get_parameter("latent_layer_name").as_string();
    conf_threshold_     = static_cast<float>(get_parameter("conf_threshold").as_double());
    nms_iou_threshold_  = static_cast<float>(get_parameter("nms_iou_threshold").as_double());
    input_width_        = get_parameter("input_width").as_int();
    input_height_       = get_parameter("input_height").as_int();

    Ort::SessionOptions session_opts;
    session_opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_opts.AddConfigEntry("session.set_denormal_as_zero", "1");
    session_opts.SetIntraOpNumThreads(1);
    session_opts.SetInterOpNumThreads(1);
    session_opts.DisableMemPattern();
    // KleidiAI ACL EP for INT8 UDOT micro-kernels — requires ONNX RT built with ACL support.
    // Falls back to CPU EP gracefully if ACL is not available in this build.
    // OrtSessionOptionsAppendExecutionProvider_ACL(session_opts, 1);

    session_ = std::make_unique<Ort::Session>(env_, model_path_.c_str(), session_opts);

    // Enumerate I/O node names.
    Ort::AllocatorWithDefaultOptions alloc;
    size_t n_inputs = session_->GetInputCount();
    for (size_t i = 0; i < n_inputs; ++i) {
        auto name = session_->GetInputNameAllocated(i, alloc);
        output_names_storage_.emplace_back(name.get());  // re-use storage vector for inputs too
    }
    input_names_.push_back(output_names_storage_[0].c_str());

    // Primary output (detection head) always at index 0.
    auto det_name = session_->GetOutputNameAllocated(0, alloc);
    output_names_storage_.emplace_back(det_name.get());
    output_names_.push_back(output_names_storage_.back().c_str());

    // Optional latent output — only add if configured.
    if (!latent_layer_name_.empty()) {
        output_names_storage_.push_back(latent_layer_name_);
        output_names_.push_back(output_names_storage_.back().c_str());
    }

    // Pre-allocate CHW buffer: 3 * H * W bytes for INT8 input.
    letterbox_buf_.resize(static_cast<size_t>(input_width_ * input_height_ * 3));
    chw_input_buf_.resize(static_cast<size_t>(input_width_ * input_height_ * 3));

    pub_det_ = create_publisher<vigia_msgs::msg::DetectionArray>(
        "/vigia/detections", vigia::qos::inference_results());
    pub_lat_ = create_publisher<vigia_msgs::msg::SpatialLatent>(
        "/vigia/spatial_latent", vigia::qos::inference_results());

    sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/vigia/camera/image_raw", vigia::qos::camera_frames(),
        std::bind(&VisionNode::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "VisionNode ready: model=%s latent_layer=%s",
                model_path_.c_str(),
                latent_layer_name_.empty() ? "(none)" : latent_layer_name_.c_str());
}

void VisionNode::neon_bgr_hwc_to_rgb_chw(const uint8_t* src, uint8_t* dst,
                                           int w, int h)
{
    // NEON vld3q_u8: deinterleave 48 bytes (16 BGR pixels) in one instruction.
    // Produces separate B, G, R channels → write R, G, B planes to CHW layout.
    const int total_pixels = w * h;
    const int vec_count    = total_pixels / 16;
    const int remainder    = total_pixels % 16;

    uint8_t* ch_r = dst;
    uint8_t* ch_g = dst + total_pixels;
    uint8_t* ch_b = dst + total_pixels * 2;

    for (int i = 0; i < vec_count; ++i) {
        uint8x16x3_t bgr = vld3q_u8(src + i * 48);
        vst1q_u8(ch_r + i * 16, bgr.val[2]);  // R channel
        vst1q_u8(ch_g + i * 16, bgr.val[1]);  // G channel
        vst1q_u8(ch_b + i * 16, bgr.val[0]);  // B channel
    }
    // Scalar tail for non-multiple-of-16 resolutions.
    for (int i = vec_count * 16; i < total_pixels; ++i) {
        ch_r[i] = src[i * 3 + 2];
        ch_g[i] = src[i * 3 + 1];
        ch_b[i] = src[i * 3 + 0];
    }
}

void VisionNode::on_image(std::shared_ptr<const sensor_msgs::msg::Image> msg)
{
    auto t0 = std::chrono::steady_clock::now();

    // Resize to model input resolution using pre-allocated buffer.
    cv::Mat src(static_cast<int>(msg->height), static_cast<int>(msg->width),
                CV_8UC3, const_cast<uint8_t*>(msg->data.data()));
    cv::Mat resized(input_height_, input_width_, CV_8UC3, letterbox_buf_.data());
    cv::resize(src, resized, resized.size(), 0, 0, cv::INTER_LINEAR);

    // NEON single-pass BGR HWC → RGB CHW transpose.
    neon_bgr_hwc_to_rgb_chw(letterbox_buf_.data(), chw_input_buf_.data(),
                             input_width_, input_height_);

    // Build ONNX input tensor (wraps buffer — zero-copy).
    std::array<int64_t, 4> shape{1, 3, input_height_, input_width_};
    auto input_tensor = Ort::Value::CreateTensor<uint8_t>(
        mem_info_,
        chw_input_buf_.data(), chw_input_buf_.size(),
        shape.data(), shape.size());

    // Run inference — detection head only (latent added if configured).
    auto outputs = session_->Run(
        Ort::RunOptions{nullptr},
        input_names_.data(), &input_tensor, 1,
        output_names_.data(), output_names_.size());

    auto t1  = std::chrono::steady_clock::now();
    float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

    // Publish detection results.
    auto det_msg = std::make_unique<vigia_msgs::msg::DetectionArray>();
    det_msg->header            = msg->header;
    det_msg->frame_id          = static_cast<uint32_t>(std::stoul(msg->header.frame_id));
    det_msg->inference_latency_ms = ms;
    // TODO: postprocess outputs[0] with NMS → fill det_msg->detections
    pub_det_->publish(std::move(det_msg));

    // Publish spatial latent S_t.
    auto lat_msg = std::make_unique<vigia_msgs::msg::SpatialLatent>();
    lat_msg->header           = msg->header;
    lat_msg->frame_id         = static_cast<uint32_t>(std::stoul(msg->header.frame_id));
    lat_msg->source_layer_name = latent_layer_name_;

    if (outputs.size() > 1) {
        const float* latent_data = outputs[1].GetTensorData<float>();
        auto latent_shape        = outputs[1].GetTensorTypeAndShapeInfo().GetShape();
        size_t latent_elems      = 1;
        for (auto d : latent_shape) latent_elems *= static_cast<size_t>(d);
        lat_msg->latent_vector.assign(latent_data, latent_data + latent_elems);
    }
    pub_lat_->publish(std::move(lat_msg));
}

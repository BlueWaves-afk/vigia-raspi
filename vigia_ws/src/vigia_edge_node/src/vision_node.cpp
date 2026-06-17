#include "vision_node.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <numeric>

// ── YOLO output layout enum ────────────────────────────────────────────────
// ONNX exports vary: [1,N,6], [1,6,N], [N,5], [5,N] all occur in the wild.
enum class YoloLayout { kUnknown, kNBy6, k6ByN, kNBy5, k5ByN };

// ── NMS (ported from src/perception.cpp) ──────────────────────────────────
// INT8 quantization can cause adjacent detections to merge; this separates them.
static std::vector<int> nmsBoxes(const std::vector<cv::Rect>& boxes,
                                  const std::vector<float>&    scores,
                                  float scoreThresh, float iouThresh)
{
    std::vector<int> indices;
    if (boxes.empty()) return indices;

    std::vector<int> order(boxes.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&scores](int a, int b){ return scores[a] > scores[b]; });

    std::vector<bool> suppressed(boxes.size(), false);
    for (size_t i = 0; i < order.size(); ++i) {
        const int idx = order[i];
        if (suppressed[idx] || scores[idx] < scoreThresh) continue;
        indices.push_back(idx);
        const cv::Rect& bA = boxes[idx];
        const float aA = static_cast<float>(bA.area());
        for (size_t j = i + 1; j < order.size(); ++j) {
            const int jdx = order[j];
            if (suppressed[jdx]) continue;
            const cv::Rect& bB = boxes[jdx];
            const float inter = static_cast<float>((bA & bB).area());
            const float aB    = static_cast<float>(bB.area());
            if (inter / (aA + aB - inter + 1e-6f) > iouThresh)
                suppressed[jdx] = true;
        }
    }
    return indices;
}

// ── Letterbox struct (matches perception.cpp Letterbox) ───────────────────
struct Letterbox { float scale{1.0f}; int pad_w{0}; int pad_h{0}; };

// ──────────────────────────────────────────────────────────────────────────

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

    model_path_        = get_parameter("model_path").as_string();
    latent_layer_name_ = get_parameter("latent_layer_name").as_string();
    conf_threshold_    = static_cast<float>(get_parameter("conf_threshold").as_double());
    nms_iou_threshold_ = static_cast<float>(get_parameter("nms_iou_threshold").as_double());
    input_width_       = get_parameter("input_width").as_int();
    input_height_      = get_parameter("input_height").as_int();

    Ort::SessionOptions session_opts;
    session_opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_opts.AddConfigEntry("session.set_denormal_as_zero", "1");
    session_opts.SetIntraOpNumThreads(1);
    session_opts.SetInterOpNumThreads(1);
    session_opts.DisableMemPattern();
    // ACL EP for KleidiAI INT8 UDOT — uncomment after ARM Compute Library build:
    // OrtSessionOptionsAppendExecutionProvider_ACL(session_opts, 1);

    session_ = std::make_unique<Ort::Session>(env_, model_path_.c_str(), session_opts);

    Ort::AllocatorWithDefaultOptions alloc;
    input_name_str_  = session_->GetInputNameAllocated(0, alloc).get();
    input_names_str_.push_back(input_name_str_);
    input_names_.push_back(input_names_str_[0].c_str());

    // Primary detection head — always output[0].
    output_names_storage_.push_back(session_->GetOutputNameAllocated(0, alloc).get());
    output_names_.push_back(output_names_storage_.back().c_str());

    // Optional spatial latent S_t — only if configured.
    if (!latent_layer_name_.empty()) {
        output_names_storage_.push_back(latent_layer_name_);
        output_names_.push_back(output_names_storage_.back().c_str());
    }

    // Pre-allocate: letterbox buffer (RGB HWC uint8) + CHW transposed buffer.
    letterbox_buf_.resize(static_cast<size_t>(input_width_ * input_height_ * 3));
    chw_input_buf_.resize(static_cast<size_t>(input_width_ * input_height_ * 3));
    // Pre-allocate NMS scratch — cleared per frame, capacity kept.
    nms_boxes_.reserve(256);
    nms_scores_.reserve(256);
    nms_class_ids_.reserve(256);

    pub_det_ = create_publisher<vigia_msgs::msg::DetectionArray>(
        "/vigia/detections", vigia::qos::inference_results());
    pub_lat_ = create_publisher<vigia_msgs::msg::SpatialLatent>(
        "/vigia/spatial_latent", vigia::qos::inference_results());
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/vigia/camera/image_raw", vigia::qos::camera_frames(),
        std::bind(&VisionNode::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "VisionNode ready: model=%s conf=%.2f iou=%.2f",
                model_path_.c_str(), conf_threshold_, nms_iou_threshold_);
}

// ── NEON single-pass BGR HWC → RGB CHW (~8µs for 320×320 uint8) ─────────
void VisionNode::neon_bgr_hwc_to_rgb_chw(const uint8_t* src, uint8_t* dst, int w, int h)
{
    const int total   = w * h;
    const int vec_cnt = total / 16;
    uint8_t* ch_r = dst;
    uint8_t* ch_g = dst + total;
    uint8_t* ch_b = dst + total * 2;

    for (int i = 0; i < vec_cnt; ++i) {
        uint8x16x3_t bgr = vld3q_u8(src + i * 48);
        vst1q_u8(ch_r + i * 16, bgr.val[2]);  // R
        vst1q_u8(ch_g + i * 16, bgr.val[1]);  // G
        vst1q_u8(ch_b + i * 16, bgr.val[0]);  // B
    }
    for (int i = vec_cnt * 16; i < total; ++i) {
        ch_r[i] = src[i * 3 + 2];
        ch_g[i] = src[i * 3 + 1];
        ch_b[i] = src[i * 3 + 0];
    }
}

// ── Letterbox resize (ported from perception.cpp preprocess()) ────────────
// Resizes preserving aspect ratio, pads with 114 (standard YOLO grey).
// Tracks scale + pad for inverse coordinate mapping in postprocess.
static Letterbox letterbox_resize(const cv::Mat& src, cv::Mat& dst_buf,
                                   uint8_t* out_rgb, int tw, int th)
{
    Letterbox lb;
    lb.scale = std::min(static_cast<float>(tw) / src.cols,
                        static_cast<float>(th) / src.rows);
    const int nw = static_cast<int>(std::round(src.cols * lb.scale));
    const int nh = static_cast<int>(std::round(src.rows * lb.scale));
    lb.pad_w = (tw - nw) / 2;
    lb.pad_h = (th - nh) / 2;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(nw, nh));
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);

    // Letterbox into pre-allocated HWC buffer.
    cv::Mat padded(th, tw, CV_8UC3, out_rgb);
    padded.setTo(cv::Scalar(114, 114, 114));
    resized.copyTo(padded(cv::Rect(lb.pad_w, lb.pad_h, nw, nh)));
    return lb;
}

// ── Full YOLO postprocess (ported from perception.cpp postprocess()) ──────
std::vector<vigia_msgs::msg::Detection> VisionNode::postprocess(
    const Ort::Value&  output_tensor,
    const Letterbox&   lb,
    const cv::Size&    orig_size)
{
    const float* data = output_tensor.GetTensorData<float>();
    auto shape = output_tensor.GetTensorTypeAndShapeInfo().GetShape();

    // Auto-detect output layout — ONNX exports vary by ultralytics version.
    YoloLayout layout = YoloLayout::kUnknown;
    size_t num_det = 0;
    if (shape.size() == 3) {
        const size_t a = static_cast<size_t>(shape[1]);
        const size_t b = static_cast<size_t>(shape[2]);
        if      (b == 6) { layout = YoloLayout::kNBy6; num_det = a; }
        else if (a == 6) { layout = YoloLayout::k6ByN; num_det = b; }
        else if (b == 5) { layout = YoloLayout::kNBy5; num_det = a; }
        else if (a == 5) { layout = YoloLayout::k5ByN; num_det = b; }
    } else if (shape.size() == 2) {
        const size_t a = static_cast<size_t>(shape[0]);
        const size_t b = static_cast<size_t>(shape[1]);
        if      (b == 6) { layout = YoloLayout::kNBy6; num_det = a; }
        else if (a == 6) { layout = YoloLayout::k6ByN; num_det = b; }
        else if (b == 5) { layout = YoloLayout::kNBy5; num_det = a; }
        else if (a == 5) { layout = YoloLayout::k5ByN; num_det = b; }
    }
    if (layout == YoloLayout::kUnknown || num_det == 0) return {};

    const auto read = [&](size_t det, size_t field) -> float {
        switch (layout) {
            case YoloLayout::kNBy6: return data[det * 6 + field];
            case YoloLayout::kNBy5: return data[det * 5 + field];
            case YoloLayout::k6ByN: return data[field * num_det + det];
            case YoloLayout::k5ByN: return data[field * num_det + det];
            default: return 0.0f;
        }
    };

    nms_boxes_.clear(); nms_scores_.clear(); nms_class_ids_.clear();

    for (size_t i = 0; i < num_det; ++i) {
        float x1, y1, x2, y2, conf, cls;
        if (layout == YoloLayout::kNBy6 || layout == YoloLayout::k6ByN) {
            x1 = read(i,0); y1 = read(i,1); x2 = read(i,2); y2 = read(i,3);
            conf = read(i,4); cls = read(i,5);
        } else {
            const float cx = read(i,0), cy = read(i,1);
            const float bw = read(i,2), bh = read(i,3);
            conf = read(i,4); cls = 0.0f;
            x1 = cx - bw*0.5f; y1 = cy - bh*0.5f;
            x2 = cx + bw*0.5f; y2 = cy + bh*0.5f;
        }
        if (conf < conf_threshold_) continue;

        // Handle normalized [0,1] coordinates (some ONNX exports).
        const bool normalized = (x1>=0&&y1>=0&&x2<=2.0f&&y2<=2.0f);
        if (normalized) {
            x1 *= input_width_; x2 *= input_width_;
            y1 *= input_height_; y2 *= input_height_;
        }

        // Inverse letterbox: remove padding and undo scale.
        x1 = (x1 - lb.pad_w) / lb.scale;
        y1 = (y1 - lb.pad_h) / lb.scale;
        x2 = (x2 - lb.pad_w) / lb.scale;
        y2 = (y2 - lb.pad_h) / lb.scale;

        x1 = std::clamp(x1, 0.0f, static_cast<float>(orig_size.width));
        y1 = std::clamp(y1, 0.0f, static_cast<float>(orig_size.height));
        x2 = std::clamp(x2, 0.0f, static_cast<float>(orig_size.width));
        y2 = std::clamp(y2, 0.0f, static_cast<float>(orig_size.height));

        const int bx = static_cast<int>(x1), by = static_cast<int>(y1);
        const int bw = static_cast<int>(x2-x1), bh_px = static_cast<int>(y2-y1);
        if (bw > 0 && bh_px > 0) {
            nms_boxes_.emplace_back(bx, by, bw, bh_px);
            nms_scores_.push_back(conf);
            nms_class_ids_.push_back(static_cast<int>(cls));
        }
    }

    std::vector<int> keep = nmsBoxes(nms_boxes_, nms_scores_,
                                      conf_threshold_, nms_iou_threshold_);

    std::vector<vigia_msgs::msg::Detection> dets;
    dets.reserve(keep.size());
    for (int idx : keep) {
        vigia_msgs::msg::Detection d;
        d.class_id    = static_cast<uint32_t>(nms_class_ids_[idx]);
        d.confidence  = nms_scores_[idx];
        d.bbox.x_offset = static_cast<uint32_t>(nms_boxes_[idx].x);
        d.bbox.y_offset = static_cast<uint32_t>(nms_boxes_[idx].y);
        d.bbox.width    = static_cast<uint32_t>(nms_boxes_[idx].width);
        d.bbox.height   = static_cast<uint32_t>(nms_boxes_[idx].height);
        dets.push_back(d);
    }
    return dets;
}

void VisionNode::on_image(std::shared_ptr<const sensor_msgs::msg::Image> msg)
{
    auto t0 = std::chrono::steady_clock::now();

    cv::Mat src(static_cast<int>(msg->height), static_cast<int>(msg->width),
                CV_8UC3, const_cast<uint8_t*>(msg->data.data()));

    // Letterbox resize → RGB HWC in letterbox_buf_.
    Letterbox lb = letterbox_resize(src, preproc_resized_,
                                    letterbox_buf_.data(), input_width_, input_height_);

    // NEON single-pass BGR HWC → RGB CHW transposition.
    neon_bgr_hwc_to_rgb_chw(letterbox_buf_.data(), chw_input_buf_.data(),
                             input_width_, input_height_);

    // Wrap CHW buffer as ONNX tensor — zero copy.
    std::array<int64_t, 4> shape{1, 3, input_height_, input_width_};
    auto in_tensor = Ort::Value::CreateTensor<uint8_t>(
        mem_info_, chw_input_buf_.data(), chw_input_buf_.size(),
        shape.data(), shape.size());

    auto outputs = session_->Run(Ort::RunOptions{nullptr},
                                 input_names_.data(), &in_tensor, 1,
                                 output_names_.data(), output_names_.size());

    auto t1  = std::chrono::steady_clock::now();
    float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

    // ── Publish DetectionArray ────────────────────────────────────────────
    auto det_msg = std::make_unique<vigia_msgs::msg::DetectionArray>();
    det_msg->header             = msg->header;
    det_msg->frame_id           = static_cast<uint32_t>(std::stoul(msg->header.frame_id));
    det_msg->inference_latency_ms = ms;
    det_msg->detections = postprocess(outputs[0], lb, cv::Size(msg->width, msg->height));
    pub_det_->publish(std::move(det_msg));

    // ── Publish SpatialLatent S_t ─────────────────────────────────────────
    auto lat_msg = std::make_unique<vigia_msgs::msg::SpatialLatent>();
    lat_msg->header            = msg->header;
    lat_msg->frame_id          = static_cast<uint32_t>(std::stoul(msg->header.frame_id));
    lat_msg->source_layer_name = latent_layer_name_;
    if (outputs.size() > 1) {
        const float* ld  = outputs[1].GetTensorData<float>();
        auto ls          = outputs[1].GetTensorTypeAndShapeInfo().GetShape();
        size_t elems     = 1;
        for (auto d : ls) elems *= static_cast<size_t>(d);
        lat_msg->latent_vector.assign(ld, ld + elems);
    }
    pub_lat_->publish(std::move(lat_msg));
}

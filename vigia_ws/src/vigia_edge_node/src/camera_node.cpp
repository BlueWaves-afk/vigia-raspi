#include "camera_node.hpp"
#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("camera_node", options)
{
    declare_parameter("camera_index",  0);
    declare_parameter("target_fps",    30.0);
    declare_parameter("frame_width",   1280);
    declare_parameter("frame_height",  720);
    declare_parameter("shm_ring_size", 300);

    camera_index_  = get_parameter("camera_index").as_int();
    target_fps_    = get_parameter("target_fps").as_double();
    frame_width_   = get_parameter("frame_width").as_int();
    frame_height_  = get_parameter("frame_height").as_int();
    shm_ring_size_ = get_parameter("shm_ring_size").as_int();

    if (!cap_.open(camera_index_, cv::CAP_V4L2)) {
        throw std::runtime_error("CameraNode: failed to open /dev/video" +
                                 std::to_string(camera_index_));
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  frame_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    cap_.set(cv::CAP_PROP_FPS,          target_fps_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE,   2);  // minimize V4L2 buffer depth for latency

    // Pre-allocate shared-memory ring and cv::Mat capture buffer.
    shm_ring_ = std::make_unique<ShmRingBuffer>(
        static_cast<size_t>(shm_ring_size_),
        static_cast<uint32_t>(frame_width_),
        static_cast<uint32_t>(frame_height_));
    frame_buf_.create(frame_height_, frame_width_, CV_8UC3);

    // Pre-fill immutable Image fields — only stamp and data change per frame.
    img_msg_template_.encoding     = "bgr8";
    img_msg_template_.width        = static_cast<uint32_t>(frame_width_);
    img_msg_template_.height       = static_cast<uint32_t>(frame_height_);
    img_msg_template_.step         = static_cast<uint32_t>(frame_width_ * 3);
    img_msg_template_.is_bigendian = false;
    img_msg_template_.data.resize(static_cast<size_t>(frame_width_ * frame_height_ * 3));

    pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/vigia/camera/image_raw", vigia::qos::camera_frames());

    auto period_ns = static_cast<int64_t>(1e9 / target_fps_);
    timer_ = create_wall_timer(
        std::chrono::nanoseconds(period_ns),
        std::bind(&CameraNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "CameraNode ready: %dx%d @ %.0f fps, ring=%d",
                frame_width_, frame_height_, target_fps_, shm_ring_size_);
}

void CameraNode::timer_callback()
{
    if (!cap_.read(frame_buf_) || frame_buf_.empty()) {
        RCLCPP_WARN(get_logger(), "Frame capture failed — skipping");
        return;
    }

    const uint64_t ts_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    // Seqlock ring write — zero-copy path for AntiDeathNode snapshot.
    shm_ring_->write_frame(
        frame_counter_, ts_us,
        frame_buf_.data,
        static_cast<size_t>(frame_buf_.total() * frame_buf_.elemSize()));

    // Intra-process publish — unique_ptr publish promotes to shared_ptr for VisionNode+DepthNode.
    auto msg = std::make_unique<sensor_msgs::msg::Image>(img_msg_template_);
    msg->header.stamp    = now();
    msg->header.frame_id = std::to_string(frame_counter_);
    std::memcpy(msg->data.data(), frame_buf_.data,
                static_cast<size_t>(frame_width_ * frame_height_ * 3));

    pub_->publish(std::move(msg));

    // Lazy-open metadata ring (FusionNode creates it; retry silently until ready).
    if (!meta_ring_) {
        try {
            meta_ring_shm_ = std::make_unique<ShmMetaRing>(/*creator=*/false);
            meta_ring_ = meta_ring_shm_->ring();
        } catch (...) {
            meta_ring_ = nullptr;
        }
    }
    // Write minimal sidecar so AntiDeathNode can correlate frame timestamps.
    if (meta_ring_) {
        FrameMetadata m{};
        m.frame_id     = frame_counter_;
        m.timestamp_us = ts_us;
        meta_ring_->write_metadata(m);
    }

    ++frame_counter_;
}

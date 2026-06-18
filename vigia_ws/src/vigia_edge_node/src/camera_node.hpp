#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/videoio.hpp>
#include <memory>
#include <chrono>

#include "vigia_edge_node/vigia_qos.hpp"
#include "vigia_edge_node/shm_ring_buffer.hpp"
#include "vigia_edge_node/frame_metadata_ring.hpp"

class CameraNode : public rclcpp::Node {
public:
    explicit CameraNode(const rclcpp::NodeOptions & options);

private:
    void timer_callback();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr                          timer_;

    cv::VideoCapture cap_;
    std::unique_ptr<ShmRingBuffer>  shm_ring_;
    // Metadata ring — opened lazily (FusionNode creates it; CameraNode is a late joiner).
    std::unique_ptr<ShmMetaRing>   meta_ring_shm_;
    FrameMetadataRing *            meta_ring_{nullptr};

    // Pre-allocated frame buffer — reused every cycle, no per-frame heap alloc.
    cv::Mat frame_buf_;
    sensor_msgs::msg::Image img_msg_template_;

    uint32_t frame_counter_{0};
    int      camera_index_{0};
    double   target_fps_{30.0};
    int      frame_width_{1280};
    int      frame_height_{720};
    int      shm_ring_size_{300};
};

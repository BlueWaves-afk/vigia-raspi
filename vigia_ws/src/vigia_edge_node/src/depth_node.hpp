#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>
#include <memory>
#include <vector>
#include <string>

#include "vigia_edge_node/vigia_qos.hpp"
#include "vigia_msgs/msg/depth_map.hpp"

class DepthNode : public rclcpp::Node {
public:
    explicit DepthNode(const rclcpp::NodeOptions & options);

private:
    void on_image(std::shared_ptr<const sensor_msgs::msg::Image> msg);
    float read_cpu_temp_celsius();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<vigia_msgs::msg::DepthMap>::SharedPtr  pub_;

    Ort::Env     env_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo mem_info_;

    std::vector<float> midas_input_buf_;   // FP32 CHW 3×256×256
    std::vector<float> midas_output_buf_;  // FP32 256×256

    std::string model_path_;
    int     midas_stride_{3};
    double  temp_warn_c_{75.0};
    double  temp_crit_c_{85.0};

    uint64_t frame_counter_{0};
    float    cached_temp_c_{0.0f};
    std::chrono::steady_clock::time_point last_temp_read_;

    const char* input_name_{nullptr};
    const char* output_name_{nullptr};
    std::string input_name_str_;
    std::string output_name_str_;
};

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <onnxruntime/onnxruntime_cxx_api.h>
#if defined(VIGIA_HAVE_ACL_EP)
#  include <onnxruntime/acl_provider_factory.h>
#endif
#include <opencv2/core.hpp>
#include <arm_neon.h>
#include <memory>
#include <string>
#include <vector>

#include "vigia_edge_node/vigia_qos.hpp"
#include "vigia_msgs/msg/detection_array.hpp"
#include "vigia_msgs/msg/spatial_latent.hpp"

class VisionNode : public rclcpp::Node {
public:
    explicit VisionNode(const rclcpp::NodeOptions & options);

private:
    void on_image(std::shared_ptr<const sensor_msgs::msg::Image> msg);

    // NEON single-pass BGR→RGB + HWC→CHW transpose (~8µs for 320×320).
    void neon_bgr_hwc_to_rgb_chw(const uint8_t* src_bgr,
                                  uint8_t*       dst_chw,
                                  int            w, int h);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<vigia_msgs::msg::DetectionArray>::SharedPtr pub_det_;
    rclcpp::Publisher<vigia_msgs::msg::SpatialLatent>::SharedPtr  pub_lat_;

    Ort::Env     env_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo mem_info_;

    // Pre-allocated inference buffers — no per-frame heap alloc.
    std::vector<uint8_t>  letterbox_buf_;   // resized input (RGB, HWC)
    std::vector<uint8_t>  chw_input_buf_;   // CHW transposed (320×320×3)
    std::vector<float>    latent_buf_;      // S_t flattened output

    std::string model_path_;
    std::string latent_layer_name_;
    float       conf_threshold_{0.25f};
    float       nms_iou_threshold_{0.45f};
    int         input_width_{320};
    int         input_height_{320};

    // ONNX node names (ASCII, kept alive for session Run call)
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    std::vector<std::string> output_names_storage_;
};

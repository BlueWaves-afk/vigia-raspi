#pragma once
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <memory>
#include <optional>

#include "vigia_edge_node/vigia_qos.hpp"
#include "vigia_msgs/msg/detection_array.hpp"
#include "vigia_msgs/msg/depth_map.hpp"
#include "vigia_msgs/msg/imu_sample.hpp"
#include "vigia_msgs/msg/gps_pvt.hpp"
#include "vigia_msgs/msg/signed_et.hpp"
#include "vigia_msgs/msg/hazard_event.hpp"

class FusionNode : public rclcpp::Node {
public:
    explicit FusionNode(const rclcpp::NodeOptions & options);

private:
    void on_detections(std::unique_ptr<vigia_msgs::msg::DetectionArray> msg);
    void on_depth(std::unique_ptr<vigia_msgs::msg::DepthMap> msg);
    void on_imu(std::shared_ptr<const vigia_msgs::msg::ImuSample> msg);
    void on_gps(std::shared_ptr<const vigia_msgs::msg::GpsPvt> msg);
    void on_signed_et(std::shared_ptr<const vigia_msgs::msg::SignedEt> msg);

    // Returns gravity-compensated world-frame Z acceleration.
    float gravity_compensate_z(const vigia_msgs::msg::ImuSample & imu);
    float compute_iss(float a_world_z, float v_ms);
    void  fuse_and_maybe_publish();

    rclcpp::Subscription<vigia_msgs::msg::DetectionArray>::SharedPtr sub_det_;
    rclcpp::Subscription<vigia_msgs::msg::DepthMap>::SharedPtr       sub_depth_;
    rclcpp::Subscription<vigia_msgs::msg::ImuSample>::SharedPtr      sub_imu_;
    rclcpp::Subscription<vigia_msgs::msg::GpsPvt>::SharedPtr         sub_gps_;
    rclcpp::Subscription<vigia_msgs::msg::SignedEt>::SharedPtr        sub_et_;
    rclcpp::Publisher<vigia_msgs::msg::HazardEvent>::SharedPtr        pub_hazard_;

    // Latest samples — always updated on arrival, never awaited.
    std::unique_ptr<vigia_msgs::msg::DetectionArray> latest_det_;
    std::unique_ptr<vigia_msgs::msg::DepthMap>       latest_depth_;
    std::shared_ptr<const vigia_msgs::msg::ImuSample> latest_imu_;
    std::shared_ptr<const vigia_msgs::msg::GpsPvt>    latest_gps_;
    std::shared_ptr<const vigia_msgs::msg::SignedEt>   latest_et_;

    // 2D Kalman filter — stack-allocated Eigen, no heap.
    Eigen::Vector2f kf_x_{0.0f, 0.0f};  // [v_x, v_y]
    Eigen::Matrix2f kf_P_{Eigen::Matrix2f::Identity()};
    rclcpp::Time    last_imu_stamp_;

    double rri_threshold_{0.75};
    double w_yolo_{0.35}, w_geometry_{0.25}, w_temporal_{0.15}, w_iss_{0.25};
    double v_min_ms_{2.0};
    double kf_Q_{0.01}, kf_R_{0.5};
    double gps_hdop_max_{2.5};
    std::string device_id_;
};

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <memory>
#include <array>
#include <cstddef>

#include "vigia_edge_node/vigia_qos.hpp"
#include "vigia_msgs/msg/detection_array.hpp"
#include "vigia_msgs/msg/depth_map.hpp"
#include "vigia_msgs/msg/imu_sample.hpp"
#include "vigia_msgs/msg/gps_pvt.hpp"
#include "vigia_msgs/msg/signed_et.hpp"
#include "vigia_msgs/msg/hazard_event.hpp"

// POD result of least-squares plane fitting on a depth ROI.
struct DepthResidualStats {
    float meanResidual{0.f};
    float minResidual{0.f};
    float stdResidual{0.f};
};

class FusionNode : public rclcpp::Node {
public:
    explicit FusionNode(const rclcpp::NodeOptions & options);

private:
    void on_detections(std::unique_ptr<vigia_msgs::msg::DetectionArray> msg);
    void on_depth(std::unique_ptr<vigia_msgs::msg::DepthMap> msg);
    void on_imu(std::shared_ptr<const vigia_msgs::msg::ImuSample> msg);
    void on_gps(std::shared_ptr<const vigia_msgs::msg::GpsPvt> msg);
    void on_signed_et(std::shared_ptr<const vigia_msgs::msg::SignedEt> msg);

    float gravity_compensate_z(const vigia_msgs::msg::ImuSample & imu);
    float compute_iss(float a_world_z, float v_ms);
    void  fuse_and_maybe_publish();

    // ── Depth geometry pipeline (ported from analytical.cpp + fusion.cpp) ──
    cv::Rect scale_roi_to_depth(const sensor_msgs::msg::RegionOfInterest & bbox,
                                 int orig_w, int orig_h, int depth_w, int depth_h) const;
    cv::Mat  extract_depth_roi(const cv::Mat & depth_map, const cv::Rect & roi);
    DepthResidualStats compute_depth_residuals(const cv::Mat & roi_depth) const;
    float compute_geometry_confidence(float depression, float roughness) const;

    // ── TemporalAnalyzer (ported from temporal.cpp) ──────────────────────
    static constexpr std::size_t kHistorySize = 16;
    std::array<float, kHistorySize> depression_buf_{};
    std::array<float, kHistorySize> roughness_buf_{};
    std::size_t temporal_head_{0};
    std::size_t temporal_count_{0};
    void  temporal_update(float depression, float roughness);
    float temporal_persistence() const;
    float temporal_stability() const;
    float compute_temporal_confidence() const;

    // Pre-allocated scratch for extractDepthROI.
    cv::Mat roi_scratch_;

    rclcpp::Subscription<vigia_msgs::msg::DetectionArray>::SharedPtr sub_det_;
    rclcpp::Subscription<vigia_msgs::msg::DepthMap>::SharedPtr       sub_depth_;
    rclcpp::Subscription<vigia_msgs::msg::ImuSample>::SharedPtr      sub_imu_;
    rclcpp::Subscription<vigia_msgs::msg::GpsPvt>::SharedPtr         sub_gps_;
    rclcpp::Subscription<vigia_msgs::msg::SignedEt>::SharedPtr        sub_et_;
    rclcpp::Publisher<vigia_msgs::msg::HazardEvent>::SharedPtr        pub_hazard_;

    std::unique_ptr<vigia_msgs::msg::DetectionArray> latest_det_;
    std::unique_ptr<vigia_msgs::msg::DepthMap>       latest_depth_;
    std::shared_ptr<const vigia_msgs::msg::ImuSample> latest_imu_;
    std::shared_ptr<const vigia_msgs::msg::GpsPvt>    latest_gps_;
    std::shared_ptr<const vigia_msgs::msg::SignedEt>   latest_et_;

    // 2D Kalman filter — stack-allocated, no heap.
    Eigen::Vector2f kf_x_{0.0f, 0.0f};
    Eigen::Matrix2f kf_P_{Eigen::Matrix2f::Identity()};
    rclcpp::Time    last_imu_stamp_;

    double rri_threshold_{0.75};
    double w_yolo_{0.35}, w_geometry_{0.25}, w_temporal_{0.15}, w_iss_{0.25};
    double v_min_ms_{2.0};
    double kf_Q_{0.01}, kf_R_{0.5};
    double gps_hdop_max_{2.5};
    std::string device_id_;
};

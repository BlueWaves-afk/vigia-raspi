#include "fusion_node.hpp"
#include <cmath>
#include <algorithm>

FusionNode::FusionNode(const rclcpp::NodeOptions & options)
: Node("fusion_node", options)
{
    declare_parameter("rri_threshold",      0.75);
    declare_parameter("w_yolo",             0.35);
    declare_parameter("w_geometry",         0.25);
    declare_parameter("w_temporal",         0.15);
    declare_parameter("w_iss",              0.25);
    declare_parameter("v_min_ms",           2.0);
    declare_parameter("kalman_process_noise", 0.01);
    declare_parameter("kalman_meas_noise",    0.5);
    declare_parameter("gps_fallback_hdop_max", 2.5);
    declare_parameter("device_id",          "vigia-000");

    rri_threshold_ = get_parameter("rri_threshold").as_double();
    w_yolo_        = get_parameter("w_yolo").as_double();
    w_geometry_    = get_parameter("w_geometry").as_double();
    w_temporal_    = get_parameter("w_temporal").as_double();
    w_iss_         = get_parameter("w_iss").as_double();
    v_min_ms_      = get_parameter("v_min_ms").as_double();
    kf_Q_          = get_parameter("kalman_process_noise").as_double();
    kf_R_          = get_parameter("kalman_meas_noise").as_double();
    gps_hdop_max_  = get_parameter("gps_fallback_hdop_max").as_double();
    device_id_     = get_parameter("device_id").as_string();

    pub_hazard_ = create_publisher<vigia_msgs::msg::HazardEvent>(
        "/vigia/hazard_event", vigia::qos::hazard_events());

    sub_det_ = create_subscription<vigia_msgs::msg::DetectionArray>(
        "/vigia/detections", vigia::qos::inference_results(),
        std::bind(&FusionNode::on_detections, this, std::placeholders::_1));
    sub_depth_ = create_subscription<vigia_msgs::msg::DepthMap>(
        "/vigia/depth", vigia::qos::inference_results(),
        std::bind(&FusionNode::on_depth, this, std::placeholders::_1));
    sub_imu_ = create_subscription<vigia_msgs::msg::ImuSample>(
        "/vigia/imu", vigia::qos::sensor_stream(),
        std::bind(&FusionNode::on_imu, this, std::placeholders::_1));
    sub_gps_ = create_subscription<vigia_msgs::msg::GpsPvt>(
        "/vigia/gps", vigia::qos::sensor_stream(),
        std::bind(&FusionNode::on_gps, this, std::placeholders::_1));
    sub_et_ = create_subscription<vigia_msgs::msg::SignedEt>(
        "/vigia/signed_et", vigia::qos::signed_et(),
        std::bind(&FusionNode::on_signed_et, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "FusionNode ready: rri_threshold=%.2f device_id=%s",
                rri_threshold_, device_id_.c_str());
}

void FusionNode::on_detections(std::unique_ptr<vigia_msgs::msg::DetectionArray> msg) {
    latest_det_ = std::move(msg);
    fuse_and_maybe_publish();
}
void FusionNode::on_depth(std::unique_ptr<vigia_msgs::msg::DepthMap> msg) {
    latest_depth_ = std::move(msg);
}
void FusionNode::on_imu(std::shared_ptr<const vigia_msgs::msg::ImuSample> msg) {
    // Kalman predict step at 100 Hz.
    if (last_imu_stamp_.nanoseconds() > 0) {
        float dt = static_cast<float>(
            (rclcpp::Time(msg->header.stamp) - last_imu_stamp_).seconds());
        if (dt > 0 && dt < 0.1f) {
            float a_world_z = gravity_compensate_z(*msg);
            (void)a_world_z;  // Z only for ISS; X/Y for Kalman velocity
            Eigen::Matrix2f A = Eigen::Matrix2f::Identity();
            Eigen::Matrix2f Q = Eigen::Matrix2f::Identity() * static_cast<float>(kf_Q_);
            kf_P_ = A * kf_P_ * A.transpose() + Q;
        }
    }
    last_imu_stamp_ = msg->header.stamp;
    latest_imu_ = std::move(msg);
}
void FusionNode::on_gps(std::shared_ptr<const vigia_msgs::msg::GpsPvt> msg) {
    // Kalman update step when GPS valid.
    if (msg->valid_fix && msg->hdop <= static_cast<float>(gps_hdop_max_)) {
        float v_total = msg->speed_ms;
        float course  = msg->course_deg * M_PIf / 180.0f;
        Eigen::Vector2f z{v_total * std::sin(course), v_total * std::cos(course)};
        Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * static_cast<float>(kf_R_);
        Eigen::Matrix2f H = Eigen::Matrix2f::Identity();
        Eigen::Matrix2f S = H * kf_P_ * H.transpose() + R;
        Eigen::Matrix2f K = kf_P_ * H.transpose() * S.inverse();
        kf_x_ = kf_x_ + K * (z - H * kf_x_);
        kf_P_ = (Eigen::Matrix2f::Identity() - K * H) * kf_P_;
    }
    latest_gps_ = std::move(msg);
}
void FusionNode::on_signed_et(std::shared_ptr<const vigia_msgs::msg::SignedEt> msg) {
    latest_et_ = std::move(msg);
}

float FusionNode::gravity_compensate_z(const vigia_msgs::msg::ImuSample & imu)
{
    // Quaternion sandwich: a_world = q * a_body * q^-1
    Eigen::Quaternionf q(imu.q_w, imu.q_x, imu.q_y, imu.q_z);
    q.normalize();
    Eigen::Vector3f a_body(imu.lin_accel_x, imu.lin_accel_y, imu.lin_accel_z);
    Eigen::Vector3f a_world = q * a_body;

    // Subtract gravity on world-Z axis.
    return a_world.z() - 9.81f;
}

float FusionNode::compute_iss(float a_world_z, float v_ms)
{
    float v_safe = std::max(v_ms, static_cast<float>(v_min_ms_));
    return std::abs(a_world_z) / v_safe;
}

void FusionNode::fuse_and_maybe_publish()
{
    if (!latest_det_ || !latest_imu_) return;

    // Gravity-compensated ISS.
    float a_z     = gravity_compensate_z(*latest_imu_);
    float v_ms    = (latest_gps_ && latest_gps_->valid_fix)
                  ? latest_gps_->speed_ms
                  : kf_x_.norm();
    float iss     = compute_iss(a_z, v_ms);
    float iss_norm = std::min(iss / 10.0f, 1.0f);  // normalize to [0,1]; 10 m/s²/m/s = saturation

    // Best YOLO confidence.
    float yolo_conf = 0.0f;
    for (const auto & d : latest_det_->detections)
        yolo_conf = std::max(yolo_conf, d.confidence);

    // Geometry confidence from depth (placeholder: ratio of near pixels).
    float geo_conf = 0.0f;
    if (latest_depth_) {
        size_t near = 0;
        for (float v : latest_depth_->data) if (v > 0.7f) ++near;
        geo_conf = static_cast<float>(near) / static_cast<float>(latest_depth_->data.size());
    }

    // Temporal confidence placeholder (frame-to-frame detection consistency).
    float temp_conf = latest_det_->detections.empty() ? 0.0f : 0.5f;

    float rri = static_cast<float>(
        w_yolo_    * yolo_conf  +
        w_geometry_* geo_conf   +
        w_temporal_* temp_conf  +
        w_iss_     * iss_norm);

    if (rri < static_cast<float>(rri_threshold_)) return;

    auto event = std::make_unique<vigia_msgs::msg::HazardEvent>();
    event->header             = latest_det_->header;
    event->device_id          = device_id_;
    event->rri_score          = rri;
    event->iss_score          = iss;
    event->yolo_confidence    = yolo_conf;
    event->geometry_confidence = geo_conf;
    event->temporal_confidence = temp_conf;
    event->detections         = *latest_det_;
    if (latest_depth_)  event->depth_map  = *latest_depth_;
    if (latest_imu_)    event->imu_sample = *latest_imu_;
    if (latest_gps_)    event->gps_pvt    = *latest_gps_;
    if (latest_et_)     event->signed_et  = *latest_et_;

    pub_hazard_->publish(std::move(event));
}

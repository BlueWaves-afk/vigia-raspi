#include "fusion_node.hpp"
#include "vigia_edge_node/vigia_rri.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <algorithm>
#include <limits>

#ifdef VIGIA_HAVE_MBEDTLS
#  include <mbedtls/sha256.h>
#endif

FusionNode::FusionNode(const rclcpp::NodeOptions & options)
: Node("fusion_node", options)
{
    declare_parameter("rri_threshold",       0.75);
    declare_parameter("w_yolo",              0.35);
    declare_parameter("w_geometry",          0.25);
    declare_parameter("w_temporal",          0.15);
    declare_parameter("w_iss",               0.25);
    declare_parameter("v_min_ms",            2.0);
    declare_parameter("kalman_process_noise",0.01);
    declare_parameter("kalman_meas_noise",   0.5);
    declare_parameter("gps_fallback_hdop_max",2.5);
    declare_parameter("device_id",           "vigia-000");

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

    // Open FrameMetadataRing as creator (FusionNode owns the shm lifetime).
    try {
        meta_ring_shm_ = std::make_unique<ShmMetaRing>(/*creator=*/true);
        meta_ring_ = meta_ring_shm_->ring();
        RCLCPP_INFO(get_logger(), "FrameMetadataRing created (%zu KB).",
                    sizeof(FrameMetadataRing) / 1024);
    } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "FrameMetadataRing create failed (%s).", e.what());
    }

    RCLCPP_INFO(get_logger(), "FusionNode ready: rri_threshold=%.2f device_id=%s",
                rri_threshold_, device_id_.c_str());
}

// ── Subscriptions ─────────────────────────────────────────────────────────

void FusionNode::on_detections(std::unique_ptr<vigia_msgs::msg::DetectionArray> msg) {
    latest_det_ = std::move(msg);
    fuse_and_maybe_publish();
}
void FusionNode::on_depth(std::unique_ptr<vigia_msgs::msg::DepthMap> msg) {
    latest_depth_ = std::move(msg);
}
void FusionNode::on_imu(std::shared_ptr<const vigia_msgs::msg::ImuSample> msg) {
    if (last_imu_stamp_.nanoseconds() > 0) {
        float dt = static_cast<float>(
            (rclcpp::Time(msg->header.stamp) - last_imu_stamp_).seconds());
        if (dt > 0 && dt < 0.1f) {
            Eigen::Matrix2f Q = Eigen::Matrix2f::Identity() * static_cast<float>(kf_Q_);
            kf_P_ = kf_P_ + Q;
        }
    }
    last_imu_stamp_ = msg->header.stamp;
    latest_imu_ = std::move(msg);
}
void FusionNode::on_gps(std::shared_ptr<const vigia_msgs::msg::GpsPvt> msg) {
    if (msg->valid_fix && msg->hdop <= static_cast<float>(gps_hdop_max_)) {
        float v_total = msg->speed_ms;
        float course  = msg->course_deg * static_cast<float>(M_PI) / 180.0f;
        Eigen::Vector2f z{v_total * std::sin(course), v_total * std::cos(course)};
        Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * static_cast<float>(kf_R_);
        Eigen::Matrix2f S = kf_P_ + R;
        Eigen::Matrix2f K = kf_P_ * S.inverse();
        kf_x_ = kf_x_ + K * (z - kf_x_);
        kf_P_ = (Eigen::Matrix2f::Identity() - K) * kf_P_;
    }
    latest_gps_ = std::move(msg);
}
void FusionNode::on_signed_et(std::shared_ptr<const vigia_msgs::msg::SignedEt> msg) {
    latest_et_ = std::move(msg);
}

// ── IMU + ISS ─────────────────────────────────────────────────────────────

float FusionNode::gravity_compensate_z(const vigia_msgs::msg::ImuSample & imu)
{
    Eigen::Quaternionf q(imu.q_w, imu.q_x, imu.q_y, imu.q_z);
    q.normalize();
    Eigen::Vector3f a_body(imu.lin_accel_x, imu.lin_accel_y, imu.lin_accel_z);
    return (q * a_body).z() - 9.81f;
}

float FusionNode::compute_iss(float a_world_z, float v_ms)
{
    return std::abs(a_world_z) / std::max(v_ms, static_cast<float>(v_min_ms_));
}

// ── Depth geometry pipeline (ported from analytical.cpp + fusion.cpp) ─────

cv::Rect FusionNode::scale_roi_to_depth(
    const sensor_msgs::msg::RegionOfInterest & bbox,
    int orig_w, int orig_h, int depth_w, int depth_h) const
{
    if (orig_w <= 0 || orig_h <= 0 || depth_w <= 0 || depth_h <= 0) return {};
    const float sx = static_cast<float>(depth_w) / static_cast<float>(orig_w);
    const float sy = static_cast<float>(depth_h) / static_cast<float>(orig_h);
    cv::Rect scaled(
        static_cast<int>(std::round(bbox.x_offset * sx)),
        static_cast<int>(std::round(bbox.y_offset * sy)),
        static_cast<int>(std::round(bbox.width    * sx)),
        static_cast<int>(std::round(bbox.height   * sy)));
    return scaled & cv::Rect(0, 0, depth_w, depth_h);
}

cv::Mat FusionNode::extract_depth_roi(const cv::Mat & depth_map, const cv::Rect & roi)
{
    if (roi.empty() || (roi & cv::Rect(0,0,depth_map.cols,depth_map.rows)).empty())
        return {};
    const cv::Rect bounded = roi & cv::Rect(0, 0, depth_map.cols, depth_map.rows);
    depth_map(bounded).copyTo(roi_scratch_);
    cv::GaussianBlur(roi_scratch_, roi_scratch_, cv::Size(3,3), 0);
    cv::normalize(roi_scratch_, roi_scratch_, 0.0f, 1.0f, cv::NORM_MINMAX);
    return roi_scratch_;
}

// Least-squares plane fit z = ax + by + c; pothole = negative residual below plane.
// Ported verbatim from analytical.cpp:382-458.
DepthResidualStats FusionNode::compute_depth_residuals(const cv::Mat & roi_depth) const
{
    DepthResidualStats stats{};
    if (roi_depth.empty() || roi_depth.type() != CV_32F) return stats;

    const int rows = roi_depth.rows;
    const int cols = roi_depth.cols;
    const int N    = rows * cols;
    const float fN = static_cast<float>(N);

    float sumX=0,sumY=0,sumZ=0,sumXX=0,sumYY=0,sumXY=0,sumXZ=0,sumYZ=0;
    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x) {
            const float z  = roi_depth.at<float>(y, x);
            const float fx = static_cast<float>(x);
            const float fy = static_cast<float>(y);
            sumX+=fx; sumY+=fy; sumZ+=z;
            sumXX+=fx*fx; sumYY+=fy*fy; sumXY+=fx*fy;
            sumXZ+=fx*z;  sumYZ+=fy*z;
        }

    const float denom =
        sumXX*sumYY*fN + 2.f*sumX*sumY*sumXY -
        sumXX*sumY*sumY - sumYY*sumX*sumX - sumXY*sumXY*fN;

    float a=0, b=0, c=0;
    if (std::abs(denom) > 1e-6f) {
        a = (sumXZ*sumYY*fN + sumX*sumY*sumYZ + sumXY*sumY*sumZ -
             sumXZ*sumY*sumY - sumYY*sumX*sumZ - sumXY*sumYZ*fN) / denom;
        b = (sumXX*sumYZ*fN + sumX*sumY*sumXZ + sumX*sumXY*sumZ -
             sumXX*sumY*sumZ - sumYZ*sumX*sumX - sumXY*sumXZ*fN) / denom;
        c = (sumZ - a*sumX - b*sumY) / fN;
    }

    float mean=0, minVal=std::numeric_limits<float>::max(), var=0;
    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x) {
            const float r = roi_depth.at<float>(y,x) -
                            (a*static_cast<float>(x) + b*static_cast<float>(y) + c);
            mean += r;
            if (r < minVal) minVal = r;
            var += r*r;
        }
    mean /= fN;
    var   = (var / fN) - (mean * mean);

    stats.meanResidual = mean;
    stats.minResidual  = minVal;
    stats.stdResidual  = std::sqrt(std::max(0.0f, var));
    return stats;
}

// Ported from fusion.cpp::computeGeometryConfidence().
float FusionNode::compute_geometry_confidence(float depression, float roughness) const
{
    const float dep        = std::clamp(depression, 0.0f, 1.0f);
    const float roughPenalty = std::exp(-roughness * 10.0f);
    return std::clamp(dep * roughPenalty, 0.0f, 1.0f);
}

// ── TemporalAnalyzer (ported from temporal.cpp) ───────────────────────────

void FusionNode::temporal_update(float depression, float roughness)
{
    depression_buf_[temporal_head_] = depression;
    roughness_buf_[temporal_head_]  = roughness;
    temporal_head_ = (temporal_head_ + 1) % kHistorySize;
    if (temporal_count_ < kHistorySize) ++temporal_count_;
}

float FusionNode::temporal_persistence() const
{
    if (temporal_count_ < 2) return 0.0f;
    const float n = static_cast<float>(temporal_count_);
    float sum=0, sumSq=0;
    for (std::size_t i = 0; i < temporal_count_; ++i) {
        const float v = depression_buf_[i];
        sum += v; sumSq += v*v;
    }
    const float mean   = sum / n;
    const float var    = (sumSq / n) - (mean * mean);
    const float stddev = std::sqrt(std::max(0.0f, var));
    return mean / (stddev + 1e-4f);
}

float FusionNode::temporal_stability() const
{
    if (temporal_count_ < 2) return 0.0f;
    const float n = static_cast<float>(temporal_count_);
    float sum=0, sumSq=0;
    for (std::size_t i = 0; i < temporal_count_; ++i) {
        const float v = roughness_buf_[i];
        sum += v; sumSq += v*v;
    }
    const float mean   = sum / n;
    const float var    = (sumSq / n) - (mean * mean);
    const float stddev = std::sqrt(std::max(0.0f, var));
    return 1.0f / (stddev + 1e-4f);
}

// Ported from fusion.cpp::computeTemporalConfidence().
float FusionNode::compute_temporal_confidence() const
{
    const float p = std::tanh(temporal_persistence() * 0.1f);
    const float s = std::tanh(temporal_stability()   * 0.01f);
    return std::clamp(0.5f*p + 0.5f*s, 0.0f, 1.0f);
}

// ── Fusion ────────────────────────────────────────────────────────────────

void FusionNode::fuse_and_maybe_publish()
{
    // Detections are the minimum input. IMU/GPS/depth are all OPTIONAL — when
    // the Pico is in stub mode the node degrades gracefully instead of going
    // dead (design spec §6.4, was the G5 bug: `if (!latest_imu_) return;`).
    if (!latest_det_) return;

    // ISS — gravity-compensated. Only valid when an IMU sample is present.
    const bool have_imu = static_cast<bool>(latest_imu_);
    float iss = 0.0f, iss_norm = 0.0f;
    if (have_imu) {
        float a_z  = gravity_compensate_z(*latest_imu_);
        float v_ms = (latest_gps_ && latest_gps_->valid_fix)
                   ? latest_gps_->speed_ms
                   : kf_x_.norm();
        iss      = compute_iss(a_z, v_ms);
        iss_norm = std::min(iss / 10.0f, 1.0f);
    }

    // Best YOLO detection confidence.
    float yolo_conf = 0.0f;
    for (const auto & d : latest_det_->detections)
        yolo_conf = std::max(yolo_conf, d.confidence);
    const bool have_yolo = !latest_det_->detections.empty();

    // Geometry confidence from plane-fitting depth residuals on best bbox.
    float geo_conf = 0.0f;
    bool  have_geo = false;
    if (latest_depth_ && !latest_det_->detections.empty()) {
        const auto& best_det = *std::max_element(
            latest_det_->detections.begin(), latest_det_->detections.end(),
            [](const auto& a, const auto& b){ return a.confidence < b.confidence; });

        // Reconstruct depth cv::Mat header wrapping the msg float array — no copy.
        const int dw = static_cast<int>(latest_depth_->width);
        const int dh = static_cast<int>(latest_depth_->height);
        cv::Mat depth_mat(dh, dw, CV_32F,
            const_cast<float*>(latest_depth_->data.data()));

        // Find original frame size from detection (frame_id not carrying size;
        // use depth map aspect as proxy for now — TODO: pass orig_w/h in DepthMap).
        cv::Rect roi = scale_roi_to_depth(best_det.bbox, dw, dh, dw, dh);
        cv::Mat  roi_depth = extract_depth_roi(depth_mat, roi);
        if (!roi_depth.empty()) {
            DepthResidualStats stats = compute_depth_residuals(roi_depth);
            const float depression = std::max(0.0f, -stats.minResidual);
            const float roughness  = stats.stdResidual;
            geo_conf = compute_geometry_confidence(depression, roughness);
            temporal_update(depression, roughness);
            have_geo = true;
        }
    }

    // Temporal confidence from circular history (only meaningful with depth).
    float temp_conf = compute_temporal_confidence();
    const bool have_temporal = have_geo && temporal_count_ >= 2;

    // Shared, degraded-mode-aware RRI (design spec §6.3). Absent terms drop out
    // and the remaining weights renormalize, so a vision-only system still
    // produces a meaningful score instead of returning early.
    vigia::RriWeights weights;
    weights.w_yolo     = static_cast<float>(w_yolo_);
    weights.w_geometry = static_cast<float>(w_geometry_);
    weights.w_temporal = static_cast<float>(w_temporal_);
    weights.w_iss      = static_cast<float>(w_iss_);

    vigia::RriInputs rin;
    rin.have_yolo     = have_yolo;     rin.yolo_conf = yolo_conf;
    rin.have_geometry = have_geo;      rin.geo_conf  = geo_conf;
    rin.have_temporal = have_temporal; rin.temp_conf = temp_conf;
    rin.have_iss      = have_imu;      rin.iss_norm  = iss_norm;

    const float rri = vigia::compute_rri(weights, rin);
    const vigia::RriTier tier = vigia::classify_tier(rin);

    // Write per-frame metadata sidecar regardless of RRI threshold —
    // AntiDeathNode needs all frames for the emergency snapshot.
    {
        const uint32_t fid = latest_det_->header.stamp.nanosec;  // proxy frame id
        const uint64_t ts_us =
            static_cast<uint64_t>(latest_det_->header.stamp.sec) * 1'000'000ULL +
            latest_det_->header.stamp.nanosec / 1000ULL;
        const uint8_t det_count =
            static_cast<uint8_t>(
                std::min<size_t>(latest_det_->detections.size(), 255));
        write_frame_metadata(rri, iss, det_count, yolo_conf, fid, ts_us);
    }

    if (rri < static_cast<float>(rri_threshold_)) return;

    auto event = std::make_unique<vigia_msgs::msg::HazardEvent>();
    event->header              = latest_det_->header;
    event->device_id           = device_id_;
    event->rri_score           = rri;
    event->iss_score           = iss;
    event->yolo_confidence     = yolo_conf;
    event->geometry_confidence = geo_conf;
    event->temporal_confidence = temp_conf;
    event->degraded            = (tier != vigia::RriTier::kFull);
    event->rri_tier            = static_cast<uint8_t>(tier);
    event->detections          = *latest_det_;
    if (latest_depth_)  event->depth_map  = *latest_depth_;
    if (latest_imu_)    event->imu_sample = *latest_imu_;
    if (latest_gps_)    event->gps_pvt    = *latest_gps_;
    if (latest_et_)     event->signed_et  = *latest_et_;

    pub_hazard_->publish(std::move(event));
}

// ── FrameMetadataRing sidecar writer ─────────────────────────────────────────

void FusionNode::write_frame_metadata(
    float rri, float iss,
    uint8_t det_count, float best_conf,
    uint32_t frame_id, uint64_t ts_us)
{
    if (!meta_ring_) return;

    FrameMetadata m{};
    m.frame_id      = frame_id;
    m.timestamp_us  = ts_us;
    m.rri_score     = rri;
    m.iss_score     = iss;
    m.detection_count = det_count;
    m.best_yolo_conf  = best_conf;

    if (latest_imu_) {
        m.q_w           = latest_imu_->q_w;
        m.q_x           = latest_imu_->q_x;
        m.q_y           = latest_imu_->q_y;
        m.q_z           = latest_imu_->q_z;
        m.lin_accel_x   = latest_imu_->lin_accel_x;
        m.lin_accel_y   = latest_imu_->lin_accel_y;
        m.lin_accel_z   = latest_imu_->lin_accel_z;
        m.imu_cal_status = latest_imu_->calibration_status;
    }

    if (latest_gps_) {
        m.latitude   = latest_gps_->latitude;
        m.longitude  = latest_gps_->longitude;
        m.altitude_m = latest_gps_->altitude_m;
        m.speed_ms   = latest_gps_->speed_ms;
        m.course_deg = latest_gps_->course_deg;
        m.fix_type   = latest_gps_->fix_type;
        m.satellites = latest_gps_->satellites_used;
        m.hdop       = latest_gps_->hdop;
    }

    // Depth map integrity fingerprint — truncated SHA-256.
#ifdef VIGIA_HAVE_MBEDTLS
    if (latest_depth_ && !latest_depth_->data.empty()) {
        uint8_t full_hash[32];
        mbedtls_sha256(
            reinterpret_cast<const uint8_t*>(latest_depth_->data.data()),
            latest_depth_->data.size() * sizeof(float),
            full_hash, /*is224=*/0);
        std::memcpy(m.depth_hash_trunc, full_hash, 8);
    }
#endif

    meta_ring_->write_metadata(m);
}

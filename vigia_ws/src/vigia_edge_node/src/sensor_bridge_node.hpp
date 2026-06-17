#pragma once
/**
 * sensor_bridge_node.hpp — Pico 2 ↔ Pi ROS2 bridge
 *
 * Phase 1 (current): text-line protocol (VIGIA_IMU / VIGIA_GPS / VIGIA_PING)
 * Phase 2 (pending SE wiring): COBS binary SignedEtPacket (173 bytes)
 *
 * Auto-detects wire protocol by first byte:
 *   'V' (0x56) → text mode
 *   0x00       → COBS binary mode
 */

#include <rclcpp/rclcpp.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "vigia_edge_node/vigia_qos.hpp"
#include "vigia_msgs/msg/imu_sample.hpp"
#include "vigia_msgs/msg/gps_pvt.hpp"
#include "vigia_msgs/msg/signed_et.hpp"
#include "vigia_msgs/msg/sensor_health.hpp"

class SensorBridgeNode : public rclcpp::Node {
public:
    explicit SensorBridgeNode(const rclcpp::NodeOptions & options);
    ~SensorBridgeNode();

    /* IMU history lookup — for FusionNode temporal alignment (Phase 2) */
    struct ImuSnapshot {
        uint64_t timestamp_us{0};
        float qw{1.f}, qx{}, qy{}, qz{};
        float ax{}, ay{}, az{};
        uint8_t cal_status{};
        bool valid{false};
    };
    std::optional<ImuSnapshot> imu_at_or_before(uint64_t target_us) const;

private:
    /* ── Serial open / retry ──────────────────────────────────────────────── */
    void open_serial_and_start();

    /* ── Read loop (select + chunk read) ─────────────────────────────────── */
    void read_loop();

    /* ── Protocol dispatch ────────────────────────────────────────────────── */
    enum class WireProto { kUnknown, kText, kCobs };
    WireProto proto_{WireProto::kUnknown};

    void process_text_line(const std::string & line);
    void process_cobs_frame(const uint8_t * payload, size_t len);

    /* ── COBS decode ──────────────────────────────────────────────────────── */
    static size_t decode_cobs(const uint8_t * src, size_t src_len,
                              uint8_t * dst, size_t dst_cap);

    /* ── Text parsers (Phase 1 protocol) ─────────────────────────────────── */
    struct ImuLine {
        uint32_t seq{}; uint64_t timestamp_us{};
        float qw{}, qx{}, qy{}, qz{};
        float ax{}, ay{}, az{};
        uint8_t cal_status{}; bool valid{};
    };
    struct GpsLine {
        uint32_t seq{}; uint64_t timestamp_us{};
        double latitude{}, longitude{};
        float speed_ms{}, hdop{};
        uint8_t fix_type{}, satellites{};
        bool valid{};
    };
    static std::optional<ImuLine> parse_imu(std::string_view sv);
    static std::optional<GpsLine> parse_gps(std::string_view sv);

    /* ── Publishers ───────────────────────────────────────────────────────── */
    rclcpp::Publisher<vigia_msgs::msg::ImuSample>::SharedPtr    pub_imu_;
    rclcpp::Publisher<vigia_msgs::msg::GpsPvt>::SharedPtr       pub_gps_;
    rclcpp::Publisher<vigia_msgs::msg::SignedEt>::SharedPtr     pub_et_;
    rclcpp::Publisher<vigia_msgs::msg::SensorHealth>::SharedPtr pub_health_;

    /* ── Timers ───────────────────────────────────────────────────────────── */
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    void publish_health();

    /* ── Reader thread ────────────────────────────────────────────────────── */
    std::thread       reader_thread_;
    std::atomic<bool> running_{false};
    int               fd_{-1};

    /* ── Serial config ────────────────────────────────────────────────────── */
    std::string serial_device_;
    int         baud_rate_{115200};

    /* ── Anti-replay counters ─────────────────────────────────────────────── */
    uint32_t last_imu_seq_{0};
    uint32_t last_gps_seq_{0};
    uint32_t last_et_seq_{0};

    /* ── IMU history ring buffer (200 samples, mutex-protected) ──────────── */
    static constexpr size_t kImuHistorySize = 200;
    mutable std::mutex                         imu_history_mutex_;
    std::array<ImuSnapshot, kImuHistorySize>   imu_history_{};
    size_t                                     imu_history_head_{0};
    size_t                                     imu_history_count_{0};
    void push_imu_history(const ImuSnapshot & snap);

    /* ── Health counters (written by reader thread, read by health timer) ─── */
    std::atomic<uint64_t> cnt_imu_{0};
    std::atomic<uint64_t> cnt_gps_{0};
    std::atomic<uint64_t> cnt_ping_{0};
    std::atomic<uint32_t> cnt_imu_gaps_{0};
    std::atomic<uint32_t> cnt_gps_gaps_{0};
    std::atomic<uint32_t> cnt_parse_errors_{0};

    /* For measured IMU Hz */
    std::chrono::steady_clock::time_point last_imu_time_{};
    uint64_t                              last_imu_cnt_snapshot_{0};
};

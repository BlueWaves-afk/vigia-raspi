#pragma once
#ifdef VIGIA_HAVE_SDBUS

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "vigia_msgs/msg/spatial_latent.hpp"
#include "vigia_msgs/msg/detection_array.hpp"
#include "vigia_msgs/msg/gps_pvt.hpp"
#include "vigia_edge_node/ble_frame_codec.hpp"
#include "vigia_edge_node/ble_gatt_constants.hpp"
#include "vigia_edge_node/ttc_estimator.hpp"
#include "vigia_edge_node/vigia_qos.hpp"

#ifdef VIGIA_HAVE_MBEDTLS
#  include "vigia_edge_node/vigia_ecdh.hpp"
#endif

// BleGattNode — BlueZ D-Bus GATT peripheral for the phone link.
//
// Subscribes to /vigia/spatial_latent + /vigia/detections, encodes a
// telemetry frame (ble_frame_codec), and notifies the bonded central via
// TELEMETRY_CHAR at stream_hz (default 5 Hz).
//
// Auth (design spec §4.2a, ECDH P-256):
//   HELLO (0x01) → Pi generates nonce_pi, emits CHALLENGE (0x02|nonce_pi|Pi_pub|sig).
//   RESPONSE (0x03|nonce_phone|Phone_pub|sig) → Pi verifies ECDSA sig, derives
//   ECDH shared secret → HKDF session key, emits CONFIRM (0x04|HMAC).
//   Telemetry streams only after Bound state.
//
// Handshake state is owned exclusively by the D-Bus thread — no locking needed.
class BleGattNode : public rclcpp::Node {
public:
    explicit BleGattNode(const rclcpp::NodeOptions& options);
    ~BleGattNode();

private:
    void on_latent(vigia_msgs::msg::SpatialLatent::ConstSharedPtr msg);
    void on_detections(vigia_msgs::msg::DetectionArray::ConstSharedPtr msg);
    void on_gps(vigia_msgs::msg::GpsPvt::ConstSharedPtr msg);

    // Encode a 6-byte FCW alert frame: [0x10 | ttc_f32_le(4) | class_id_u8(1)]
    static std::vector<uint8_t> encode_fcw(float ttc_s, int class_id);

    // D-Bus / BlueZ thread entry — blocks until shutdown_.
    void dbus_thread_main();
    std::vector<uint8_t> encode_current_frame();

    // ── ROS2 ─────────────────────────────────────────────────────────────────
    rclcpp::Subscription<vigia_msgs::msg::SpatialLatent>::SharedPtr  sub_latent_;
    rclcpp::Subscription<vigia_msgs::msg::DetectionArray>::SharedPtr sub_det_;
    rclcpp::Subscription<vigia_msgs::msg::GpsPvt>::SharedPtr         sub_gps_;
    std::atomic<float> ego_speed_ms_{0.f};  // updated by on_gps, read by on_detections

    // ── Mailbox ───────────────────────────────────────────────────────────────
    std::mutex            mailbox_mutex_;
    std::vector<float>    latest_latent_;
    float                 latest_rri_{0.0f};
    bool                  mailbox_ready_{false};

    // ── Handshake state (D-Bus thread only) ──────────────────────────────────
    enum class HandshakeState : uint8_t { Idle, HelloReceived, Bound, Failed };
    HandshakeState        hs_state_{HandshakeState::Idle};
    std::vector<uint8_t>  hs_nonce_pi_;    // 32-byte nonce sent in CHALLENGE
    std::vector<uint8_t>  session_key_;    // 32-byte HKDF-derived session key

    // ── TTC / FCW state (updated in on_detections, consumed in D-Bus thread) ─
    // Separate mutex so high-freq detection callback doesn't block D-Bus notifies.
    std::mutex fcw_mutex_;
    // Pending FCW frame; empty = no pending alert.
    std::vector<uint8_t> pending_fcw_;
    // Per-class TTC estimators: vehicle(1), pedestrian(2), cyclist(3).
    // Frame dimensions populated on first detection (assume 320×240 default until known).
    vigia::TtcEstimator ttc_vehicle_{1,   320.f, 240.f};
    vigia::TtcEstimator ttc_pedestrian_{2, 320.f, 240.f};
    vigia::TtcEstimator ttc_cyclist_{3,   320.f, 240.f};
    std::chrono::steady_clock::time_point last_det_time_{};
    // Profile-scaled TTC threshold pushed from phone via CONTROL_CHAR (default = NEW × BaseTtc)
    float ttc_threshold_s_{4.5f};   // NEW profile: BaseTtc(3.0s) × sProfile(1.5)

    // ── D-Bus thread ──────────────────────────────────────────────────────────
    std::thread           dbus_thread_;
    std::atomic<bool>     shutdown_{false};
    bool                  stream_paused_{false};

    // ── Params ────────────────────────────────────────────────────────────────
    std::string ble_adapter_;
    std::string device_id_;
    float       stream_hz_;
    int         default_dims_;
    std::string identity_key_path_;

    // ── Identity key ──────────────────────────────────────────────────────────
#ifdef VIGIA_HAVE_MBEDTLS
    std::unique_ptr<vigia::VigiaIdentityKey> identity_key_;
#endif
};

#endif // VIGIA_HAVE_SDBUS

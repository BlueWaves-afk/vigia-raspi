#pragma once
#ifdef VIGIA_HAVE_SDBUS

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "vigia_msgs/msg/spatial_latent.hpp"
#include "vigia_msgs/msg/detection_array.hpp"
#include "vigia_edge_node/ble_frame_codec.hpp"
#include "vigia_edge_node/ble_gatt_constants.hpp"
#include "vigia_edge_node/ttc_estimator.hpp"
#include "vigia_edge_node/vigia_qos.hpp"

// BleGattNode — BlueZ D-Bus GATT peripheral for the phone link.
//
// Subscribes to /vigia/spatial_latent + /vigia/detections, encodes a
// telemetry frame (ble_frame_codec), and notifies the bonded central via
// TELEMETRY_CHAR at stream_hz (default 5 Hz).
//
// The D-Bus GLib main loop runs on a dedicated std::thread (SCHED_OTHER).
// ROS2 callbacks update a single-slot atomic mailbox; the GLib timer drains it.
//
// Auth (ECDH handshake, §4) and the full handshake state machine are Phase 2.
// Phase 1 (this file): advertising + GATT skeleton + mailbox plumbing.
class BleGattNode : public rclcpp::Node {
public:
    explicit BleGattNode(const rclcpp::NodeOptions& options);
    ~BleGattNode();

private:
    // ROS2 callbacks — cache latest frame into mailbox.
    void on_latent(vigia_msgs::msg::SpatialLatent::ConstSharedPtr msg);
    void on_detections(vigia_msgs::msg::DetectionArray::ConstSharedPtr msg);

    // Encode a 6-byte FCW alert frame: [0x10 | ttc_f32_le(4) | class_id_u8(1)]
    static std::vector<uint8_t> encode_fcw(float ttc_s, int class_id);

    // D-Bus / BlueZ thread entry — blocks until shutdown_.
    void dbus_thread_main();

    // Encode the latest mailbox contents into a BLE frame and return it.
    // Returns empty vector if no data is ready yet.
    std::vector<uint8_t> encode_current_frame();

    // ── ROS2 ────────────────────────────────────────────────────────────
    rclcpp::Subscription<vigia_msgs::msg::SpatialLatent>::SharedPtr  sub_latent_;
    rclcpp::Subscription<vigia_msgs::msg::DetectionArray>::SharedPtr sub_det_;

    // ── Mailbox (single-slot, lock-free hand-off to D-Bus thread) ───────
    // Guarded by mailbox_mutex_ — the notify timer fires at stream_hz so
    // contention is bounded and brief.
    std::mutex mailbox_mutex_;
    std::vector<float>  latest_latent_;
    float               latest_rri_{0.0f};
    bool                mailbox_ready_{false};

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

    // ── D-Bus thread ─────────────────────────────────────────────────────
    std::thread dbus_thread_;
    std::atomic<bool> shutdown_{false};

    // ── Params ───────────────────────────────────────────────────────────
    std::string ble_adapter_;      // e.g. "hci0"
    std::string device_id_;        // e.g. "vigia-001"
    float       stream_hz_;        // notification cadence (default 5 Hz)
    int         default_dims_;     // 256 or 512
};

#endif // VIGIA_HAVE_SDBUS

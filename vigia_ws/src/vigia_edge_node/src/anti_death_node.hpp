#pragma once
// anti_death_node.hpp — Phase 5 Anti-Death Storage & MQTT Uplink
//
// Implements doc 05 (05_anti_death_and_depin_contracts.md):
//   - FrameMetadataRing snapshot on /dev/shm/vigia_meta_ring
//   - MsgPack serialization (~440 KB payload)
//   - Paho Eclipse MQTT async_client pre-connected at startup (TLS 1.2 mTLS)
//   - SIM7600 LTE ECM — runtime status check via /dev/ttyUSB2
//   - 15-second emergency state machine on SCHED_FIFO 99
//   - Double-buffer SpatialLatent for lock-free subscriber/reader access
//
// Emergency sequence invariants (doc 05 §0):
//   - execute_emergency_sequence() never yields to ROS 2
//   - steady_clock for all budget checks
//   - single msgpack::sbuffer allocation (+ one Paho message object)
//   - atomic_flag prevents re-entry
//
// Compile-time gates:
//   VIGIA_HAVE_PAHO_MQTT  — enables Paho async_client + MsgPack path
//   VIGIA_HAVE_MBEDTLS    — enables depth_hash_trunc SHA-256 computation
//   VIGIA_HAVE_LIBGPIOD   — enables real GPIO edge-wait vs 1ms timer poll

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "vigia_edge_node/vigia_qos.hpp"
#include "vigia_edge_node/frame_metadata_ring.hpp"
#include "vigia_edge_node/shm_ring_buffer.hpp"
#include "vigia_msgs/msg/spatial_latent.hpp"
#include "vigia_msgs/msg/signed_et.hpp"
#include "vigia_msgs/msg/hazard_event.hpp"

#ifdef VIGIA_HAVE_PAHO_MQTT
#  include <mqtt/async_client.h>
#  include <msgpack.hpp>
#endif

#ifdef VIGIA_HAVE_LIBGPIOD
#  include <gpiod.h>
#endif

// ── Emergency state machine ────────────────────────────────────────────────

enum class EmergencyState : uint8_t {
    IDLE               = 0,
    CAPTURING_SNAPSHOT = 1,
    SERIALIZING        = 2,
    MQTT_CONNECTING    = 3,
    MQTT_TRANSMITTING  = 4,
    SAFE_SHUTDOWN      = 5,
    ABORTED            = 6,
};

struct StateBudget {
    static constexpr double kCaptureDeadlineS   = 1.0;
    static constexpr double kSerializeDeadlineS = 3.0;
    static constexpr double kConnectDeadlineS   = 8.0;
    static constexpr double kTransmitDeadlineS  = 13.0;
    static constexpr double kPowerWindowS       = 15.0;
};

// Stack-allocated snapshot (~36 KB). Reserved on the SCHED_FIFO 99 stack
// (512 KB per doc 05 §9). No heap allocation.
struct SnapshotData {
    std::array<FrameMetadata, kMetaRingDepth> frame_metadata{};
    uint32_t  metadata_write_idx{0};

    // Pointer into one of the two pre-allocated SpatialLatent buffers.
    const vigia_msgs::msg::SpatialLatent * spatial_latent_snapshot{nullptr};

    vigia_msgs::msg::SignedEt signed_et_snapshot{};
    bool signed_et_valid{false};

    const vigia_msgs::msg::HazardEvent * hazard_event_snapshot{nullptr};

    uint64_t capture_timestamp_us{0};
    char     device_id[64]{};
};

// ── SIM7600 runtime status ─────────────────────────────────────────────────

struct SimStatus {
    bool registered{false};
    int  rssi_dbm{-999};
    bool data_bearer{false};
};

// ── AntiDeathNode ──────────────────────────────────────────────────────────

class AntiDeathNode : public rclcpp::Node {
public:
    explicit AntiDeathNode(const rclcpp::NodeOptions & options);
    ~AntiDeathNode() override;

    // Test hook — trigger emergency sequence without GPIO.
    void trigger_emergency_for_test() {
        bool exp = false;
        emergency_in_progress_.compare_exchange_strong(
            exp, true, std::memory_order_acq_rel);
        t_zero_ = std::chrono::steady_clock::now();
        execute_emergency_sequence();
    }

private:
    // ── Startup ─────────────────────────────────────────────────────────────
    void startup_init();
    void sim7600_init();
    void mqtt_client_init();

    // ── Subscriber callbacks ─────────────────────────────────────────────────
    void on_spatial_latent(std::unique_ptr<vigia_msgs::msg::SpatialLatent> msg);
    void on_signed_et(std::shared_ptr<const vigia_msgs::msg::SignedEt> msg);
    void on_hazard_event(std::unique_ptr<vigia_msgs::msg::HazardEvent> msg);

    // ── GPIO monitor (1 ms wall timer) ───────────────────────────────────────
    void gpio_poll_callback();

    // ── Emergency state machine ──────────────────────────────────────────────
    void   execute_emergency_sequence();
    bool   do_capture_snapshot(SnapshotData & snap);
    bool   do_serialize(const SnapshotData & snap
#ifdef VIGIA_HAVE_PAHO_MQTT
                      , msgpack::sbuffer & buf
#endif
                        );
    bool   do_mqtt_connect();
    void   do_mqtt_transmit(
#ifdef VIGIA_HAVE_PAHO_MQTT
        const msgpack::sbuffer & buf
#endif
    );
    void   do_safe_shutdown();
    double elapsed_seconds_since_t0() const;

#ifdef VIGIA_HAVE_PAHO_MQTT
    // MsgPack helper sub-packers
    void pack_frame_metadata_array(msgpack::packer<msgpack::sbuffer> & pk,
                                   const SnapshotData & snap);
    void pack_signed_et(msgpack::packer<msgpack::sbuffer> & pk,
                        const vigia_msgs::msg::SignedEt & et, bool valid);
    void pack_hazard_event(msgpack::packer<msgpack::sbuffer> & pk,
                           const vigia_msgs::msg::HazardEvent & ev);
#endif

    // ── SIM7600 AT command helper ────────────────────────────────────────────
    SimStatus query_sim_status();

    // ── Subscriptions ────────────────────────────────────────────────────────
    rclcpp::Subscription<vigia_msgs::msg::SpatialLatent>::SharedPtr sub_latent_;
    rclcpp::Subscription<vigia_msgs::msg::SignedEt>::SharedPtr       sub_et_;
    rclcpp::Subscription<vigia_msgs::msg::HazardEvent>::SharedPtr    sub_hazard_;
    rclcpp::TimerBase::SharedPtr gpio_timer_;

    // ── Double-buffer SpatialLatent (amortized zero-alloc after warm-up) ────
    vigia_msgs::msg::SpatialLatent latest_latent_a_;
    vigia_msgs::msg::SpatialLatent latest_latent_b_;
    std::atomic<vigia_msgs::msg::SpatialLatent *> active_latent_buf_{nullptr};

    // ── Latest cached messages ───────────────────────────────────────────────
    vigia_msgs::msg::SignedEt   latest_signed_et_{};
    std::atomic<bool>            signed_et_valid_{false};
    std::unique_ptr<vigia_msgs::msg::HazardEvent> latest_hazard_event_;

    // ── Shared memory ────────────────────────────────────────────────────────
    std::unique_ptr<ShmMetaRing> meta_ring_shm_;
    FrameMetadataRing *          meta_ring_{nullptr};

    // ── Emergency sequence state ─────────────────────────────────────────────
    std::atomic<bool>            emergency_in_progress_{false};
    std::chrono::steady_clock::time_point t_zero_;

    // ── Paho MQTT async client ────────────────────────────────────────────────
#ifdef VIGIA_HAVE_PAHO_MQTT
    std::unique_ptr<mqtt::async_client>   mqtt_client_;
    mqtt::connect_options                 conn_opts_;
#endif

    // ── GPIO (libgpiod) ──────────────────────────────────────────────────────
#ifdef VIGIA_HAVE_LIBGPIOD
    struct gpiod_chip * gpio_chip_{nullptr};
    struct gpiod_line * gpio_line_{nullptr};
#endif

    // ── Parameters ───────────────────────────────────────────────────────────
    std::string ups_gpio_chip_;
    int         ups_gpio_line_{17};
    std::string device_id_;
    std::string mqtt_broker_host_;
    int         mqtt_broker_port_{8883};
    std::string mqtt_topic_prefix_{"vigia/attest"};
    std::string sim7600_port_{"/dev/ttyUSB2"};
    double      emergency_budget_sec_{13.0};
};

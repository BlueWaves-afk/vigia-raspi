#pragma once
/**
 * anti_death_node.hpp — Emergency telemetry uplink to VIGIA AWS backend
 *
 * Transport: HTTPS POST to API Gateway /telemetry (NOT MQTT).
 * The AWS backend uses REST (APIGateway → ValidatorFunction → DynamoDB →
 * EventBridge Pipe → OrchestratorFunction → Nova Lite VLM → Bedrock Agent → Solana).
 *
 * Signing: Ed25519 (libsodium crypto_sign_ed25519_detached) over the string
 *   "VIGIA:{hazardType}:{lat}:{lon}:{timestamp}:{confidence}"
 * Signature and public key are base58-encoded in the JSON payload — matching
 * tweetnacl/nacl.sign format used by the ValidatorFunction.
 *
 * Key storage: /etc/vigia/device_ed25519.key (64-byte seed+pubkey, binary)
 *              /etc/vigia/device_pubkey.b58  (base58 public key, text)
 *
 * Dependencies: libsodium-dev, libcurl-dev, libgpiod-dev
 */

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "vigia_edge_node/vigia_qos.hpp"
#include "vigia_msgs/msg/spatial_latent.hpp"
#include "vigia_msgs/msg/signed_et.hpp"
#include "vigia_msgs/msg/hazard_event.hpp"

enum class AntiDeathState : uint8_t {
    MONITORING = 0,
    CAPTURE_SNAPSHOT,
    SERIALIZE,
    HTTPS_TRANSMIT,
    SAFE_SHUTDOWN
};

class AntiDeathNode : public rclcpp::Node {
public:
    explicit AntiDeathNode(const rclcpp::NodeOptions & options);
    ~AntiDeathNode();

    // For integration testing — trigger the emergency sequence directly.
    void trigger_emergency_for_test() { power_fail_.store(true); }

private:
    // ── ROS2 subscriber callbacks ──────────────────────────────────────────────
    void on_spatial_latent(std::unique_ptr<vigia_msgs::msg::SpatialLatent> msg);
    void on_signed_et(std::shared_ptr<const vigia_msgs::msg::SignedEt> msg);
    void on_hazard_event(std::unique_ptr<vigia_msgs::msg::HazardEvent> msg);

    // ── GPIO monitor ───────────────────────────────────────────────────────────
    void gpio_monitor_loop();

    // ── Emergency sequence ─────────────────────────────────────────────────────
    void run_emergency_sequence();

    // ── AWS telemetry uplink ───────────────────────────────────────────────────
    struct TelemetryPayload {
        std::string hazard_type;
        double lat{0.0}, lon{0.0};
        float  confidence{0.0f};
        std::string timestamp_iso;   // ISO 8601
        std::string geohash;         // ngeohash7 — computed from lat/lon
        std::string hazard_id;       // "{geohash}#{timestamp}"
        std::string frame_base64;    // optional JPEG frame from shm ring buffer
    };

    bool load_ed25519_key();
    std::string sign_payload(const TelemetryPayload & p) const;
    std::string base58_encode(const uint8_t * data, size_t len) const;
    std::string build_json(const TelemetryPayload & p, const std::string & sig) const;
    bool post_telemetry(const std::string & json_body);

    // ── Subscriptions ──────────────────────────────────────────────────────────
    rclcpp::Subscription<vigia_msgs::msg::SpatialLatent>::SharedPtr sub_latent_;
    rclcpp::Subscription<vigia_msgs::msg::SignedEt>::SharedPtr       sub_et_;
    rclcpp::Subscription<vigia_msgs::msg::HazardEvent>::SharedPtr    sub_hazard_;

    // Latest state — subscriber callbacks update, emergency handler reads.
    std::shared_ptr<const vigia_msgs::msg::SpatialLatent> latest_latent_;
    std::shared_ptr<const vigia_msgs::msg::SignedEt>       latest_et_;
    std::shared_ptr<const vigia_msgs::msg::HazardEvent>    latest_hazard_;

    // ── Threading ──────────────────────────────────────────────────────────────
    std::atomic<bool>  power_fail_{false};
    std::atomic<bool>  shutdown_requested_{false};
    std::thread        gpio_thread_;
    AntiDeathState     state_{AntiDeathState::MONITORING};

    // ── Parameters ─────────────────────────────────────────────────────────────
    std::string ups_gpio_chip_;
    int         ups_gpio_line_{17};
    bool        ups_gpio_active_low_{true};
    double      power_window_seconds_{15.0};

    // AWS Telemetry API Gateway endpoint
    // e.g. https://sq2ri2n51g.execute-api.us-east-1.amazonaws.com/prod/telemetry
    std::string aws_telemetry_url_;
    std::string device_key_path_;    // /etc/vigia/device_ed25519.key
    std::string device_pubkey_b58_;  // base58 Ed25519 public key (loaded at startup)

    // Ed25519 key material (libsodium) — zeroed if key not found
    static constexpr size_t kSeedLen   = 32;
    static constexpr size_t kPubkeyLen = 32;
    static constexpr size_t kSeckeyLen = 64;  // seed + pubkey (libsodium expanded form)
    uint8_t ed25519_seckey_[kSeckeyLen]{};    // zeroed until load_ed25519_key() succeeds
    bool    key_loaded_{false};
};

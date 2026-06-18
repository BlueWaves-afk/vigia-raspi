#pragma once
#ifdef VIGIA_HAVE_PAHO_MQTT

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "vigia_msgs/msg/hazard_event.hpp"

// HazardUplinkNode — continuous MQTT attestation uplink.
//
// Subscribes to /vigia/hazard_events. For every event it:
//   1. Serialises a MsgPack payload matching the attestation.py schema (doc 05 §5).
//   2. Publishes to vigia/attest/{device_id}/hazard (QoS 1, persistent session).
//
// The ECDSA signature already lives inside SignedEt (produced by the Pico 2
// ATECC608A). This node just bundles and ships it — no signing happens here.
//
// TLS mutual auth uses the per-device cert issued by vigia-sign-device.sh.
// Set verify_ecdsa=true in sensor_bridge_params.yaml once the SE is provisioned.
class HazardUplinkNode : public rclcpp::Node {
public:
    explicit HazardUplinkNode(const rclcpp::NodeOptions& options);
    ~HazardUplinkNode();

private:
    void on_hazard(vigia_msgs::msg::HazardEvent::ConstSharedPtr msg);
    bool connect_mqtt();
    std::vector<uint8_t> pack_event(
        const vigia_msgs::msg::HazardEvent& ev) const;

    rclcpp::Subscription<vigia_msgs::msg::HazardEvent>::SharedPtr sub_;

    std::string device_id_;
    std::string broker_host_;
    int         broker_port_;
    std::string cert_path_;
    std::string key_path_;
    std::string ca_path_;

    void*       mqtt_client_{nullptr};  // opaque mqtt::async_client*
    std::atomic<bool> connected_{false};
};

#endif // VIGIA_HAVE_PAHO_MQTT

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "vigia_edge_node/vigia_qos.hpp"
#include "vigia_msgs/msg/spatial_latent.hpp"
#include "vigia_msgs/msg/signed_et.hpp"
#include "vigia_msgs/msg/hazard_event.hpp"

// Anti-death state machine states (§ Phase 5 spec).
enum class AntiDeathState : uint8_t {
    MONITORING = 0,
    CAPTURE_SNAPSHOT,
    SERIALIZE,
    MQTT_CONNECT,
    MQTT_TRANSMIT,
    SAFE_SHUTDOWN
};

class AntiDeathNode : public rclcpp::Node {
public:
    explicit AntiDeathNode(const rclcpp::NodeOptions & options);
    ~AntiDeathNode();

private:
    void on_spatial_latent(std::unique_ptr<vigia_msgs::msg::SpatialLatent> msg);
    void on_signed_et(std::shared_ptr<const vigia_msgs::msg::SignedEt> msg);
    void on_hazard_event(std::unique_ptr<vigia_msgs::msg::HazardEvent> msg);

    // GPIO monitor loop — runs in a separate thread within the executor.
    void gpio_monitor_loop();
    void run_emergency_sequence();

    rclcpp::Subscription<vigia_msgs::msg::SpatialLatent>::SharedPtr sub_latent_;
    rclcpp::Subscription<vigia_msgs::msg::SignedEt>::SharedPtr       sub_et_;
    rclcpp::Subscription<vigia_msgs::msg::HazardEvent>::SharedPtr    sub_hazard_;

    // Latest snapshot — written by subscriber callbacks, read by emergency handler.
    // Protected by seqlock on latest_seq_.
    std::atomic<uint32_t> latest_seq_{0};
    std::shared_ptr<const vigia_msgs::msg::SpatialLatent> latest_latent_;
    std::shared_ptr<const vigia_msgs::msg::SignedEt>       latest_et_;
    std::shared_ptr<const vigia_msgs::msg::HazardEvent>    latest_hazard_;

    std::atomic<bool>  power_fail_{false};
    std::atomic<bool>  shutdown_requested_{false};
    std::thread        gpio_thread_;

    AntiDeathState state_{AntiDeathState::MONITORING};

    std::string ups_gpio_chip_;
    int         ups_gpio_line_{17};
    bool        ups_gpio_active_low_{true};
    double      power_window_seconds_{15.0};
    std::string mqtt_broker_host_;
    int         mqtt_broker_port_{8883};
    std::string mqtt_topic_prefix_;
};

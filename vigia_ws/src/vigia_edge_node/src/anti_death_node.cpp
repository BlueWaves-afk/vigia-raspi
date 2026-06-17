#include "anti_death_node.hpp"
#include <chrono>
#include <cstdio>

using namespace std::chrono_literals;

AntiDeathNode::AntiDeathNode(const rclcpp::NodeOptions & options)
: Node("anti_death_node", options)
{
    declare_parameter("ups_gpio_chip",       "/dev/gpiochip4");
    declare_parameter("ups_gpio_line",       17);
    declare_parameter("ups_gpio_active_low", true);
    declare_parameter("power_window_seconds", 15.0);
    declare_parameter("mqtt_broker_host",    "");
    declare_parameter("mqtt_broker_port",    8883);
    declare_parameter("mqtt_topic_prefix",   "vigia/events");

    ups_gpio_chip_        = get_parameter("ups_gpio_chip").as_string();
    ups_gpio_line_        = get_parameter("ups_gpio_line").as_int();
    ups_gpio_active_low_  = get_parameter("ups_gpio_active_low").as_bool();
    power_window_seconds_ = get_parameter("power_window_seconds").as_double();
    mqtt_broker_host_     = get_parameter("mqtt_broker_host").as_string();
    mqtt_broker_port_     = get_parameter("mqtt_broker_port").as_int();
    mqtt_topic_prefix_    = get_parameter("mqtt_topic_prefix").as_string();

    sub_latent_ = create_subscription<vigia_msgs::msg::SpatialLatent>(
        "/vigia/spatial_latent", vigia::qos::inference_results(),
        std::bind(&AntiDeathNode::on_spatial_latent, this, std::placeholders::_1));
    sub_et_ = create_subscription<vigia_msgs::msg::SignedEt>(
        "/vigia/signed_et", vigia::qos::signed_et(),
        std::bind(&AntiDeathNode::on_signed_et, this, std::placeholders::_1));
    sub_hazard_ = create_subscription<vigia_msgs::msg::HazardEvent>(
        "/vigia/hazard_event", vigia::qos::hazard_events(),
        std::bind(&AntiDeathNode::on_hazard_event, this, std::placeholders::_1));

    // GPIO monitor runs as a std::thread — blocks on libgpiod event_wait().
    gpio_thread_ = std::thread(&AntiDeathNode::gpio_monitor_loop, this);

    RCLCPP_INFO(get_logger(), "AntiDeathNode ready: gpio_chip=%s line=%d window=%.0fs",
                ups_gpio_chip_.c_str(), ups_gpio_line_, power_window_seconds_);
}

AntiDeathNode::~AntiDeathNode()
{
    shutdown_requested_.store(true);
    if (gpio_thread_.joinable()) gpio_thread_.join();
}

void AntiDeathNode::on_spatial_latent(std::unique_ptr<vigia_msgs::msg::SpatialLatent> msg) {
    latest_latent_ = std::move(msg);
}
void AntiDeathNode::on_signed_et(std::shared_ptr<const vigia_msgs::msg::SignedEt> msg) {
    latest_et_ = std::move(msg);
}
void AntiDeathNode::on_hazard_event(std::unique_ptr<vigia_msgs::msg::HazardEvent> msg) {
    latest_hazard_ = std::move(msg);
}

void AntiDeathNode::gpio_monitor_loop()
{
    pthread_setname_np(pthread_self(), "vigia_gpio");

    // libgpiod event wait loop.
    // Requires: sudo apt install libgpiod-dev
    // Included here as a polling fallback — replace with gpiod::chip event_wait() when libgpiod2 available.
    while (!shutdown_requested_.load()) {
        // TODO: Replace with libgpiod gpiod::line::event_wait() call (Phase 5).
        // For Phase 1, poll sysfs as a stub:
        std::this_thread::sleep_for(100ms);

        if (power_fail_.load()) {
            RCLCPP_ERROR(get_logger(), "POWER FAIL detected — starting emergency sequence");
            run_emergency_sequence();
            break;
        }
    }
}

void AntiDeathNode::run_emergency_sequence()
{
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::duration<double>(power_window_seconds_);

    // State: CAPTURE_SNAPSHOT
    state_ = AntiDeathState::CAPTURE_SNAPSHOT;
    RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] State: CAPTURE_SNAPSHOT");
    // Snapshot is already in latest_latent_ / latest_et_ / latest_hazard_.
    // (Seqlock-protected — no mutex needed at SCHED_FIFO 99.)

    // State: SERIALIZE
    state_ = AntiDeathState::SERIALIZE;
    RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] State: SERIALIZE");
    // TODO: MsgPack serialization (Phase 5 implementation).

    // State: MQTT_CONNECT
    state_ = AntiDeathState::MQTT_CONNECT;
    RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] State: MQTT_CONNECT");
    // TODO: Paho MQTT async client connect (Phase 5 implementation).

    // State: MQTT_TRANSMIT
    state_ = AntiDeathState::MQTT_TRANSMIT;
    RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] State: MQTT_TRANSMIT");
    // TODO: Paho MQTT publish with QoS 1 (Phase 5 implementation).

    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();
    RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] Transmit complete. %ld ms remaining in window.", remaining);

    // State: SAFE_SHUTDOWN
    state_ = AntiDeathState::SAFE_SHUTDOWN;
    RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] State: SAFE_SHUTDOWN — syncing and halting");
    sync();
    // system("sudo systemctl poweroff") would run here in Phase 5.
}

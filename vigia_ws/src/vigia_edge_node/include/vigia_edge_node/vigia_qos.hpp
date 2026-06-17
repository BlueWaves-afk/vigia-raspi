#pragma once
#include <rclcpp/rclcpp.hpp>

namespace vigia::qos {

inline rclcpp::QoS sensor_stream() {
    return rclcpp::QoS(rclcpp::KeepLast(1))
        .best_effort()
        .durability_volatile();
}

inline rclcpp::QoS camera_frames() {
    return rclcpp::QoS(rclcpp::KeepLast(4))
        .best_effort()
        .durability_volatile();
}

inline rclcpp::QoS inference_results() {
    return rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .durability_volatile();
}

inline rclcpp::QoS hazard_events() {
    return rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .transient_local();
}

inline rclcpp::QoS signed_et() {
    return rclcpp::QoS(rclcpp::KeepLast(5))
        .reliable()
        .durability_volatile();
}

}  // namespace vigia::qos

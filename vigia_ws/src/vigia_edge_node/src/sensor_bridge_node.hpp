#pragma once
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <cstdint>
#include <string>

#include "vigia_edge_node/vigia_qos.hpp"
#include "vigia_msgs/msg/imu_sample.hpp"
#include "vigia_msgs/msg/gps_pvt.hpp"
#include "vigia_msgs/msg/signed_et.hpp"

// COBS decoder state machine — no dynamic allocation.
struct CobsDecoder {
    static constexpr size_t kMaxPacket = 512;
    uint8_t  buf[kMaxPacket]{};
    size_t   len{0};
    uint8_t  code{0};
    uint8_t  copy{0};
    bool     frame_ready{false};

    // Feed one byte. Returns true when a complete frame is ready in buf[0..len-1].
    bool feed(uint8_t byte);
    void reset();
};

class SensorBridgeNode : public rclcpp::Node {
public:
    explicit SensorBridgeNode(const rclcpp::NodeOptions & options);

private:
    void poll_serial();
    bool open_serial(const std::string & port, int baud);
    void dispatch_packet(const uint8_t* data, size_t len);

    rclcpp::Publisher<vigia_msgs::msg::ImuSample>::SharedPtr  pub_imu_;
    rclcpp::Publisher<vigia_msgs::msg::GpsPvt>::SharedPtr     pub_gps_;
    rclcpp::Publisher<vigia_msgs::msg::SignedEt>::SharedPtr   pub_et_;
    rclcpp::TimerBase::SharedPtr                              timer_;

    CobsDecoder decoder_;
    int          serial_fd_{-1};
    uint32_t     last_sequence_{0};
    uint32_t     sig_fail_counter_{0};
    bool         ecdsa_verify_enabled_{true};

    std::string serial_port_;
    int         baud_rate_{921600};
    std::string device_cert_path_;
};

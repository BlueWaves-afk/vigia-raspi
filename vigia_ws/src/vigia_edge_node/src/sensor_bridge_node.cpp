#include "sensor_bridge_node.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

using namespace std::chrono_literals;

// Packet type IDs — must match STM32 firmware definitions.
static constexpr uint8_t PKT_IMU       = 0x01;
static constexpr uint8_t PKT_GPS       = 0x02;
static constexpr uint8_t PKT_SIGNED_ET = 0x03;

// --------------------------------------------------------------------------
// COBS decoder
// --------------------------------------------------------------------------

bool CobsDecoder::feed(uint8_t byte)
{
    frame_ready = false;
    if (byte == 0x00) {
        // End of COBS frame.
        if (len > 0) {
            frame_ready = true;
            return true;
        }
        reset();
        return false;
    }
    if (copy == 0) {
        if (code != 0xFF) {
            if (len < kMaxPacket) buf[len++] = 0x00;
        }
        code = byte;
        copy = byte;
    } else {
        --copy;
        if (len < kMaxPacket) buf[len++] = byte;
        if (copy == 0 && code != 0xFF) {
            // consumed a block
        }
    }
    if (copy == 0) {
        code = 0;
    }
    return false;
}

void CobsDecoder::reset()
{
    len = code = copy = 0;
    frame_ready = false;
}

// --------------------------------------------------------------------------
// SensorBridgeNode
// --------------------------------------------------------------------------

SensorBridgeNode::SensorBridgeNode(const rclcpp::NodeOptions & options)
: Node("sensor_bridge_node", options)
{
    declare_parameter("serial_port",          "/dev/ttyACM0");
    declare_parameter("baud_rate",            921600);
    declare_parameter("ecdsa_verify_enabled", true);
    declare_parameter("device_cert_path",     "/etc/vigia/device_cert.pem");

    serial_port_          = get_parameter("serial_port").as_string();
    baud_rate_            = get_parameter("baud_rate").as_int();
    ecdsa_verify_enabled_ = get_parameter("ecdsa_verify_enabled").as_bool();
    device_cert_path_     = get_parameter("device_cert_path").as_string();

    pub_imu_ = create_publisher<vigia_msgs::msg::ImuSample>("/vigia/imu", vigia::qos::sensor_stream());
    pub_gps_ = create_publisher<vigia_msgs::msg::GpsPvt>("/vigia/gps", vigia::qos::sensor_stream());
    pub_et_  = create_publisher<vigia_msgs::msg::SignedEt>("/vigia/signed_et", vigia::qos::signed_et());

    if (open_serial(serial_port_, baud_rate_)) {
        RCLCPP_INFO(get_logger(), "SensorBridgeNode: %s @ %d baud", serial_port_.c_str(), baud_rate_);
    } else {
        RCLCPP_WARN(get_logger(), "SensorBridgeNode: %s not available — running without STM32",
                    serial_port_.c_str());
    }

    // 1 ms poll timer — at 921600 baud, a 34-byte packet arrives in ~0.37 ms.
    timer_ = create_wall_timer(1ms, std::bind(&SensorBridgeNode::poll_serial, this));
}

bool SensorBridgeNode::open_serial(const std::string & port, int baud)
{
    serial_fd_ = ::open(port.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) return false;

    termios tty{};
    tcgetattr(serial_fd_, &tty);
    cfsetispeed(&tty, B921600);
    tty.c_cflag  = CS8 | CLOCAL | CREAD;
    tty.c_iflag  = IGNPAR;
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(serial_fd_, TCSANOW, &tty);
    return true;
}

void SensorBridgeNode::poll_serial()
{
    if (serial_fd_ < 0) return;

    uint8_t byte_buf[256];
    ssize_t n = ::read(serial_fd_, byte_buf, sizeof(byte_buf));
    if (n <= 0) return;

    for (ssize_t i = 0; i < n; ++i) {
        decoder_.feed(byte_buf[i]);
        if (decoder_.frame_ready) {
            dispatch_packet(decoder_.buf, decoder_.len);
            decoder_.reset();
        }
    }
}

void SensorBridgeNode::dispatch_packet(const uint8_t* data, size_t len)
{
    if (len < 1) return;
    uint8_t pkt_type = data[0];

    if (pkt_type == PKT_IMU && len >= 34) {
        // Layout: [type(1)] [q_w,q_x,q_y,q_z,ax,ay,az : 7×float32] [cal(1)]
        auto msg = std::make_unique<vigia_msgs::msg::ImuSample>();
        msg->header.stamp = now();
        float vals[7];
        std::memcpy(vals, data + 1, 28);
        msg->q_w          = vals[0];
        msg->q_x          = vals[1];
        msg->q_y          = vals[2];
        msg->q_z          = vals[3];
        msg->lin_accel_x  = vals[4];
        msg->lin_accel_y  = vals[5];
        msg->lin_accel_z  = vals[6];
        msg->calibration_status = data[29];
        pub_imu_->publish(std::move(msg));
        return;
    }

    if (pkt_type == PKT_GPS && len >= 26) {
        // Layout: [type(1)] [lat,lon : 2×float64] [alt,speed,course,hdop : 4×float32]
        //         [fix_type(1)] [sats(1)] [valid(1)]
        auto msg = std::make_unique<vigia_msgs::msg::GpsPvt>();
        msg->header.stamp = now();
        std::memcpy(&msg->latitude,  data + 1, 8);
        std::memcpy(&msg->longitude, data + 9, 8);
        float f4[4];
        std::memcpy(f4, data + 17, 16);
        msg->altitude_m  = f4[0];
        msg->speed_ms    = f4[1];
        msg->course_deg  = f4[2];
        msg->hdop        = f4[3];
        msg->fix_type         = data[33];
        msg->satellites_used  = data[34];
        msg->valid_fix        = (data[35] != 0);
        pub_gps_->publish(std::move(msg));
        return;
    }

    if (pkt_type == PKT_SIGNED_ET && len >= 101) {
        uint32_t seq;
        std::memcpy(&seq, data + 1, 4);

        // Anti-replay: drop out-of-order or repeated sequence numbers.
        if (seq <= last_sequence_) {
            RCLCPP_WARN(get_logger(), "SignedEt replay? seq=%u last=%u — dropped", seq, last_sequence_);
            return;
        }

        // TODO: ECDSA verification via mbedTLS when ecdsa_verify_enabled_ is true.
        // For Phase 1 STM32 integration stub, verification is skipped.
        if (ecdsa_verify_enabled_) {
            // mbedtls_pk_verify() call here — implemented in Phase 2.
        }

        last_sequence_ = seq;
        auto msg = std::make_unique<vigia_msgs::msg::SignedEt>();
        msg->header.stamp = now();
        msg->sequence     = seq;
        std::memcpy(&msg->stm32_timestamp_us, data + 5, 8);
        std::memcpy(msg->et_hash.data(),       data + 13, 32);
        std::memcpy(msg->ecdsa_signature.data(), data + 45, 64);
        pub_et_->publish(std::move(msg));
    }
}

#ifdef VIGIA_HAVE_PAHO_MQTT
#include "hazard_uplink_node.hpp"

#include <mqtt/async_client.h>
#include <msgpack.hpp>
#include <sstream>
#include <iomanip>

static constexpr int kQos          = 1;
static constexpr int kConnTimeout  = 10;   // seconds
static constexpr int kPayloadVer   = 1;

// ── hex helper ───────────────────────────────────────────────────────────────
static std::string to_hex(const uint8_t* data, size_t len)
{
    std::ostringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < len; ++i)
        ss << std::setw(2) << static_cast<unsigned>(data[i]);
    return ss.str();
}

// ── constructor ───────────────────────────────────────────────────────────────
HazardUplinkNode::HazardUplinkNode(const rclcpp::NodeOptions& options)
: Node("hazard_uplink_node", options)
{
    device_id_   = declare_parameter<std::string>("device_id",    "vigia-001");
    broker_host_ = declare_parameter<std::string>("mqtt_broker_host", "");
    broker_port_ = declare_parameter<int>        ("mqtt_port",        8883);
    cert_path_   = declare_parameter<std::string>("mqtt_cert_path",
                       "/etc/vigia/device_cert.pem");
    key_path_    = declare_parameter<std::string>("mqtt_key_path",
                       "/etc/vigia/device.key");
    // AWS IoT Core CA — download from https://www.amazontrust.com/repository/AmazonRootCA1.pem
    // and install to /etc/vigia/AmazonRootCA1.pem on each Pi before first boot.
    ca_path_     = declare_parameter<std::string>("mqtt_ca_path",
                       "/etc/vigia/AmazonRootCA1.pem");

    if (broker_host_.empty()) {
        RCLCPP_WARN(get_logger(),
            "HazardUplinkNode: mqtt_broker_host not set — uplink disabled. "
            "Set via param or MQTT_BROKER_HOST env.");
        return;
    }

    if (!connect_mqtt()) {
        RCLCPP_ERROR(get_logger(),
            "HazardUplinkNode: initial MQTT connect failed — will retry on first event.");
    }

    // R-CRIT-4: topic + QoS must match FusionNode's publisher exactly
    // ("/vigia/hazard_event", vigia::qos::hazard_events()) or no events arrive.
    sub_ = create_subscription<vigia_msgs::msg::HazardEvent>(
        "/vigia/hazard_event", vigia::qos::hazard_events(),
        std::bind(&HazardUplinkNode::on_hazard, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
        "HazardUplinkNode started — broker=%s:%d device=%s",
        broker_host_.c_str(), broker_port_, device_id_.c_str());
}

HazardUplinkNode::~HazardUplinkNode()
{
    if (mqtt_client_) {
        auto* c = static_cast<mqtt::async_client*>(mqtt_client_);
        if (c->is_connected()) c->disconnect()->wait();
        delete c;
    }
}

// ── MQTT connect (TLS mTLS) ───────────────────────────────────────────────────
bool HazardUplinkNode::connect_mqtt()
{
    const std::string server_uri =
        "ssl://" + broker_host_ + ":" + std::to_string(broker_port_);
    // client_id MUST equal device_id exactly — IoT Core policy grants
    // iot:Connect to arn:.../client/${iot:ClientId} and iot:Publish to
    // vigia/attest/${iot:ClientId}/hazard. Any prefix breaks the policy match.
    const std::string client_id = device_id_;

    auto* c = new mqtt::async_client(server_uri, client_id);
    mqtt_client_ = c;

    mqtt::ssl_options ssl;
    ssl.set_trust_store(ca_path_);
    ssl.set_key_store(cert_path_);
    ssl.set_private_key(key_path_);
    ssl.set_ssl_version(MQTT_SSL_VERSION_TLS_1_2);
    ssl.set_verify(true);

    mqtt::connect_options opts;
    opts.set_ssl(ssl);
    opts.set_keep_alive_interval(20);
    opts.set_clean_session(false);   // QoS 1 persistence across reconnects
    opts.set_automatic_reconnect(true);
    opts.set_connect_timeout(std::chrono::seconds(kConnTimeout));

    // Last-will: mark device offline on unexpected disconnect.
    const std::string will_topic = "vigia/status/" + device_id_ + "/health";
    opts.set_will(mqtt::message(will_topic,
        R"({"status":"offline","reason":"unexpected_disconnect"})", kQos, false));

    try {
        c->connect(opts)->wait();
        connected_.store(true, std::memory_order_relaxed);
        RCLCPP_INFO(get_logger(), "MQTT connected to %s", server_uri.c_str());
        return true;
    } catch (const mqtt::exception& ex) {
        RCLCPP_ERROR(get_logger(), "MQTT connect failed: %s", ex.what());
        return false;
    }
}

// ── MsgPack serialiser (schema: attestation.py decode_payload) ───────────────
std::vector<uint8_t> HazardUplinkNode::pack_event(
    const vigia_msgs::msg::HazardEvent& ev) const
{
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);

    const auto& se  = ev.signed_et;
    const auto& gps = ev.gps_pvt;
    const auto& imu = ev.imu_sample;

    // Root map — 7 keys (doc 05 §5 schema)
    pk.pack_map(7);

    pk.pack("version");   pk.pack(kPayloadVer);
    pk.pack("device_id"); pk.pack(device_id_);

    // signed_et — fields consumed by reconstruct_et_hash_input()
    pk.pack("signed_et");
    pk.pack_map(15);
    pk.pack("sequence");         pk.pack(se.sequence);
    pk.pack("mcu_timestamp_us"); pk.pack(se.timestamp_us);
    pk.pack("et_hash");
        pk.pack_bin(32);
        pk.pack_bin_body(reinterpret_cast<const char*>(se.et_hash.data()), 32);
    pk.pack("ecdsa_sig");
        pk.pack_bin(64);
        pk.pack_bin_body(reinterpret_cast<const char*>(se.ecdsa_sig.data()), 64);
    pk.pack("sig_valid");        pk.pack(se.sig_valid);
    pk.pack("q_w");              pk.pack(se.q_w);
    pk.pack("q_x");              pk.pack(se.q_x);
    pk.pack("q_y");              pk.pack(se.q_y);
    pk.pack("q_z");              pk.pack(se.q_z);
    pk.pack("accel_x");          pk.pack(se.lin_accel_x);
    pk.pack("accel_y");          pk.pack(se.lin_accel_y);
    pk.pack("accel_z");          pk.pack(se.lin_accel_z);
    pk.pack("imu_cal_status");   pk.pack(static_cast<uint8_t>(se.calibration_status));
    pk.pack("gps_fix_type");     pk.pack(static_cast<uint8_t>(gps.fix_type));
    pk.pack("valid");            pk.pack(se.sig_valid);

    // frame_metadata — single-element array with GPS fields
    pk.pack("frame_metadata");
    pk.pack_array(1);
    pk.pack_map(5);
    pk.pack("lat");      pk.pack(gps.lat);
    pk.pack("lon");      pk.pack(gps.lon);
    pk.pack("alt_m");    pk.pack(gps.altitude_m);
    pk.pack("speed_ms"); pk.pack(gps.speed_ms);
    pk.pack("hdop");     pk.pack(gps.hdop);

    // spatial_latent — raw float32 LE bytes
    pk.pack("spatial_latent");
    const size_t latent_bytes = ev.spatial_latent.size() * sizeof(float);
    pk.pack_bin(static_cast<uint32_t>(latent_bytes));
    pk.pack_bin_body(reinterpret_cast<const char*>(ev.spatial_latent.data()), latent_bytes);

    // hazard_event sub-map
    pk.pack("hazard_event");
    pk.pack_map(4);
    pk.pack("rri_score");        pk.pack(ev.rri_score);
    pk.pack("iss_score");        pk.pack(ev.iss_score);
    pk.pack("yolo_confidence");  pk.pack(ev.yolo_confidence);
    pk.pack("degraded");         pk.pack(ev.degraded);

    // capture_timestamp_us
    pk.pack("capture_timestamp_us");
    pk.pack(static_cast<uint64_t>(
        ev.header.stamp.sec * 1'000'000ULL + ev.header.stamp.nanosec / 1000ULL));

    return std::vector<uint8_t>(buf.data(), buf.data() + buf.size());
}

// ── Event callback ────────────────────────────────────────────────────────────
void HazardUplinkNode::on_hazard(vigia_msgs::msg::HazardEvent::ConstSharedPtr msg)
{
    if (broker_host_.empty()) return;

    auto* c = static_cast<mqtt::async_client*>(mqtt_client_);
    if (!c || (!c->is_connected() && !connect_mqtt())) {
        RCLCPP_WARN(get_logger(), "MQTT not connected — dropping event seq=%u",
                    msg->signed_et.sequence);
        return;
    }

    const auto payload = pack_event(*msg);
    const std::string topic = "vigia/attest/" + device_id_ + "/hazard";

    try {
        c->publish(topic, payload.data(), payload.size(), kQos, /*retained=*/false)->wait();
        RCLCPP_DEBUG(get_logger(),
            "Published %zu bytes to %s (seq=%u sig_valid=%d rri=%.3f)",
            payload.size(), topic.c_str(),
            msg->signed_et.sequence,
            static_cast<int>(msg->signed_et.sig_valid),
            static_cast<double>(msg->rri_score));
    } catch (const mqtt::exception& ex) {
        RCLCPP_ERROR(get_logger(), "MQTT publish failed: %s", ex.what());
    }
}

#endif // VIGIA_HAVE_PAHO_MQTT

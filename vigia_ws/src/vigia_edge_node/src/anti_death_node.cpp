// anti_death_node.cpp — Phase 5 Anti-Death Storage & MQTT Uplink
//
// Implements doc 05 (05_anti_death_and_depin_contracts.md) in full.
// When VIGIA_HAVE_PAHO_MQTT is defined: Paho async_client + MsgPack path.
// When not defined: falls back to the Phase 1 HTTPS POST path (legacy).
//
// Key invariants (doc 05 §0):
//   - execute_emergency_sequence() runs synchronously on SCHED_FIFO 99 timer thread
//   - steady_clock only for all budget checks
//   - single msgpack::sbuffer allocation permitted inside the emergency window
//   - atomic_flag prevents re-entry

#include "anti_death_node.hpp"

#include <pthread.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>

#include <algorithm>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>
#include <iomanip>

// Watchdog notification (systemd sd_notify) — included when available.
// Falls back gracefully to no-op.
#if __has_include(<systemd/sd-daemon.h>)
#  include <systemd/sd-daemon.h>
#  define VIGIA_WATCHDOG_PING() sd_notify(0, "WATCHDOG=1")
#else
#  define VIGIA_WATCHDOG_PING() ((void)0)
#endif

// When Paho not available, keep legacy HTTPS POST for emergency uplink.
#ifndef VIGIA_HAVE_PAHO_MQTT
#  include <curl/curl.h>
#  include <sodium.h>
static const char kB58Chars[] = "123456789ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnopqrstuvwxyz";
static size_t curl_write_cb(void * ptr, size_t sz, size_t nm, std::string * s) {
    s->append(static_cast<char*>(ptr), sz * nm);
    return sz * nm;
}
#endif

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────

AntiDeathNode::AntiDeathNode(const rclcpp::NodeOptions & opts)
: Node("anti_death_node", opts)
{
    declare_parameter("ups_gpio_chip",      "/dev/gpiochip4");
    declare_parameter("ups_gpio_line",      17);
    declare_parameter("device_id",          "vigia-001");
    declare_parameter("mqtt_broker_host",   "");
    declare_parameter("mqtt_broker_port",   8883);
    declare_parameter("mqtt_topic_prefix",  "vigia/attest");
    declare_parameter("sim7600_port",       "/dev/ttyUSB2");
    declare_parameter("emergency_budget_sec", 13.0);

    // Legacy params (used when Paho not available)
    declare_parameter("aws_telemetry_url",  "");
    declare_parameter("device_key_path",    "/etc/vigia/device_ed25519.key");

    ups_gpio_chip_    = get_parameter("ups_gpio_chip").as_string();
    ups_gpio_line_    = get_parameter("ups_gpio_line").as_int();
    mqtt_broker_host_ = get_parameter("mqtt_broker_host").as_string();
    mqtt_broker_port_ = get_parameter("mqtt_broker_port").as_int();
    mqtt_topic_prefix_= get_parameter("mqtt_topic_prefix").as_string();
    sim7600_port_     = get_parameter("sim7600_port").as_string();
    emergency_budget_sec_ = get_parameter("emergency_budget_sec").as_double();

    // Prefer device_id from /etc/vigia/device_id file; param is the fallback.
    {
        std::ifstream id_file("/etc/vigia/device_id");
        if (id_file) {
            id_file >> device_id_;
        } else {
            device_id_ = get_parameter("device_id").as_string();
        }
    }

    // Pre-allocate double-buffer latent vector capacity.
    // kMaxLatentElems = 102400 (320×320 = 102400 for penultimate layer).
    constexpr size_t kMaxLatentElems = 102400;
    latest_latent_a_.latent_vector.reserve(kMaxLatentElems);
    latest_latent_b_.latent_vector.reserve(kMaxLatentElems);
    active_latent_buf_.store(&latest_latent_a_, std::memory_order_relaxed);

    sub_latent_ = create_subscription<vigia_msgs::msg::SpatialLatent>(
        "/vigia/spatial_latent", vigia::qos::inference_results(),
        std::bind(&AntiDeathNode::on_spatial_latent, this, std::placeholders::_1));
    sub_et_ = create_subscription<vigia_msgs::msg::SignedEt>(
        "/vigia/signed_et", vigia::qos::signed_et(),
        std::bind(&AntiDeathNode::on_signed_et, this, std::placeholders::_1));
    sub_hazard_ = create_subscription<vigia_msgs::msg::HazardEvent>(
        "/vigia/hazard_event", vigia::qos::hazard_events(),
        std::bind(&AntiDeathNode::on_hazard_event, this, std::placeholders::_1));

    // All blocking startup operations (LTE init, MQTT connect, shm map).
    startup_init();

    // 1 ms GPIO poll timer on the node's executor.
    gpio_timer_ = create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&AntiDeathNode::gpio_poll_callback, this));

    RCLCPP_INFO(get_logger(),
        "AntiDeathNode armed: device=%s gpio=%s:%d budget=%.0fs",
        device_id_.c_str(), ups_gpio_chip_.c_str(), ups_gpio_line_, emergency_budget_sec_);
}

AntiDeathNode::~AntiDeathNode()
{
#ifdef VIGIA_HAVE_LIBGPIOD
    if (gpio_line_) gpiod_line_release(gpio_line_);
    if (gpio_chip_) gpiod_chip_close(gpio_chip_);
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// Startup initialization (doc 05 §1) — blocking, never inside emergency window
// ─────────────────────────────────────────────────────────────────────────────

void AntiDeathNode::startup_init()
{
    // 1. SIM7600 LTE status check (AT commands → confirm bearer is up)
    sim7600_init();

    // 2. Paho MQTT pre-connect
    mqtt_client_init();

    // 3. Map FrameMetadataRing sidecar (reader side — FusionNode is creator)
    try {
        meta_ring_shm_ = std::make_unique<ShmMetaRing>(/*creator=*/false);
        meta_ring_ = meta_ring_shm_->ring();
        RCLCPP_INFO(get_logger(), "FrameMetadataRing mapped (%zu KB).",
                    sizeof(FrameMetadataRing) / 1024);
    } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(),
            "FrameMetadataRing not available (%s) — emergency snapshot will have no metadata.",
            e.what());
    }

    // 4. libgpiod setup (v1 API; v2 falls through to timer poll)
#ifdef VIGIA_HAVE_LIBGPIOD
    gpio_chip_ = gpiod_chip_open(ups_gpio_chip_.c_str());
    if (gpio_chip_) {
        gpio_line_ = gpiod_chip_get_line(gpio_chip_, ups_gpio_line_);
        if (gpio_line_) {
            if (gpiod_line_request_falling_edge_events(gpio_line_, "vigia_antideath") < 0) {
                RCLCPP_WARN(get_logger(), "gpiod_line_request failed — using timer poll");
                gpiod_line_ = nullptr;
            }
        }
    }
#endif

    RCLCPP_INFO(get_logger(), "startup_init complete.");
}

// ─────────────────────────────────────────────────────────────────────────────
// SIM7600 LTE initialization (doc 05 §6)
// ─────────────────────────────────────────────────────────────────────────────

void AntiDeathNode::sim7600_init()
{
    // Open AT command port — SIM7600 serial control interface.
    // ECM mode (usb0) must be activated by vigia-sim7600-init.sh before
    // this node starts (ExecStartPre in vigia-edge.service).
    // Here we only verify the bearer is live.
    int fd = ::open(sim7600_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        RCLCPP_WARN(get_logger(),
            "SIM7600 AT port %s unavailable (%s) — LTE status unknown.",
            sim7600_port_.c_str(), strerror(errno));
        return;
    }

    // Configure raw serial: 115200 8N1
    struct termios tio{};
    tcgetattr(fd, &tio);
    cfsetspeed(&tio, B115200);
    tio.c_cflag = CS8 | CREAD | CLOCAL;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tcsetattr(fd, TCSANOW, &tio);

    // Send AT — verify modem is responsive.
    auto at_cmd = [&](const char * cmd, int timeout_ms = 500) -> std::string {
        ::write(fd, cmd, strlen(cmd));
        ::write(fd, "\r\n", 2);
        std::string resp;
        resp.reserve(128);
        struct pollfd pfd{fd, POLLIN, 0};
        auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
        while (Clock::now() < deadline) {
            int r = ::poll(&pfd, 1, 50);
            if (r > 0 && (pfd.revents & POLLIN)) {
                char buf[64];
                ssize_t n = ::read(fd, buf, sizeof(buf));
                if (n > 0) resp.append(buf, n);
                if (resp.find("OK") != std::string::npos ||
                    resp.find("ERROR") != std::string::npos) break;
            }
        }
        return resp;
    };

    const std::string ping = at_cmd("AT");
    if (ping.find("OK") == std::string::npos) {
        RCLCPP_WARN(get_logger(), "SIM7600 not responding to AT command — bearer unknown.");
        ::close(fd);
        return;
    }

    // AT+CREG? — check network registration (1=home, 5=roaming)
    const std::string creg = at_cmd("AT+CREG?");
    const bool registered = (creg.find(",1") != std::string::npos ||
                              creg.find(",5") != std::string::npos);

    // AT+CSQ — signal quality (0–31; 99=unknown)
    const std::string csq_resp = at_cmd("AT+CSQ");
    int rssi_raw = 99;
    {
        const auto pos = csq_resp.find("+CSQ: ");
        if (pos != std::string::npos && pos + 6 < csq_resp.size())
            rssi_raw = std::stoi(csq_resp.substr(pos + 6));
    }
    // Convert RSSI: -113 + (2 × value) dBm
    const int rssi_dbm = (rssi_raw < 99) ? (-113 + 2 * rssi_raw) : -999;

    // AT+CGACT? — PDP context active
    const std::string cgact = at_cmd("AT+CGACT?");
    const bool bearer_up = (cgact.find(",1") != std::string::npos);

    ::close(fd);

    if (!registered) {
        RCLCPP_WARN(get_logger(), "SIM7600: not registered on network (CREG: %s)",
                    creg.substr(0, 40).c_str());
    }
    if (rssi_dbm < -100) {
        RCLCPP_WARN(get_logger(), "SIM7600: weak signal %d dBm — transmit may fail.", rssi_dbm);
    }
    if (!bearer_up) {
        RCLCPP_WARN(get_logger(), "SIM7600: PDP context not active — usb0 may be down.");
    }

    RCLCPP_INFO(get_logger(),
        "SIM7600 status: registered=%s rssi=%d dBm bearer=%s",
        registered ? "YES" : "NO", rssi_dbm, bearer_up ? "UP" : "DOWN");
}

// ─────────────────────────────────────────────────────────────────────────────
// Paho MQTT client initialization (doc 05 §7)
// ─────────────────────────────────────────────────────────────────────────────

void AntiDeathNode::mqtt_client_init()
{
#ifdef VIGIA_HAVE_PAHO_MQTT
    if (mqtt_broker_host_.empty()) {
        RCLCPP_WARN(get_logger(),
            "mqtt_broker_host not set — Paho MQTT disabled, falling back to HTTPS.");
        return;
    }

    const std::string client_id = "vigia_" + device_id_.substr(0, 8);
    const std::string broker_uri =
        "ssl://" + mqtt_broker_host_ + ":" + std::to_string(mqtt_broker_port_);

    mqtt_client_ = std::make_unique<mqtt::async_client>(
        broker_uri,
        client_id,
        /*max_buffered_messages=*/5,
        nullptr);

    // TLS 1.2 mutual authentication — certs written by provision_device.py
    mqtt::ssl_options ssl_opts = mqtt::ssl_options_builder()
        .trust_store("/etc/vigia/ca_chain.pem")
        .key_store("/etc/vigia/device_cert.pem")
        .private_key("/etc/vigia/device_key.pem")
        .ssl_version(MQTT_SSL_VERSION_TLS_1_2)
        .verify(true)
        .finalize();

    conn_opts_ = mqtt::connect_options_builder()
        .ssl(ssl_opts)
        .keep_alive_interval(std::chrono::seconds(20))
        .connect_timeout(std::chrono::seconds(5))
        .clean_session(false)  // persist QoS 1 state across reboots
        .will(mqtt::will_options(
            "vigia/status/" + device_id_, "OFFLINE", 1, true))
        .finalize();

    try {
        auto tok = mqtt_client_->connect(conn_opts_);
        if (!tok->wait_for(std::chrono::seconds(15))) {
            RCLCPP_WARN(get_logger(),
                "MQTT initial connect timed out — will retry during emergency.");
            return;
        }
        RCLCPP_INFO(get_logger(), "MQTT connected to %s (TLS 1.2 mTLS).", broker_uri.c_str());

        // Publish ONLINE status (LWT inverse)
        mqtt_client_->publish("vigia/status/" + device_id_, "ONLINE", 1, true)->wait();
    } catch (const mqtt::exception & ex) {
        RCLCPP_WARN(get_logger(), "MQTT connect failed: %s", ex.what());
    }
#else
    // Legacy path: initialize libsodium for Ed25519 signing
    if (sodium_init() < 0)
        RCLCPP_ERROR(get_logger(), "libsodium init failed — Ed25519 signing disabled");
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// Subscriber callbacks — double-buffer pattern (doc 05 §2)
// ─────────────────────────────────────────────────────────────────────────────

void AntiDeathNode::on_spatial_latent(std::unique_ptr<vigia_msgs::msg::SpatialLatent> msg)
{
    // Write into the INACTIVE buffer (not the one execute_emergency_sequence reads).
    auto * write_buf = (active_latent_buf_.load(std::memory_order_relaxed) == &latest_latent_a_)
                       ? &latest_latent_b_ : &latest_latent_a_;

    write_buf->header            = msg->header;
    write_buf->frame_id          = msg->frame_id;
    write_buf->source_layer_name = msg->source_layer_name;
    write_buf->latent_vector.resize(msg->latent_vector.size());
    std::memcpy(write_buf->latent_vector.data(),
                msg->latent_vector.data(),
                msg->latent_vector.size() * sizeof(float));

    // Atomic swap — emergency reader always sees a consistent buffer.
    active_latent_buf_.store(write_buf, std::memory_order_release);
}

void AntiDeathNode::on_signed_et(std::shared_ptr<const vigia_msgs::msg::SignedEt> msg)
{
    latest_signed_et_ = *msg;
    signed_et_valid_.store(true, std::memory_order_release);
}

void AntiDeathNode::on_hazard_event(std::unique_ptr<vigia_msgs::msg::HazardEvent> msg)
{
    latest_hazard_event_ = std::move(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
// GPIO poll callback — 1 ms timer (doc 05 §4.2)
// ─────────────────────────────────────────────────────────────────────────────

void AntiDeathNode::gpio_poll_callback()
{
    // Ping the systemd watchdog on every N-th tick (~every 10 s = 10000 ticks).
    static uint32_t tick = 0;
    if (++tick % 10000 == 0) VIGIA_WATCHDOG_PING();

    bool triggered = false;

#ifdef VIGIA_HAVE_LIBGPIOD
    if (gpio_line_) {
        struct timespec zero{0, 0};
        int ev = gpiod_line_event_wait(gpio_line_, &zero);
        if (ev == 1) {
            struct gpiod_line_event event;
            gpiod_line_event_read(gpio_line_, &event);
            triggered = (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE);
        }
    }
#endif

    if (!triggered) return;

    // Atomic guard — reject concurrent re-entry.
    bool expected = false;
    if (!emergency_in_progress_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
        RCLCPP_WARN(get_logger(), "UPS re-assertion during active sequence — ignored.");
        return;
    }

    t_zero_ = Clock::now();
    RCLCPP_FATAL(get_logger(),
        "UPS POWER_FAIL asserted. Emergency sequence initiated (window=%.0fs).",
        emergency_budget_sec_);

    // Synchronous — this call blocks until SAFE_SHUTDOWN.
    execute_emergency_sequence();
}

// ─────────────────────────────────────────────────────────────────────────────
// elapsed_seconds_since_t0
// ─────────────────────────────────────────────────────────────────────────────

double AntiDeathNode::elapsed_seconds_since_t0() const
{
    return std::chrono::duration<double>(Clock::now() - t_zero_).count();
}

// ─────────────────────────────────────────────────────────────────────────────
// execute_emergency_sequence (doc 05 §4.3)
// ─────────────────────────────────────────────────────────────────────────────

void AntiDeathNode::execute_emergency_sequence()
{
    EmergencyState state = EmergencyState::CAPTURING_SNAPSHOT;

#ifdef VIGIA_HAVE_PAHO_MQTT
    msgpack::sbuffer payload_buf;   // THE ONE PERMITTED HEAP ALLOCATION
#endif

    SnapshotData snapshot{};        // ~36 KB on stack (SCHED_FIFO 99 thread, 512 KB stack)

    while (state != EmergencyState::SAFE_SHUTDOWN &&
           state != EmergencyState::ABORTED)
    {
        const double elapsed = elapsed_seconds_since_t0();

        switch (state) {

        case EmergencyState::CAPTURING_SNAPSHOT:
            if (elapsed > StateBudget::kCaptureDeadlineS) {
                RCLCPP_ERROR(get_logger(),
                    "BUDGET EXCEEDED: CAPTURING_SNAPSHOT at T+%.2fs", elapsed);
                state = EmergencyState::ABORTED; break;
            }
            state = do_capture_snapshot(snapshot)
                    ? EmergencyState::SERIALIZING
                    : EmergencyState::ABORTED;
            break;

        case EmergencyState::SERIALIZING:
            if (elapsed > StateBudget::kSerializeDeadlineS) {
                RCLCPP_ERROR(get_logger(),
                    "BUDGET EXCEEDED: SERIALIZING at T+%.2fs", elapsed);
                state = EmergencyState::ABORTED; break;
            }
            state = do_serialize(snapshot
#ifdef VIGIA_HAVE_PAHO_MQTT
                                 , payload_buf
#endif
                                 )
                    ? EmergencyState::MQTT_CONNECTING
                    : EmergencyState::ABORTED;
            break;

        case EmergencyState::MQTT_CONNECTING:
            if (elapsed > StateBudget::kConnectDeadlineS) {
                RCLCPP_ERROR(get_logger(),
                    "BUDGET EXCEEDED: MQTT_CONNECTING at T+%.2fs", elapsed);
                state = EmergencyState::ABORTED; break;
            }
            state = do_mqtt_connect()
                    ? EmergencyState::MQTT_TRANSMITTING
                    : EmergencyState::ABORTED;
            break;

        case EmergencyState::MQTT_TRANSMITTING:
            if (elapsed > StateBudget::kTransmitDeadlineS) {
                RCLCPP_ERROR(get_logger(),
                    "BUDGET EXCEEDED: MQTT_TRANSMITTING at T+%.2fs", elapsed);
                state = EmergencyState::ABORTED; break;
            }
            do_mqtt_transmit(
#ifdef VIGIA_HAVE_PAHO_MQTT
                payload_buf
#endif
            );
            state = EmergencyState::SAFE_SHUTDOWN;
            break;

        default:
            state = EmergencyState::SAFE_SHUTDOWN;
        }
    }

    do_safe_shutdown();
}

// ─────────────────────────────────────────────────────────────────────────────
// do_capture_snapshot (doc 05 §4.4, budget T+0 → T+1.0s)
// ─────────────────────────────────────────────────────────────────────────────

bool AntiDeathNode::do_capture_snapshot(SnapshotData & snap)
{
    const auto t0 = Clock::now();

    // 1. Seqlock snapshot of FrameMetadataRing (~36 KB memcpy, ~15 µs)
    if (meta_ring_) {
        meta_ring_->snapshot(snap.frame_metadata, snap.metadata_write_idx);
    }

    // 2. Freeze spatial latent pointer (atomic load — no copy of the data).
    snap.spatial_latent_snapshot =
        active_latent_buf_.load(std::memory_order_acquire);

    // 3. Copy SignedEt (~200 B value copy)
    snap.signed_et_snapshot = latest_signed_et_;
    snap.signed_et_valid     = signed_et_valid_.load(std::memory_order_acquire);

    // 4. Freeze HazardEvent pointer
    snap.hazard_event_snapshot = latest_hazard_event_.get();

    // 5. Record capture metadata
    snap.capture_timestamp_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            t_zero_.time_since_epoch()).count();
    std::strncpy(snap.device_id, device_id_.c_str(), sizeof(snap.device_id) - 1);

    const double ms =
        std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    RCLCPP_INFO(get_logger(),
        "CAPTURING_SNAPSHOT: %.2f ms. frames=%u latent_elems=%zu signed_et=%s",
        ms,
        snap.metadata_write_idx,
        snap.spatial_latent_snapshot
            ? snap.spatial_latent_snapshot->latent_vector.size() : 0UL,
        snap.signed_et_valid ? "YES" : "NO");

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// MsgPack serialization helpers (VIGIA_HAVE_PAHO_MQTT path)
// ─────────────────────────────────────────────────────────────────────────────

#ifdef VIGIA_HAVE_PAHO_MQTT

void AntiDeathNode::pack_frame_metadata_array(
    msgpack::packer<msgpack::sbuffer> & pk,
    const SnapshotData & snap)
{
    // Serialize only frames that have been written (up to kMetaRingDepth).
    const uint32_t n = std::min(snap.metadata_write_idx,
                                static_cast<uint32_t>(kMetaRingDepth));
    pk.pack_array(n);

    // Write in chronological order (oldest first).
    const uint32_t start_idx = (snap.metadata_write_idx > kMetaRingDepth)
        ? (snap.metadata_write_idx - kMetaRingDepth) : 0;

    for (uint32_t i = 0; i < n; ++i) {
        const FrameMetadata & m = snap.frame_metadata[(start_idx + i) % kMetaRingDepth];
        pk.pack_map(14);
        pk.pack("frame_id");         pk.pack(m.frame_id);
        pk.pack("timestamp_us");     pk.pack(m.timestamp_us);
        pk.pack("rri_score");        pk.pack(m.rri_score);
        pk.pack("iss_score");        pk.pack(m.iss_score);
        pk.pack("q_w");              pk.pack(m.q_w);
        pk.pack("q_x");              pk.pack(m.q_x);
        pk.pack("q_y");              pk.pack(m.q_y);
        pk.pack("q_z");              pk.pack(m.q_z);
        pk.pack("accel_x");          pk.pack(m.lin_accel_x);
        pk.pack("accel_y");          pk.pack(m.lin_accel_y);
        pk.pack("accel_z");          pk.pack(m.lin_accel_z);
        pk.pack("lat");              pk.pack(m.latitude);
        pk.pack("lon");              pk.pack(m.longitude);
        pk.pack("depth_hash");
            pk.pack_bin(8);
            pk.pack_bin_body(reinterpret_cast<const char*>(m.depth_hash_trunc), 8);
    }
}

void AntiDeathNode::pack_signed_et(
    msgpack::packer<msgpack::sbuffer> & pk,
    const vigia_msgs::msg::SignedEt & et,
    bool valid)
{
    if (!valid) { pk.pack_nil(); return; }
    pk.pack_map(6);
    pk.pack("sequence");          pk.pack(et.sequence);
    pk.pack("mcu_timestamp_us"); pk.pack(et.timestamp_us);
    pk.pack("imu_cal_status");   pk.pack(et.calibration_status);
    pk.pack("gps_fix_type");     pk.pack(et.fix_type);
    pk.pack("et_hash");
        pk.pack_bin(32);
        pk.pack_bin_body(reinterpret_cast<const char*>(et.et_hash.data()), 32);
    pk.pack("ecdsa_sig");
        pk.pack_bin(64);
        pk.pack_bin_body(reinterpret_cast<const char*>(et.ecdsa_sig.data()), 64);
}

void AntiDeathNode::pack_hazard_event(
    msgpack::packer<msgpack::sbuffer> & pk,
    const vigia_msgs::msg::HazardEvent & ev)
{
    pk.pack_map(7);
    pk.pack("rri_score");           pk.pack(ev.rri_score);
    pk.pack("iss_score");           pk.pack(ev.iss_score);
    pk.pack("yolo_confidence");     pk.pack(ev.yolo_confidence);
    pk.pack("lat");                 pk.pack(ev.gps_pvt.latitude);
    pk.pack("lon");                 pk.pack(ev.gps_pvt.longitude);
    pk.pack("speed_ms");            pk.pack(ev.gps_pvt.speed_ms);
    pk.pack("detection_count");
        pk.pack(static_cast<uint8_t>(ev.detections.detections.size()));
}

// ─────────────────────────────────────────────────────────────────────────────
// do_serialize — MsgPack encoding (doc 05 §4.5, budget T+1.0s → T+3.0s)
// ─────────────────────────────────────────────────────────────────────────────

bool AntiDeathNode::do_serialize(const SnapshotData & snap, msgpack::sbuffer & buf)
{
    // Single upfront reserve — no realloc during packing.
    // 500 KB covers the estimated ~440 KB payload (doc 05 §5).
    buf.reserve(500 * 1024);

    msgpack::packer<msgpack::sbuffer> pk(buf);

    pk.pack_map(7);

    pk.pack("version");
        pk.pack(uint8_t{1});
    pk.pack("device_id");
        pk.pack(std::string(snap.device_id));
    pk.pack("capture_timestamp_us");
        pk.pack(snap.capture_timestamp_us);
    pk.pack("frame_metadata");
        pack_frame_metadata_array(pk, snap);

    // Spatial latent: packed as msgpack bin (4 B/elem) not float array (5 B/elem).
    // Server decodes with: np.frombuffer(data, dtype='<f4')
    pk.pack("spatial_latent");
    if (snap.spatial_latent_snapshot &&
        !snap.spatial_latent_snapshot->latent_vector.empty())
    {
        const auto & lv = snap.spatial_latent_snapshot->latent_vector;
        const uint32_t byte_len = static_cast<uint32_t>(lv.size() * sizeof(float));
        pk.pack_bin(byte_len);
        pk.pack_bin_body(reinterpret_cast<const char*>(lv.data()), byte_len);
    } else {
        pk.pack_nil();
    }

    pk.pack("signed_et");
        pack_signed_et(pk, snap.signed_et_snapshot, snap.signed_et_valid);

    pk.pack("hazard_event");
    if (snap.hazard_event_snapshot) {
        pack_hazard_event(pk, *snap.hazard_event_snapshot);
    } else {
        pk.pack_nil();
    }

    RCLCPP_INFO(get_logger(),
        "SERIALIZING complete: %.1f KB at T+%.2fs",
        buf.size() / 1024.0, elapsed_seconds_since_t0());

    return true;
}

#else  // !VIGIA_HAVE_PAHO_MQTT

// Legacy fallback: simple JSON via the old sign_payload + post_telemetry path.
// Re-uses the existing helper methods inlined below.

bool AntiDeathNode::do_serialize(const SnapshotData & snap)
{
    (void)snap;  // payload built inline in do_mqtt_transmit for the legacy path
    RCLCPP_INFO(get_logger(),
        "SERIALIZING (legacy JSON path) at T+%.2fs", elapsed_seconds_since_t0());
    return true;
}

#endif  // VIGIA_HAVE_PAHO_MQTT

// ─────────────────────────────────────────────────────────────────────────────
// do_mqtt_connect (doc 05 §4.6, budget T+3.0s → T+8.0s)
// ─────────────────────────────────────────────────────────────────────────────

bool AntiDeathNode::do_mqtt_connect()
{
#ifdef VIGIA_HAVE_PAHO_MQTT
    if (!mqtt_client_) {
        RCLCPP_ERROR(get_logger(), "MQTT_CONNECTING: client not initialized.");
        return false;
    }
    if (mqtt_client_->is_connected()) {
        RCLCPP_INFO(get_logger(),
            "MQTT_CONNECTING: already connected. T+%.2fs", elapsed_seconds_since_t0());
        return true;
    }

    RCLCPP_WARN(get_logger(), "MQTT client disconnected — reconnecting...");
    constexpr int kMaxRetries = 3;
    for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
        if (elapsed_seconds_since_t0() > StateBudget::kConnectDeadlineS) {
            RCLCPP_ERROR(get_logger(), "MQTT_CONNECTING: deadline exceeded mid-retry.");
            return false;
        }
        try {
            const double remaining =
                StateBudget::kConnectDeadlineS - elapsed_seconds_since_t0();
            auto tok = mqtt_client_->connect(conn_opts_);
            const bool ok =
                tok->wait_for(std::chrono::duration<double>(remaining / kMaxRetries));
            if (ok && mqtt_client_->is_connected()) {
                RCLCPP_INFO(get_logger(),
                    "MQTT reconnected on attempt %d. T+%.2fs",
                    attempt, elapsed_seconds_since_t0());
                return true;
            }
        } catch (const mqtt::exception & ex) {
            RCLCPP_WARN(get_logger(), "MQTT connect attempt %d: %s", attempt, ex.what());
        }
    }
    RCLCPP_ERROR(get_logger(), "MQTT_CONNECTING: all retries exhausted.");
    return false;

#else
    // No Paho — "connect" always succeeds (legacy path uses curl per-call).
    RCLCPP_INFO(get_logger(), "MQTT_CONNECTING: legacy path (curl).");
    return true;
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// do_mqtt_transmit (doc 05 §4.7, budget T+8.0s → T+13.0s)
// ─────────────────────────────────────────────────────────────────────────────

void AntiDeathNode::do_mqtt_transmit(
#ifdef VIGIA_HAVE_PAHO_MQTT
    const msgpack::sbuffer & buf
#endif
)
{
#ifdef VIGIA_HAVE_PAHO_MQTT
    if (!mqtt_client_) return;

    const std::string topic = mqtt_topic_prefix_ + "/" + device_id_ + "/hazard";

    RCLCPP_INFO(get_logger(),
        "MQTT_TRANSMITTING: %.1f KB → '%s'. T+%.2fs",
        buf.size() / 1024.0, topic.c_str(), elapsed_seconds_since_t0());

    try {
        // Zero-copy reference: Paho reads directly from buf (stack lifetime
        // of execute_emergency_sequence ensures buf outlives the publish).
        auto msg = mqtt::make_message(topic, nullptr, 0, 1, false);
        msg->set_payload_ref(buf.data(), buf.size());

        auto tok = mqtt_client_->publish(msg);

        const double budget_remaining =
            StateBudget::kTransmitDeadlineS - elapsed_seconds_since_t0();
        const bool puback = tok->wait_for(std::chrono::duration<double>(budget_remaining));

        if (puback) {
            RCLCPP_INFO(get_logger(),
                "MQTT PUBACK received. Event attested at T+%.2fs.",
                elapsed_seconds_since_t0());
        } else {
            RCLCPP_WARN(get_logger(),
                "MQTT PUBACK timeout. Payload likely in-flight at power cut. "
                "QoS 1 guarantees at-least-once if broker received packet.");
        }
    } catch (const mqtt::exception & ex) {
        RCLCPP_ERROR(get_logger(), "MQTT publish exception: %s", ex.what());
    }

#else
    // ── Legacy HTTPS POST via libcurl ─────────────────────────────────────────
    const std::string aws_url = get_parameter("aws_telemetry_url").as_string();
    const std::string key_path = get_parameter("device_key_path").as_string();
    if (aws_url.empty() || key_path.empty()) {
        RCLCPP_ERROR(get_logger(), "MQTT_TRANSMITTING: aws_telemetry_url/key not set.");
        return;
    }

    // Build minimal JSON from cached hazard event
    std::string json_body;
    {
        std::ostringstream j;
        j << std::fixed;
        const auto * hz = latest_hazard_event_.get();
        if (!hz) {
            RCLCPP_ERROR(get_logger(), "No hazard event — skipping HTTPS transmit.");
            return;
        }
        // Timestamp
        auto now_t = std::chrono::system_clock::now();
        auto tt = std::chrono::system_clock::to_time_t(now_t);
        std::tm tm_utc{};
        gmtime_r(&tt, &tm_utc);
        char ts[32]; std::strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S.000Z", &tm_utc);

        j << "{\"hazardType\":\"road_hazard\","
          << "\"lat\":" << std::setprecision(7) << hz->gps_pvt.latitude << ","
          << "\"lon\":" << std::setprecision(7) << hz->gps_pvt.longitude << ","
          << "\"timestamp\":\"" << ts << "\","
          << "\"confidence\":" << std::setprecision(4) << hz->rri_score << "}";
        json_body = j.str();
    }

    CURL * curl = curl_easy_init();
    if (!curl) { RCLCPP_ERROR(get_logger(), "curl_easy_init failed"); return; }

    std::string resp;
    curl_slist * hdrs = nullptr;
    hdrs = curl_slist_append(hdrs, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_URL,            aws_url.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS,     json_body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE,  (long)json_body.size());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER,     hdrs);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,  curl_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA,      &resp);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 8L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT,        static_cast<long>(
        StateBudget::kTransmitDeadlineS - elapsed_seconds_since_t0()));
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);
    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    curl_slist_free_all(hdrs);
    curl_easy_cleanup(curl);

    RCLCPP_INFO(get_logger(), "HTTPS_TRANSMIT %s HTTP %ld at T+%.2fs",
        res == CURLE_OK ? "OK" : curl_easy_strerror(res),
        http_code, elapsed_seconds_since_t0());
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// do_safe_shutdown (doc 05 §4.8)
// ─────────────────────────────────────────────────────────────────────────────

void AntiDeathNode::do_safe_shutdown()
{
    RCLCPP_FATAL(get_logger(),
        "SAFE_SHUTDOWN at T+%.2fs (%.1fs remaining in window).",
        elapsed_seconds_since_t0(),
        StateBudget::kPowerWindowS - elapsed_seconds_since_t0());

    // Flush journald ring buffer to ramoops — survives power cut.
    ::sync();

#ifdef VIGIA_HAVE_PAHO_MQTT
    if (mqtt_client_ && mqtt_client_->is_connected()) {
        try {
            mqtt_client_->disconnect()->wait_for(std::chrono::milliseconds(500));
        } catch (...) {}
    }
#endif

    rclcpp::shutdown();

    // Spin wait — power cut arrives imminently.
    RCLCPP_FATAL(get_logger(), "Awaiting power cut. Goodbye.");
    while (true) { __asm__ volatile("yield"); }
}

#ifdef VIGIA_HAVE_SDBUS
#include "ble_gatt_node.hpp"

#include <sdbus-c++/sdbus-c++.h>
#include <chrono>
#include <cstring>
#include <functional>
#include <random>

#ifdef VIGIA_HAVE_MBEDTLS
#  include <mbedtls/ctr_drbg.h>
#  include <mbedtls/entropy.h>
#endif

// ─────────────────────────────────────────────────────────────────────────────
// BlueZ D-Bus constants
// ─────────────────────────────────────────────────────────────────────────────
static constexpr const char* kBluezService     = "org.bluez";
static constexpr const char* kGattMgrIface     = "org.bluez.GattManager1";
static constexpr const char* kLeAdvMgrIface    = "org.bluez.LEAdvertisingManager1";
static constexpr const char* kGattSvcIface     = "org.bluez.GattService1";
static constexpr const char* kGattCharIface    = "org.bluez.GattCharacteristic1";
static constexpr const char* kLeAdvIface       = "org.bluez.LEAdvertisement1";

static constexpr const char* kAppObjPath       = "/com/vigia/ble";
static constexpr const char* kSvcObjPath       = "/com/vigia/ble/service0";
static constexpr const char* kHandshakeObjPath = "/com/vigia/ble/service0/char0";
static constexpr const char* kTelemetryObjPath = "/com/vigia/ble/service0/char1";
static constexpr const char* kControlObjPath   = "/com/vigia/ble/service0/char2";
static constexpr const char* kAttestObjPath    = "/com/vigia/ble/service0/char3";
static constexpr const char* kAdvObjPath       = "/com/vigia/ble/advertisement0";

// ─────────────────────────────────────────────────────────────────────────────
// Handshake wire protocol constants (design spec §4.2a)
// ─────────────────────────────────────────────────────────────────────────────
namespace proto = vigia::ble::proto;
static constexpr uint8_t kHello     = proto::kHello;     // 0x01
static constexpr uint8_t kChallenge = proto::kChallenge; // 0x02
static constexpr uint8_t kResponse  = proto::kResponse;  // 0x03
static constexpr uint8_t kConfirm   = proto::kBound;     // 0x04 — CONFIRM on wire
static constexpr uint8_t kError     = proto::kErr;       // 0xFF

static constexpr size_t kNonceBytes  = 32;
static constexpr size_t kPubBytes    = 65;   // uncompressed P-256: 0x04 || X(32) || Y(32)
// Minimum RESPONSE frame: [0x03][nonce_phone(32)][Phone_pub(65)][sig(at least 1 byte)]
static constexpr size_t kResponseMinLen = 1 + kNonceBytes + kPubBytes + 1;

// HKDF info string (design spec §4.2a)
static const std::string kHkdfInfo = "vigia-ble-v1";
// HMAC CONFIRM label
static const std::string kConfirmLabel = "VIGIA-CONFIRM";

// ─────────────────────────────────────────────────────────────────────────────
// Utility: generate cryptographically random bytes
// ─────────────────────────────────────────────────────────────────────────────
static std::vector<uint8_t> random_bytes(size_t n)
{
#ifdef VIGIA_HAVE_MBEDTLS
    static mbedtls_entropy_context  entropy;
    static mbedtls_ctr_drbg_context ctr_drbg;
    static bool seeded = false;
    if (!seeded) {
        mbedtls_entropy_init(&entropy);
        mbedtls_ctr_drbg_init(&ctr_drbg);
        const char* pers = "vigia_rand";
        mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                               reinterpret_cast<const uint8_t*>(pers), strlen(pers));
        seeded = true;
    }
    std::vector<uint8_t> out(n);
    mbedtls_ctr_drbg_random(&ctr_drbg, out.data(), n);
    return out;
#else
    std::vector<uint8_t> out(n);
    FILE* f = fopen("/dev/urandom", "rb");
    if (f) { fread(out.data(), 1, n, f); fclose(f); }
    return out;
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
BleGattNode::BleGattNode(const rclcpp::NodeOptions& options)
    : Node("ble_gatt_node", options)
{
    ble_adapter_       = declare_parameter<std::string>("ble_adapter",       "hci0");
    device_id_         = declare_parameter<std::string>("device_id",         "vigia-001");
    stream_hz_         = static_cast<float>(declare_parameter<double>("stream_hz", 5.0));
    default_dims_      = declare_parameter<int>("default_dims", 256);
    identity_key_path_ = declare_parameter<std::string>("identity_key_path",
                             vigia::VigiaIdentityKey::kDefaultPath);

#ifdef VIGIA_HAVE_MBEDTLS
    try {
        identity_key_ = std::make_unique<vigia::VigiaIdentityKey>(identity_key_path_);
        RCLCPP_INFO(get_logger(), "Pi identity key loaded from %s", identity_key_path_.c_str());
    } catch (const std::exception& ex) {
        RCLCPP_WARN(get_logger(),
                    "Identity key not found (%s) — ECDH auth disabled; run vigia-pair-qr --provision",
                    ex.what());
    }
#endif

    sub_latent_ = create_subscription<vigia_msgs::msg::SpatialLatent>(
        "/vigia/spatial_latent", vigia::qos::inference_results(),
        std::bind(&BleGattNode::on_latent, this, std::placeholders::_1));

    sub_det_ = create_subscription<vigia_msgs::msg::DetectionArray>(
        "/vigia/detections", vigia::qos::inference_results(),
        std::bind(&BleGattNode::on_detections, this, std::placeholders::_1));

    dbus_thread_ = std::thread(&BleGattNode::dbus_thread_main, this);
    RCLCPP_INFO(get_logger(), "BleGattNode started — adapter=%s stream=%.1f Hz dims=%d",
                ble_adapter_.c_str(), static_cast<double>(stream_hz_), default_dims_);
}

BleGattNode::~BleGattNode()
{
    shutdown_.store(true, std::memory_order_relaxed);
    if (dbus_thread_.joinable()) dbus_thread_.join();
}

// ─────────────────────────────────────────────────────────────────────────────
void BleGattNode::on_latent(vigia_msgs::msg::SpatialLatent::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lk(mailbox_mutex_);
    latest_latent_ = msg->latent_vector;
    mailbox_ready_ = !latest_latent_.empty();
}

void BleGattNode::on_detections(vigia_msgs::msg::DetectionArray::ConstSharedPtr msg)
{
    float rri = 0.0f;
    for (const auto& d : msg->detections) rri = std::max(rri, d.confidence);
    rri = std::min(rri, 1.0f);
    std::lock_guard<std::mutex> lk(mailbox_mutex_);
    latest_rri_ = rri;
}

// ─────────────────────────────────────────────────────────────────────────────
std::vector<uint8_t> BleGattNode::encode_current_frame()
{
    std::lock_guard<std::mutex> lk(mailbox_mutex_);
    if (!mailbox_ready_) return {};
    vigia::ble::DimsCode dims = (default_dims_ == 512)
        ? vigia::ble::DimsCode::k512
        : vigia::ble::DimsCode::k256;
    std::vector<uint8_t> out;
    vigia::ble::encode_frame(latest_rri_, dims,
                              latest_latent_.data(), latest_latent_.size(), out);
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// D-Bus / BlueZ main loop (sdbus-c++ v2 API)
// ─────────────────────────────────────────────────────────────────────────────
void BleGattNode::dbus_thread_main()
{
    try {
        auto conn = sdbus::createSystemBusConnection();
        conn->enterEventLoopAsync();  // dispatch D-Bus events in background thread

        const std::string adapter_path = "/org/bluez/" + ble_adapter_;

        // ── Service object ────────────────────────────────────────────────────
        auto svc_obj = sdbus::createObject(*conn, sdbus::ObjectPath{kSvcObjPath});
        svc_obj->addVTable(
            sdbus::registerProperty("UUID")
                .withGetter([](){ return std::string(vigia::ble::kServiceUuid); }),
            sdbus::registerProperty("Primary")
                .withGetter([](){ return true; })
        ).forInterface(kGattSvcIface);

        // ── TELEMETRY_CHAR ────────────────────────────────────────────────────
        std::vector<uint8_t> telemetry_value;
        auto tel_obj = sdbus::createObject(*conn, sdbus::ObjectPath{kTelemetryObjPath});
        tel_obj->addVTable(
            sdbus::registerProperty("UUID")
                .withGetter([](){ return std::string(vigia::ble::kTelemetryUuid); }),
            sdbus::registerProperty("Service")
                .withGetter([](){ return sdbus::ObjectPath(kSvcObjPath); }),
            sdbus::registerProperty("Flags")
                .withGetter([](){ return std::vector<std::string>{"notify",
                                                                  "encrypt-authenticated-read"}; }),
            sdbus::registerProperty("Value")
                .withGetter([&telemetry_value](){ return telemetry_value; }),
            sdbus::registerMethod("StartNotify").implementedAs([&](){}),
            sdbus::registerMethod("StopNotify").implementedAs([&](){})
        ).forInterface(kGattCharIface);

        // ── HANDSHAKE_CHAR — full Phase 2 ECDH ───────────────────────────────
        std::vector<uint8_t> hs_notify_buf;

        auto hs_obj = sdbus::createObject(*conn, sdbus::ObjectPath{kHandshakeObjPath});
        hs_obj->addVTable(
            sdbus::registerProperty("UUID")
                .withGetter([](){ return std::string(vigia::ble::kHandshakeUuid); }),
            sdbus::registerProperty("Service")
                .withGetter([](){ return sdbus::ObjectPath(kSvcObjPath); }),
            sdbus::registerProperty("Flags")
                .withGetter([](){ return std::vector<std::string>{"read", "write", "notify",
                                                                  "encrypt-authenticated-read",
                                                                  "encrypt-authenticated-write"}; }),
            sdbus::registerProperty("Value")
                .withGetter([&hs_notify_buf](){ return hs_notify_buf; }),
            sdbus::registerMethod("ReadValue")
                .withInputParamNames("options")
                .withOutputParamNames("value")
                .implementedAs([&hs_notify_buf](std::map<std::string, sdbus::Variant>){
                    return hs_notify_buf;
                }),
            sdbus::registerMethod("WriteValue")
                .withInputParamNames("value", "options")
                .implementedAs([this, &hs_notify_buf](
                    std::vector<uint8_t> val,
                    std::map<std::string, sdbus::Variant>)
                {
                    if (val.empty()) return;
                    const uint8_t opcode = val[0];

                    // ── HELLO (0x01) → emit CHALLENGE ─────────────────────────
                    if (opcode == kHello) {
                        hs_state_   = HandshakeState::HelloReceived;
                        session_key_.clear();

#ifndef VIGIA_HAVE_MBEDTLS
                        hs_notify_buf = {kError, 0x00};
                        hs_state_ = HandshakeState::Failed;
                        RCLCPP_WARN(rclcpp::get_logger("ble_gatt_node"),
                                    "HELLO received but mbedTLS not compiled in");
                        return;
#else
                        if (!identity_key_) {
                            hs_notify_buf = {kError, 0x01};
                            hs_state_ = HandshakeState::Failed;
                            RCLCPP_WARN(rclcpp::get_logger("ble_gatt_node"),
                                        "HELLO received but no identity key loaded");
                            return;
                        }

                        hs_nonce_pi_ = random_bytes(kNonceBytes);
                        auto pi_pub  = identity_key_->public_key_uncompressed();

                        std::vector<uint8_t> to_sign;
                        to_sign.insert(to_sign.end(), hs_nonce_pi_.begin(), hs_nonce_pi_.end());
                        to_sign.insert(to_sign.end(), pi_pub.begin(), pi_pub.end());
                        auto sig = identity_key_->sign(to_sign.data(), to_sign.size());

                        hs_notify_buf.clear();
                        hs_notify_buf.push_back(kChallenge);
                        hs_notify_buf.insert(hs_notify_buf.end(),
                                             hs_nonce_pi_.begin(), hs_nonce_pi_.end());
                        hs_notify_buf.insert(hs_notify_buf.end(), pi_pub.begin(), pi_pub.end());
                        hs_notify_buf.insert(hs_notify_buf.end(), sig.begin(), sig.end());

                        RCLCPP_INFO(rclcpp::get_logger("ble_gatt_node"),
                                    "HELLO received — emitting CHALLENGE (%zu bytes)",
                                    hs_notify_buf.size());
#endif
                    }

                    // ── RESPONSE (0x03) → verify + derive session key ─────────
                    else if (opcode == kResponse) {
                        if (hs_state_ != HandshakeState::HelloReceived) {
                            RCLCPP_WARN(rclcpp::get_logger("ble_gatt_node"),
                                        "RESPONSE received in wrong state — ignoring");
                            return;
                        }

#ifndef VIGIA_HAVE_MBEDTLS
                        hs_notify_buf = {kError, 0x00};
                        hs_state_ = HandshakeState::Failed;
                        return;
#else
                        if (val.size() < kResponseMinLen) {
                            RCLCPP_WARN(rclcpp::get_logger("ble_gatt_node"),
                                        "RESPONSE too short: %zu bytes", val.size());
                            hs_notify_buf = {kError, 0x02};
                            hs_state_ = HandshakeState::Failed;
                            return;
                        }

                        const uint8_t* nonce_phone = val.data() + 1;
                        const uint8_t* phone_pub   = val.data() + 1 + kNonceBytes;
                        const uint8_t* phone_sig   = val.data() + 1 + kNonceBytes + kPubBytes;
                        const size_t   phone_sig_len = val.size() - (1 + kNonceBytes + kPubBytes);

                        if (phone_pub[0] != 0x04) {
                            RCLCPP_WARN(rclcpp::get_logger("ble_gatt_node"),
                                        "RESPONSE: phone public key not uncompressed point");
                            hs_notify_buf = {kError, 0x03};
                            hs_state_ = HandshakeState::Failed;
                            return;
                        }

                        std::vector<uint8_t> signed_data;
                        signed_data.insert(signed_data.end(),
                                           nonce_phone, nonce_phone + kNonceBytes);
                        signed_data.insert(signed_data.end(),
                                           hs_nonce_pi_.begin(), hs_nonce_pi_.end());
                        signed_data.insert(signed_data.end(),
                                           phone_pub, phone_pub + kPubBytes);

                        if (!vigia::VigiaIdentityKey::verify_peer(
                                phone_pub,
                                signed_data.data(), signed_data.size(),
                                phone_sig, phone_sig_len))
                        {
                            RCLCPP_WARN(rclcpp::get_logger("ble_gatt_node"),
                                        "RESPONSE: phone ECDSA signature invalid — rejecting");
                            hs_notify_buf = {kError, 0x04};
                            hs_state_ = HandshakeState::Failed;
                            return;
                        }

                        std::vector<uint8_t> shared_secret;
                        try {
                            shared_secret = identity_key_->ecdh_shared_secret(phone_pub);
                        } catch (const std::exception& ex) {
                            RCLCPP_ERROR(rclcpp::get_logger("ble_gatt_node"),
                                         "ECDH failed: %s", ex.what());
                            hs_notify_buf = {kError, 0x05};
                            hs_state_ = HandshakeState::Failed;
                            return;
                        }

                        std::vector<uint8_t> salt;
                        salt.insert(salt.end(), hs_nonce_pi_.begin(), hs_nonce_pi_.end());
                        salt.insert(salt.end(), nonce_phone, nonce_phone + kNonceBytes);

                        session_key_ = vigia::hkdf_sha256(
                            shared_secret.data(), shared_secret.size(),
                            salt.data(), salt.size(),
                            reinterpret_cast<const uint8_t*>(kHkdfInfo.data()),
                            kHkdfInfo.size(), 32);

                        std::vector<uint8_t> hmac_data;
                        hmac_data.insert(hmac_data.end(), kConfirmLabel.begin(), kConfirmLabel.end());
                        hmac_data.insert(hmac_data.end(), hs_nonce_pi_.begin(), hs_nonce_pi_.end());
                        hmac_data.insert(hmac_data.end(), nonce_phone, nonce_phone + kNonceBytes);

                        auto confirm_mac = vigia::hmac_sha256(
                            session_key_.data(), session_key_.size(),
                            hmac_data.data(), hmac_data.size());

                        hs_notify_buf.clear();
                        hs_notify_buf.push_back(kConfirm);
                        hs_notify_buf.insert(hs_notify_buf.end(),
                                             confirm_mac.begin(), confirm_mac.end());

                        hs_state_ = HandshakeState::Bound;
                        RCLCPP_INFO(rclcpp::get_logger("ble_gatt_node"),
                                    "ECDH handshake complete — session established");
#endif
                    }
                })
        ).forInterface(kGattCharIface);

        // ── CONTROL_CHAR — dim selection ──────────────────────────────────────
        auto ctrl_obj = sdbus::createObject(*conn, sdbus::ObjectPath{kControlObjPath});
        ctrl_obj->addVTable(
            sdbus::registerProperty("UUID")
                .withGetter([](){ return std::string(vigia::ble::kControlUuid); }),
            sdbus::registerProperty("Service")
                .withGetter([](){ return sdbus::ObjectPath(kSvcObjPath); }),
            sdbus::registerProperty("Flags")
                .withGetter([](){ return std::vector<std::string>{"write-without-response",
                                                                  "encrypt-authenticated-write"}; }),
            sdbus::registerProperty("Value")
                .withGetter([](){ return std::vector<uint8_t>{}; }),
            sdbus::registerMethod("WriteValue")
                .withInputParamNames("value", "options")
                .implementedAs([this](std::vector<uint8_t> val,
                                      std::map<std::string, sdbus::Variant>) {
                    if (val.empty()) return;
                    if (val[0] == vigia::ble::control::kRequest256) default_dims_ = 256;
                    else if (val[0] == vigia::ble::control::kRequest512) default_dims_ = 512;
                })
        ).forInterface(kGattCharIface);

        // ── ATTEST_CHAR — stub (§6.6, blocked on ATECC608A hardware wiring) ──
        auto att_obj = sdbus::createObject(*conn, sdbus::ObjectPath{kAttestObjPath});
        att_obj->addVTable(
            sdbus::registerProperty("UUID")
                .withGetter([](){ return std::string(vigia::ble::kAttestUuid); }),
            sdbus::registerProperty("Service")
                .withGetter([](){ return sdbus::ObjectPath(kSvcObjPath); }),
            sdbus::registerProperty("Flags")
                .withGetter([](){ return std::vector<std::string>{"notify",
                                                                  "encrypt-authenticated-read"}; }),
            sdbus::registerProperty("Value")
                .withGetter([](){ return std::vector<uint8_t>{}; }),
            sdbus::registerMethod("StartNotify").implementedAs([&](){}),
            sdbus::registerMethod("StopNotify").implementedAs([&](){})
        ).forInterface(kGattCharIface);

        // ── App root ObjectManager (required by BlueZ RegisterApplication) ────
        auto app_obj = sdbus::createObject(*conn, sdbus::ObjectPath{kAppObjPath});

        // sdbus-c++ v2: createProxy takes ServiceName + ObjectPath (no connection arg).
        auto gatt_mgr = sdbus::createProxy(
            sdbus::ServiceName{kBluezService},
            sdbus::ObjectPath{adapter_path});
        gatt_mgr->callMethod("RegisterApplication")
            .onInterface(kGattMgrIface)
            .withArguments(sdbus::ObjectPath(kAppObjPath),
                           std::map<std::string, sdbus::Variant>{})
            .dontExpectReply();

        // ── LE Advertisement ──────────────────────────────────────────────────
        const std::string short_name = "VIGIA-" + device_id_.substr(
            device_id_.size() > 4 ? device_id_.size() - 4 : 0);

        auto adv_obj = sdbus::createObject(*conn, sdbus::ObjectPath{kAdvObjPath});
        adv_obj->addVTable(
            sdbus::registerProperty("Type")
                .withGetter([](){ return std::string("peripheral"); }),
            sdbus::registerProperty("LocalName")
                .withGetter([&short_name](){ return short_name; }),
            sdbus::registerProperty("ServiceUUIDs")
                .withGetter([](){ return std::vector<std::string>{vigia::ble::kServiceUuid}; }),
            sdbus::registerProperty("Discoverable")
                .withGetter([](){ return true; }),
            sdbus::registerProperty("Includes")
                .withGetter([](){ return std::vector<std::string>{"tx-power"}; }),
            sdbus::registerMethod("Release").implementedAs([&](){
                RCLCPP_INFO(rclcpp::get_logger("ble_gatt_node"), "Advertisement released");
            })
        ).forInterface(kLeAdvIface);

        auto adv_mgr = sdbus::createProxy(
            sdbus::ServiceName{kBluezService},
            sdbus::ObjectPath{adapter_path});
        adv_mgr->callMethod("RegisterAdvertisement")
            .onInterface(kLeAdvMgrIface)
            .withArguments(sdbus::ObjectPath(kAdvObjPath),
                           std::map<std::string, sdbus::Variant>{})
            .dontExpectReply();

        RCLCPP_INFO(rclcpp::get_logger("ble_gatt_node"),
                    "BlueZ GATT + advertisement registered — advertising as '%s'",
                    short_name.c_str());

        // ── Event loop ────────────────────────────────────────────────────────
        // sdbus-c++ v2: event loop runs in background via enterEventLoopAsync().
        // This thread just drives telemetry notify and monitors for shutdown.
        const auto interval_ms = std::chrono::milliseconds(
            static_cast<int>(1000.0f / stream_hz_));
        auto last_notify = std::chrono::steady_clock::now();
        std::vector<uint8_t> prev_hs_buf;

        while (!shutdown_.load(std::memory_order_relaxed)) {
            auto now = std::chrono::steady_clock::now();

            // Emit HANDSHAKE_CHAR notification when the buffer changes.
            if (hs_notify_buf != prev_hs_buf && !hs_notify_buf.empty()) {
                prev_hs_buf = hs_notify_buf;
                hs_obj->emitPropertiesChangedSignal(kGattCharIface);
            }

            // Emit TELEMETRY_CHAR notification at stream_hz — only when Bound.
            if (now - last_notify >= interval_ms) {
                last_notify = now;
                const bool bound = (hs_state_ == HandshakeState::Bound);
#ifdef VIGIA_HAVE_MBEDTLS
                const bool auth_ok = bound || !identity_key_;
#else
                const bool auth_ok = true;
#endif
                if (auth_ok) {
                    auto frame = encode_current_frame();
                    if (!frame.empty()) {
                        telemetry_value = frame;
                        tel_obj->emitPropertiesChangedSignal(kGattCharIface);
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        // Cleanup
        adv_mgr->callMethod("UnregisterAdvertisement")
            .onInterface(kLeAdvMgrIface)
            .withArguments(sdbus::ObjectPath(kAdvObjPath))
            .dontExpectReply();
        gatt_mgr->callMethod("UnregisterApplication")
            .onInterface(kGattMgrIface)
            .withArguments(sdbus::ObjectPath(kAppObjPath))
            .dontExpectReply();

    } catch (const sdbus::Error& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ble_gatt_node"),
                     "D-Bus error in BLE thread: %s — %s",
                     e.getName().c_str(), e.getMessage().c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ble_gatt_node"),
                     "BLE thread exception: %s", e.what());
    }
}

#endif // VIGIA_HAVE_SDBUS

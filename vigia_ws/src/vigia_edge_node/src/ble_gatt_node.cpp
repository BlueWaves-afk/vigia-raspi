#ifdef VIGIA_HAVE_SDBUS
#include "ble_gatt_node.hpp"

#include <sdbus-c++/sdbus-c++.h>
#include <chrono>
#include <cstring>
#include <functional>

// ─────────────────────────────────────────────────────────────────────────────
// BlueZ D-Bus object paths / interfaces used by this node.
// See: https://git.kernel.org/pub/scm/bluetooth/bluez.git/tree/doc/gatt-api.txt
// ─────────────────────────────────────────────────────────────────────────────
static constexpr const char* kBluezService     = "org.bluez";
static constexpr const char* kAdapterIface     = "org.bluez.Adapter1";
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
BleGattNode::BleGattNode(const rclcpp::NodeOptions& options)
    : Node("ble_gatt_node", options)
{
    ble_adapter_  = declare_parameter<std::string>("ble_adapter",  "hci0");
    device_id_    = declare_parameter<std::string>("device_id",    "vigia-001");
    stream_hz_    = static_cast<float>(declare_parameter<double>("stream_hz",     5.0));
    default_dims_ = declare_parameter<int>("default_dims", 256);

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
    latest_latent_  = msg->latent_vector;
    mailbox_ready_  = !latest_latent_.empty();
}

void BleGattNode::on_detections(vigia_msgs::msg::DetectionArray::ConstSharedPtr msg)
{
    // Derive a lightweight continuous RRI proxy from detection confidence.
    // Full RRI (with IMU/depth) is computed in FusionNode; this is the
    // "stream RRI" described in §6.3 — max detection confidence, renormalized.
    float rri = 0.0f;
    for (const auto& d : msg->detections)
        rri = std::max(rri, d.confidence);
    rri = std::min(rri, 1.0f);

    std::lock_guard<std::mutex> lk(mailbox_mutex_);
    latest_rri_ = rri;
}

// ─────────────────────────────────────────────────────────────────────────────
std::vector<uint8_t> BleGattNode::encode_current_frame()
{
    std::lock_guard<std::mutex> lk(mailbox_mutex_);
    if (!mailbox_ready_) return {};

    vigia::DimsCode dims = (default_dims_ == 512)
                           ? vigia::DimsCode::k512
                           : vigia::DimsCode::k256;

    return vigia::encode_frame(latest_rri_, dims, latest_latent_);
}

// ─────────────────────────────────────────────────────────────────────────────
// D-Bus / BlueZ main loop.
//
// Phase 1 (skeleton): registers the GATT application and advertisement with
// BlueZ, then fires a notify timer at stream_hz. The TELEMETRY_CHAR value is
// updated on the timer and BlueZ relays it to the subscribed central.
//
// Phase 2 (auth, §4): add HANDSHAKE_CHAR ReadValue/WriteValue handlers and
// the ECDH session-key state machine.
// ─────────────────────────────────────────────────────────────────────────────
void BleGattNode::dbus_thread_main()
{
    try {
        auto conn = sdbus::createSystemBusConnection();

        // ── Discover adapter object path ─────────────────────────────────
        const std::string adapter_path = "/org/bluez/" + ble_adapter_;

        // ── Register GATT Application ────────────────────────────────────
        // BlueZ requires an ObjectManager on the app root. sdbus-c++ v2 uses
        // the createObject API. We register stub GATT objects; BlueZ reads
        // their properties via GetAll / GetManagedObjects.

        // Service object
        auto svc_obj = sdbus::createObject(*conn, kSvcObjPath);
        svc_obj->addVTable(
            sdbus::registerInterface(kGattSvcIface)
                .withProperties(
                    sdbus::defineProperty("UUID").withGetter(
                        [](){ return std::string(vigia::ble::kServiceUuid); }),
                    sdbus::defineProperty("Primary").withGetter(
                        [](){ return true; })
                )
        ).forInterface(kGattSvcIface);

        // TELEMETRY_CHAR — Notify only (no read/write from phone)
        std::vector<uint8_t> telemetry_value;  // updated by timer below
        auto tel_obj = sdbus::createObject(*conn, kTelemetryObjPath);
        tel_obj->addVTable(
            sdbus::registerInterface(kGattCharIface)
                .withProperties(
                    sdbus::defineProperty("UUID").withGetter(
                        [](){ return std::string(vigia::ble::kTelemetryCharUuid); }),
                    sdbus::defineProperty("Service").withGetter(
                        [](){ return sdbus::ObjectPath(kSvcObjPath); }),
                    sdbus::defineProperty("Flags").withGetter(
                        [](){ return std::vector<std::string>{"notify", "encrypt-authenticated-read"}; }),
                    sdbus::defineProperty("Value").withGetter(
                        [&telemetry_value](){ return telemetry_value; })
                )
                .withMethods(
                    sdbus::registerMethod("StartNotify").implementedAs([&](){}),
                    sdbus::registerMethod("StopNotify").implementedAs([&](){})
                )
        ).forInterface(kGattCharIface);

        // HANDSHAKE_CHAR — stub (Phase 2: ECDH handshake handlers go here)
        auto hs_obj = sdbus::createObject(*conn, kHandshakeObjPath);
        hs_obj->addVTable(
            sdbus::registerInterface(kGattCharIface)
                .withProperties(
                    sdbus::defineProperty("UUID").withGetter(
                        [](){ return std::string(vigia::ble::kHandshakeCharUuid); }),
                    sdbus::defineProperty("Service").withGetter(
                        [](){ return sdbus::ObjectPath(kSvcObjPath); }),
                    sdbus::defineProperty("Flags").withGetter(
                        [](){ return std::vector<std::string>{"read", "write", "notify",
                                                              "encrypt-authenticated-read",
                                                              "encrypt-authenticated-write"}; }),
                    sdbus::defineProperty("Value").withGetter(
                        [](){ return std::vector<uint8_t>{}; })
                )
                .withMethods(
                    sdbus::registerMethod("ReadValue")
                        .withInputParamNames("options")
                        .withOutputParamNames("value")
                        .implementedAs([](std::map<std::string,sdbus::Variant>){
                            // Phase 2: return CHALLENGE message
                            return std::vector<uint8_t>{vigia::ble::proto::kHello};
                        }),
                    sdbus::registerMethod("WriteValue")
                        .withInputParamNames("value", "options")
                        .implementedAs([](std::vector<uint8_t>, std::map<std::string,sdbus::Variant>){
                            // Phase 2: process RESPONSE message
                        })
                )
        ).forInterface(kGattCharIface);

        // CONTROL_CHAR — phone→Pi commands (dim selection, pause/resume)
        auto ctrl_obj = sdbus::createObject(*conn, kControlObjPath);
        ctrl_obj->addVTable(
            sdbus::registerInterface(kGattCharIface)
                .withProperties(
                    sdbus::defineProperty("UUID").withGetter(
                        [](){ return std::string(vigia::ble::kControlCharUuid); }),
                    sdbus::defineProperty("Service").withGetter(
                        [](){ return sdbus::ObjectPath(kSvcObjPath); }),
                    sdbus::defineProperty("Flags").withGetter(
                        [](){ return std::vector<std::string>{"write-without-response",
                                                              "encrypt-authenticated-write"}; }),
                    sdbus::defineProperty("Value").withGetter(
                        [](){ return std::vector<uint8_t>{}; })
                )
                .withMethods(
                    sdbus::registerMethod("WriteValue")
                        .withInputParamNames("value", "options")
                        .implementedAs([this](std::vector<uint8_t> val,
                                              std::map<std::string,sdbus::Variant>) {
                            if (val.empty()) return;
                            if (val[0] == vigia::ble::control::kRequest256) default_dims_ = 256;
                            else if (val[0] == vigia::ble::control::kRequest512) default_dims_ = 512;
                        })
                )
        ).forInterface(kGattCharIface);

        // ATTEST_CHAR — Pi→phone anti-spoof beacon (Phase 2, §6.6 stub)
        auto att_obj = sdbus::createObject(*conn, kAttestObjPath);
        att_obj->addVTable(
            sdbus::registerInterface(kGattCharIface)
                .withProperties(
                    sdbus::defineProperty("UUID").withGetter(
                        [](){ return std::string(vigia::ble::kAttestCharUuid); }),
                    sdbus::defineProperty("Service").withGetter(
                        [](){ return sdbus::ObjectPath(kSvcObjPath); }),
                    sdbus::defineProperty("Flags").withGetter(
                        [](){ return std::vector<std::string>{"notify",
                                                              "encrypt-authenticated-read"}; }),
                    sdbus::defineProperty("Value").withGetter(
                        [](){ return std::vector<uint8_t>{}; })
                )
                .withMethods(
                    sdbus::registerMethod("StartNotify").implementedAs([&](){}),
                    sdbus::registerMethod("StopNotify").implementedAs([&](){})
                )
        ).forInterface(kGattCharIface);

        // ObjectManager on app root (required by BlueZ RegisterApplication)
        auto app_obj = sdbus::createObject(*conn, kAppObjPath);
        // BlueZ calls GetManagedObjects to enumerate our GATT hierarchy.
        // sdbus-c++ v2 auto-implements ObjectManager when objects are created
        // under the same connection with paths below the app root.

        // Register GATT application with BlueZ
        auto gatt_mgr = sdbus::createProxy(*conn, kBluezService, adapter_path);
        gatt_mgr->callMethod("RegisterApplication")
            .onInterface(kGattMgrIface)
            .withArguments(sdbus::ObjectPath(kAppObjPath),
                           std::map<std::string,sdbus::Variant>{})
            .dontExpectReply();

        // ── Register LE Advertisement ────────────────────────────────────
        const std::string short_name = "VIGIA-" + device_id_.substr(
            device_id_.size() > 4 ? device_id_.size() - 4 : 0);

        auto adv_obj = sdbus::createObject(*conn, kAdvObjPath);
        adv_obj->addVTable(
            sdbus::registerInterface(kLeAdvIface)
                .withProperties(
                    sdbus::defineProperty("Type").withGetter(
                        [](){ return std::string("peripheral"); }),
                    sdbus::defineProperty("LocalName").withGetter(
                        [&short_name](){ return short_name; }),
                    sdbus::defineProperty("ServiceUUIDs").withGetter(
                        [](){ return std::vector<std::string>{vigia::ble::kServiceUuid}; }),
                    sdbus::defineProperty("Discoverable").withGetter(
                        [](){ return true; }),
                    sdbus::defineProperty("Includes").withGetter(
                        [](){ return std::vector<std::string>{"tx-power"}; })
                )
                .withMethods(
                    sdbus::registerMethod("Release").implementedAs([&](){
                        RCLCPP_INFO(rclcpp::get_logger("ble_gatt_node"), "Advertisement released");
                    })
                )
        ).forInterface(kLeAdvIface);

        auto adv_mgr = sdbus::createProxy(*conn, kBluezService, adapter_path);
        adv_mgr->callMethod("RegisterAdvertisement")
            .onInterface(kLeAdvMgrIface)
            .withArguments(sdbus::ObjectPath(kAdvObjPath),
                           std::map<std::string,sdbus::Variant>{})
            .dontExpectReply();

        RCLCPP_INFO(rclcpp::get_logger("ble_gatt_node"),
                    "BlueZ GATT app + advertisement registered — advertising as '%s'",
                    short_name.c_str());

        // ── Notify timer — stream_hz ─────────────────────────────────────
        const auto interval_ms = std::chrono::milliseconds(
            static_cast<int>(1000.0f / stream_hz_));
        auto last_notify = std::chrono::steady_clock::now();

        // Process D-Bus events + fire notify timer until shutdown.
        while (!shutdown_.load(std::memory_order_relaxed)) {
            conn->processPendingRequest();  // non-blocking D-Bus dispatch

            auto now = std::chrono::steady_clock::now();
            if (now - last_notify >= interval_ms) {
                last_notify = now;
                auto frame = encode_current_frame();
                if (!frame.empty()) {
                    telemetry_value = frame;
                    // Signal BlueZ that the characteristic value changed → notify.
                    tel_obj->emitSignal("PropertiesChanged")
                        .onInterface("org.freedesktop.DBus.Properties")
                        .withArguments(
                            std::string(kGattCharIface),
                            std::map<std::string, sdbus::Variant>{
                                {"Value", sdbus::Variant(telemetry_value)}},
                            std::vector<std::string>{}
                        );
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

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

#include <openvino/openvino.hpp>

#include "coordinator.hpp"
#include "event_store.hpp"
#include "perception.hpp"
#include "analytical.hpp"
#include "temporal.hpp"
#include "fusion.hpp"
#include "sensor_bridge.hpp"

namespace {

std::atomic<bool> g_running{true};

void signalHandler(int)
{
    g_running.store(false);
}

/// Log available CPU optimizations reported by OpenVINO.
/// On Raspberry Pi 4 with OpenVINO 2025, the CPU plugin should report
/// Arm Compute Library (ACL) as its backend for NEON/fp16 acceleration.
void logDeviceCapabilities(ov::Core& core, const std::string& device)
{
    std::cout << "[vigia] OpenVINO device: " << device << '\n';

    try {
        const std::string fullName =
            core.get_property(device, ov::device::full_name);
        std::cout << "[vigia]   Full name  : " << fullName << '\n';
    } catch (...) {}

    try {
        const std::vector<std::string> caps =
            core.get_property(device, ov::device::capabilities);
        std::cout << "[vigia]   Capabilities:";
        for (const auto& cap : caps)
            std::cout << ' ' << cap;
        std::cout << '\n';
    } catch (...) {}

    // Check for Arm Compute Library (ACL) backend
#if defined(__aarch64__) || defined(__ARM_NEON)
    try {
        const std::string fullName =
            core.get_property(device, ov::device::full_name);
        // OpenVINO 2025 CPU plugin on ARM reports ACL in the full_name string
        if (fullName.find("ACL") != std::string::npos ||
            fullName.find("arm_compute") != std::string::npos ||
            fullName.find("Arm") != std::string::npos) {
            std::cout << "[vigia]   ACL backend : DETECTED (NEON-accelerated inference)\n";
        } else {
            std::cerr << "[vigia]   WARNING: ACL backend NOT detected. "
                         "CPU inference may not be NEON-optimized.\n"
                         "[vigia]   Ensure OpenVINO was built with "
                         "-DENABLE_ARM_COMPUTE_CMAKE=ON\n";
        }
    } catch (...) {
        std::cerr << "[vigia]   WARNING: Could not query device properties "
                     "for ACL detection.\n";
    }
#endif
}

const char* envOrDefault(const char* key, const char* fallback)
{
    const char* val = std::getenv(key);
    return (val && val[0] != '\0') ? val : fallback;
}

bool envBool(const char* key, bool defaultVal)
{
    const char* val = std::getenv(key);
    if (!val || val[0] == '\0')
        return defaultVal;
    return std::strcmp(val, "0") != 0 &&
           std::strcmp(val, "false") != 0 &&
           std::strcmp(val, "FALSE") != 0;
}

vigia::EventStore::Config loadEventStoreConfig()
{
    vigia::EventStore::Config cfg{};

    std::strncpy(cfg.promoter.device_id,
                 envOrDefault("VIGIA_DEVICE_ID", "vigia-dev-001"),
                 sizeof(cfg.promoter.device_id) - 1);
    cfg.promoter.device_id[sizeof(cfg.promoter.device_id) - 1] = '\0';

    cfg.promoter.rri_threshold   = 0.75f;
    cfg.promoter.dedup_radius_m  = 5.0f;
    cfg.promoter.dedup_window_s  = 30.0f;
    cfg.promoter.require_gps     = envBool("VIGIA_GPS_REQUIRE_VALID", true);
    cfg.promoter.max_hdop        = 2.5f;
    cfg.promoter.ring_capacity   = 512;

    cfg.signer.hmac_key_file = envOrDefault("VIGIA_HMAC_KEY_FILE", "");
    cfg.sync.endpoint = envOrDefault(
        "VIGIA_SYNC_ENDPOINT", "http://127.0.0.1:8080/v1/events");
#if defined(VIGIA_HAVE_CURL)
    cfg.sync.use_curl = true;
#else
    cfg.sync.use_curl = false;
#endif

    cfg.batch_size          = 50;
    cfg.sync_interval_s     = 5.0f;
    cfg.dev_stdout_fallback = true;

    return cfg;
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    const std::string yoloModel  = (argc > 1)
        ? argv[1]
        : "models/yolo26/yolo26_model.xml";
    const std::string midasModel = (argc > 2)
        ? argv[2]
        : "models/midasv21/openvino_midas_v21_small_256.xml";
    const int targetFps   = (argc > 3) ? std::max(1, std::atoi(argv[3])) : 30;
    const int cameraIndex = (argc > 4) ? std::atoi(argv[4]) : 0;
    const std::string sensorPort = (argc > 5) ? argv[5] : "/dev/ttyACM0";
    const std::string device = "CPU";

    try {
        // Single ov::Core shared across all agents — avoids duplicate plugin
        // discovery, device enumeration, and cache initialization (~50-100 ms
        // saved on Raspberry Pi 4).
        ov::Core core;
        logDeviceCapabilities(core, device);

        vigia::PerceptionAgent perception(core, yoloModel, device, cameraIndex);
        vigia::AnalyticalAgent analytical(core, midasModel, device);
        vigia::TemporalAnalyzer temporal;
        vigia::FusionEngine     fusion;

        // M6: sensor bridge — start before Coordinator so the first frames
        // already have IMU/GPS state.  Tolerates Pico not being plugged in;
        // the bridge logs a warning and Coordinator falls back to vision-only.
        vigia::SensorBridge::Config bridgeCfg;
        bridgeCfg.device = sensorPort;
        bridgeCfg.baud = 115200;
        bridgeCfg.pubkey_file = envOrDefault("VIGIA_PUBKEY_FILE", "");
        bridgeCfg.allow_stub_sig = envBool("VIGIA_ALLOW_STUB_SIG", false);
        vigia::SensorBridge bridge(bridgeCfg);
        bridge.start();

        vigia::EventStore eventStore(loadEventStoreConfig());
        eventStore.start();

        vigia::Coordinator coordinator(
            perception, analytical, temporal, fusion, targetFps
        );
        coordinator.setSensorBridge(bridge);
        coordinator.setEventStore(eventStore);

        coordinator.start();

        std::cout << "[vigia] Pipeline running (FPS target: "
                  << targetFps << ", sensor port: " << sensorPort
                  << ", device: " << eventStore.promoter().config().device_id
                  << "). Press Ctrl+C to stop.\n";

        while (g_running.load())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "\n[vigia] Shutting down...\n";
        coordinator.stop();
        eventStore.stop();
        bridge.stop();

    } catch (const std::exception& ex) {
        std::cerr << "[vigia] Fatal: " << ex.what() << '\n';
        return 1;
    }

    return 0;
}
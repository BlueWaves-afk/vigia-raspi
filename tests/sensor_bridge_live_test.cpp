/**
 * Live hardware test — Pico 2 on /dev/ttyACM0 → SensorBridge → SensorState.
 *
 * Build (from repo build/ dir after cmake ..):
 *   make sensor_bridge_live_test -j$(nproc)
 *   ./sensor_bridge_live_test --duration 15
 *
 * Standalone (no OpenVINO):
 *   g++ -std=c++17 tests/sensor_bridge_live_test.cpp \
 *     src/sensor_packet.cpp src/sensor_state.cpp src/sensor_bridge.cpp \
 *     -Iinclude -pthread -O2 -o sensor_bridge_live_test
 */

#include "sensor_bridge.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

using namespace vigia;

namespace {

int failures = 0;

void fail(const char* msg) {
    std::cout << "[FAIL] " << msg << '\n';
    ++failures;
}

void pass(const char* msg) {
    std::cout << "[PASS] " << msg << '\n';
}

void usage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [--port /dev/ttyACM0] [--duration SEC]\n"
        << "\n"
        << "Opens the Pico USB serial port, runs SensorBridge, and prints\n"
        << "SensorState / health counters every second.\n"
        << "\n"
        << "Prerequisites:\n"
        << "  - Pico flashed with vigia_pico_hello (VIGIA_IMU / VIGIA_GPS lines)\n"
        << "  - User in dialout group: sudo usermod -aG dialout $USER\n"
        << "  - No other process using the port (stop pico_*_monitor.py first)\n";
}

} // namespace

int main(int argc, char** argv) {
    std::string port = "/dev/ttyACM0";
    int duration_sec = 10;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            return 0;
        }
        if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--duration" && i + 1 < argc) {
            duration_sec = std::max(1, std::atoi(argv[++i]));
        } else {
            std::cerr << "Unknown argument: " << arg << '\n';
            usage(argv[0]);
            return 1;
        }
    }

    std::cout << "[LIVE] ===== SensorBridge hardware test =====\n";
    std::cout << "[LIVE] port=" << port << " duration=" << duration_sec << "s\n";

    SensorBridge::Config cfg;
    cfg.device = port;
    cfg.baud = 115200;

    SensorBridge bridge(cfg);
    bridge.start();

    if (!bridge.isRunning()) {
        fail("serial port did not open — check wiring, dialout group, and that nothing else holds the port");
        std::cout << "  ls -l " << port << '\n';
        std::cout << "  groups   # should include dialout\n";
        return 1;
    }
    pass("serial port opened");

    const uint64_t imu_count_start = bridge.state().getHealth().imu_count;
    const uint64_t gps_count_start = bridge.state().getHealth().gps_count;
    const uint64_t ping_count_start = bridge.state().getHealth().ping_count;

    uint64_t last_imu_count = imu_count_start;
    double imu_hz_sum = 0.0;
    int imu_hz_samples = 0;

    for (int sec = 1; sec <= duration_sec; ++sec) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        const auto health = bridge.state().getHealth();
        const auto imu = bridge.state().getLatestImu();
        const auto gps = bridge.state().getLatestGps();

        const uint64_t imu_delta = health.imu_count - last_imu_count;
        last_imu_count = health.imu_count;
        imu_hz_sum += static_cast<double>(imu_delta);
        ++imu_hz_samples;

        std::cout << "[LIVE] t=" << sec << "s"
                  << " imu=" << health.imu_count
                  << " (+" << imu_delta << "/s)"
                  << " gps=" << health.gps_count
                  << " ping=" << health.ping_count
                  << " imu_gaps=" << health.imu_seq_gaps
                  << " parse_err=" << health.parse_errors;

        if (imu) {
            std::cout << " | latest_imu seq=" << imu->seq
                      << " valid=" << imu->valid
                      << " qnorm=" << imu->qnorm
                      << " cal=" << static_cast<unsigned>(imu->cal_status);
        } else {
            std::cout << " | latest_imu (none yet)";
        }

        if (gps) {
            std::cout << " | latest_gps valid=" << gps->valid
                      << " sats=" << static_cast<unsigned>(gps->satellites)
                      << " fix=" << static_cast<unsigned>(gps->fix_type);
        }

        std::cout << '\n';
    }

    bridge.stop();

    const auto health = bridge.state().getHealth();
    const auto imu = bridge.state().getLatestImu();
    const auto gps = bridge.state().getLatestGps();

    const uint64_t imu_received = health.imu_count - imu_count_start;
    const uint64_t gps_received = health.gps_count - gps_count_start;
    const uint64_t ping_received = health.ping_count - ping_count_start;
    const double imu_hz_avg =
        imu_hz_samples > 0 ? imu_hz_sum / static_cast<double>(imu_hz_samples) : 0.0;

    std::cout << "\n[LIVE] ----- summary -----\n";
    std::cout << "[LIVE] IMU lines received : " << imu_received
              << " (~" << imu_hz_avg << " Hz avg)\n";
    std::cout << "[LIVE] GPS lines received : " << gps_received << '\n';
    std::cout << "[LIVE] PING lines received: " << ping_received << '\n';
    std::cout << "[LIVE] IMU seq gaps       : " << health.imu_seq_gaps << '\n';
    std::cout << "[LIVE] Parse errors       : " << health.parse_errors << '\n';

    // --- acceptance ---
    if (imu_received >= static_cast<uint64_t>(duration_sec) * 50u) {
        pass("IMU ingest rate >= 50 Hz");
    } else if (ping_received > 0 && imu_received == 0) {
        fail("PING seen but no VIGIA_IMU — BNO085 may still be initializing (check imu_ready in PING)");
    } else if (imu_received == 0 && ping_received == 0 && gps_received == 0) {
        fail("no VIGIA lines received — reflash firmware or check USB");
    } else {
        fail("IMU rate too low (< 50 Hz sustained)");
    }

    if (health.imu_seq_gaps == 0) {
        pass("zero IMU sequence gaps");
    } else {
        fail("IMU sequence gaps detected");
    }

    if (health.parse_errors == 0) {
        pass("zero parse errors");
    } else {
        fail("parse errors on VIGIA_ lines");
    }

    if (imu && imu->valid) {
        pass("getLatestImu() returns valid sample");
        if (imu->qnorm >= 0.98f && imu->qnorm <= 1.02f) {
            pass("quaternion norm in [0.98, 1.02]");
        } else {
            fail("quaternion norm out of range (board moving or IMU calibrating?)");
        }
    } else if (imu_received > 0) {
        fail("IMU lines received but latest sample not valid");
    }

    if (gps_received > 0 && gps && gps->valid) {
        pass("getLatestGps() returns valid fix");
    } else if (gps_received > 0) {
        std::cout << "[WARN] GPS lines received but no valid fix yet (normal indoors)\n";
    }

    // Ring buffer: history lookup on latest timestamp
    if (imu && imu->timestamp_us > 0) {
        const auto aligned = bridge.state().getSampleAtOrBefore(imu->timestamp_us);
        if (aligned && aligned->seq == imu->seq) {
            pass("getSampleAtOrBefore(latest_ts) aligns with latest IMU");
        } else {
            fail("getSampleAtOrBefore alignment check");
        }
    }

    if (failures == 0) {
        std::cout << "\n[LIVE] All checks passed — SensorState ingest is working.\n";
        std::cout << "[LIVE] Safe to proceed to Coordinator / fusion wiring (M6).\n";
        return 0;
    }

    std::cout << "\n[LIVE] " << failures << " check(s) failed — fix before M6.\n";
    return 1;
}

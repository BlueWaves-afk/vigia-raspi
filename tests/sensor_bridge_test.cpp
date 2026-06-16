#include "sensor_packet.hpp"
#include "sensor_state.hpp"
#include "sensor_bridge.hpp"

#include <cmath>
#include <iostream>
#include <string>

using namespace vigia;

namespace {

int failures = 0;

void expectTrue(bool cond, const char* label) {
    if (cond) {
        std::cout << "[PASS] " << label << '\n';
    } else {
        std::cout << "[FAIL] " << label << '\n';
        ++failures;
    }
}

template <typename T>
void expectNear(T actual, T expected, T epsilon, const char* label) {
    const bool ok = std::fabs(static_cast<double>(actual - expected)) <=
                    static_cast<double>(epsilon);
    expectTrue(ok, label);
}

} // namespace

/*
Standalone build (no OpenCV/OpenVINO):

clang++ -std=c++17 \
  tests/sensor_bridge_test.cpp \
  src/sensor_packet.cpp \
  src/sensor_state.cpp \
  src/sensor_bridge.cpp \
  -Iinclude -pthread -O2 \
  -o sensor_bridge_test
*/

int main() {
    std::cout << "[TEST] ===== SensorBridge / SensorState Unit Test =====\n";

    constexpr const char* kImuGolden =
        "VIGIA_IMU seq=42 timestamp_us=717011207 qw=0.998 qx=0.012 qy=-0.003 "
        "qz=0.055 ax=0.01 ay=-0.02 az=0.15 cal=3 valid=1 qnorm=1.0000";

    constexpr const char* kGpsGolden =
        "VIGIA_GPS seq=0 timestamp_us=97389367 lat=37.1234567 lon=-122.1234567 "
        "speed_ms=0.00 fix_type=3 satellites=12 hdop=0.85 valid=1 src=ubx";

    constexpr const char* kPingGolden =
        "VIGIA_PING seq=0 uptime_ms=2500 boot_ms=1200 fw=gps+imu uart_rx=123 "
        "baud=9600 imu_ready=1";

    /* ---- parseImuLine ---- */
    {
        const auto imu = parseImuLine(kImuGolden);
        expectTrue(imu.has_value(), "parseImuLine returns sample");
        if (imu) {
            expectTrue(imu->seq == 42, "IMU seq");
            expectTrue(imu->timestamp_us == 717011207ULL, "IMU timestamp_us");
            expectNear(imu->qw, 0.998f, 0.0001f, "IMU qw");
            expectNear(imu->qx, 0.012f, 0.0001f, "IMU qx");
            expectTrue(imu->cal_status == 3, "IMU cal");
            expectTrue(imu->valid, "IMU valid");
            expectNear(imu->qnorm, 1.0f, 0.0001f, "IMU qnorm");
        }
    }

    /* ---- parseGpsLine ---- */
    {
        const auto gps = parseGpsLine(kGpsGolden);
        expectTrue(gps.has_value(), "parseGpsLine returns fix");
        if (gps) {
            expectTrue(gps->seq == 0, "GPS seq");
            expectTrue(gps->timestamp_us == 97389367ULL, "GPS timestamp_us");
            expectNear(gps->latitude, 37.1234567, 1e-6, "GPS lat");
            expectNear(gps->longitude, -122.1234567, 1e-6, "GPS lon");
            expectTrue(gps->fix_type == 3, "GPS fix_type");
            expectTrue(gps->satellites == 12, "GPS satellites");
            expectTrue(gps->valid, "GPS valid");
            expectTrue(gps->source == "ubx", "GPS src");
        }
    }

    /* ---- parsePingLine ---- */
    {
        const auto ping = parsePingLine(kPingGolden);
        expectTrue(ping.has_value(), "parsePingLine returns ping");
        if (ping) {
            expectTrue(ping->uptime_ms == 2500ULL, "PING uptime_ms");
            expectTrue(ping->boot_ms == 1200ULL, "PING boot_ms");
        }
    }

    /* ---- SensorState ring buffer ---- */
    {
        SensorState state;

        for (uint32_t i = 0; i < 5; ++i) {
            ImuSample s{};
            s.seq = i;
            s.timestamp_us = 1000ULL * (i + 1);
            s.valid = true;
            state.updateImu(s);
        }

        const auto exact = state.getSampleAtOrBefore(3000);
        expectTrue(exact.has_value(), "getSampleAtOrBefore exact hit");
        if (exact)
            expectTrue(exact->timestamp_us == 3000ULL, "exact timestamp");

        const auto between = state.getSampleAtOrBefore(3500);
        expectTrue(between.has_value(), "getSampleAtOrBefore between samples");
        if (between)
            expectTrue(between->timestamp_us == 3000ULL, "floor timestamp");

        const auto before = state.getSampleAtOrBefore(500);
        expectTrue(!before.has_value(), "getSampleAtOrBefore before first");

        const auto latest = state.getLatestImu();
        expectTrue(latest.has_value() && latest->seq == 4, "getLatestImu");
    }

    /* ---- ring buffer capacity ---- */
    {
        SensorState state;
        for (std::size_t i = 0; i < SensorState::kImuHistorySize + 10; ++i) {
            ImuSample s{};
            s.seq = static_cast<uint32_t>(i);
            s.timestamp_us = 1000ULL * (i + 1);
            state.updateImu(s);
        }

        const auto oldest_kept = state.getSampleAtOrBefore(11000);
        expectTrue(oldest_kept.has_value(), "ring buffer retains recent window");
        if (oldest_kept)
            expectTrue(oldest_kept->timestamp_us >= 11000ULL, "oldest sample in window");

        const auto newest = state.getLatestImu();
        expectTrue(newest.has_value() &&
                       newest->seq == SensorState::kImuHistorySize + 9,
                   "ring buffer latest after wrap");
    }

    /* ---- SensorBridge processLine (no serial) ---- */
    {
        SensorBridge bridge;
        bridge.processLine(kImuGolden);
        bridge.processLine(kGpsGolden);
        bridge.processLine(kPingGolden);

        const auto imu = bridge.state().getLatestImu();
        const auto gps = bridge.state().getLatestGps();
        const auto health = bridge.state().getHealth();

        expectTrue(imu.has_value() && imu->seq == 42, "bridge IMU update");
        expectTrue(gps.has_value() && gps->valid, "bridge GPS update");
        expectTrue(health.imu_count == 1, "bridge imu_count");
        expectTrue(health.gps_count == 1, "bridge gps_count");
        expectTrue(health.ping_count == 1, "bridge ping_count");
        expectTrue(health.last_ping_uptime_ms == 2500ULL, "bridge ping uptime");
    }

    /* ---- seq gap tracking ---- */
    {
        SensorBridge bridge;

        ImuSample first{};
        first.seq = 10;
        first.valid = true;
        bridge.processLine(
            "VIGIA_IMU seq=10 timestamp_us=100 qw=1 qx=0 qy=0 qz=0 "
            "ax=0 ay=0 az=0 cal=3 valid=1 qnorm=1.0000");

        bridge.processLine(
            "VIGIA_IMU seq=13 timestamp_us=200 qw=1 qx=0 qy=0 qz=0 "
            "ax=0 ay=0 az=0 cal=3 valid=1 qnorm=1.0000");

        const auto health = bridge.state().getHealth();
        expectTrue(health.imu_seq_gaps == 2, "IMU seq gap count");
    }

    if (failures == 0) {
        std::cout << "[TEST] All sensor bridge tests passed\n";
        return 0;
    }

    std::cout << "[TEST] " << failures << " failure(s)\n";
    return 1;
}

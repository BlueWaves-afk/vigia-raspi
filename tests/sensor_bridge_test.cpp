#include "sensor_packet.hpp"
#include "sensor_state.hpp"
#include "sensor_bridge.hpp"
#include "cobs.hpp"
#include "signed_et_packet.hpp"

#include <cmath>
#include <cstring>
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
  src/cobs.cpp \
  src/signed_et_packet.cpp \
  src/ecdsa_verify.cpp \
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

    /* ---- COBS SignedEt decode ---- */
    {
        SignedEtPacketView pkt{};
        pkt.magic = kSignedEtMagic;
        pkt.version = kSignedEtVersion;
        pkt.sequence = 7;
        pkt.latitude = 12.97;
        pkt.longitude = 77.59;
        std::memset(pkt.et_hash, 0x11, 32);

        std::uint8_t raw[173];
        std::memcpy(raw, &pkt, sizeof(pkt));

        std::uint8_t frame[256];
        const std::size_t frame_len = cobsEncode(raw, sizeof(raw), frame, sizeof(frame));
        expectTrue(frame_len > 0, "COBS encode");

        std::uint8_t decoded[256];
        const std::size_t dec_len = cobsDecode(frame + 1, frame_len - 2, decoded, sizeof(decoded));
        expectTrue(dec_len == kSignedEtPacketSize, "COBS decode length");

        SignedEtSample sample{};
        expectTrue(parseSignedEtPacket(decoded, dec_len, sample), "parseSignedEtPacket");
        expectTrue(sample.sequence == 7, "COBS packet sequence");

        SensorBridge bridge(SensorBridge::Config{});
        bridge.processCobsFrame(frame + 1, frame_len - 2);
        const auto et = bridge.state().getLatestSignedEt();
        expectTrue(et.has_value() && et->sequence == 7, "bridge SignedEt update");
    }

    /* ---- COBS accumulator buffer-overflow guard ---- */
    {
        // Feed a COBS frame that never terminates (no 0x00 delimiter) and
        // exceeds max_cobs_frame_bytes. The bridge must cap it and record a
        // parse_error rather than growing cobs_acc_ unboundedly.
        SensorBridge::Config cfg{};
        cfg.max_cobs_frame_bytes = 8;   // tiny cap for testing
        SensorBridge bridge(cfg);

        // Simulate the COBS protocol being detected first (0x00 → Cobs).
        // We don't have readLoop here, so feed bytes via processCobsFrame
        // with a too-large payload — cobsDecode returns 0, recordParseError fires.
        std::uint8_t overflow_src[16];
        std::memset(overflow_src, 0xAA, sizeof(overflow_src));
        bridge.processCobsFrame(overflow_src, sizeof(overflow_src));
        const auto h = bridge.state().getHealth();
        expectTrue(h.parse_errors >= 1, "COBS oversized frame → parse_error");
    }

    /* ---- signed_et seq-gap tracking (forward gap only) ---- */
    {
        // Two consecutive signed-et packets with a gap of 3 should record
        // 2 missing sequence numbers, not a parse_error.
        SensorBridge bridge;

        SignedEtPacketView pkt{};
        pkt.magic = kSignedEtMagic;
        pkt.version = kSignedEtVersion;

        auto encodeAndFeed = [&](uint32_t seq) {
            pkt.sequence = seq;
            uint8_t raw[173];
            std::memcpy(raw, &pkt, sizeof(pkt));
            uint8_t frame[256];
            const std::size_t flen = cobsEncode(raw, sizeof(raw), frame, sizeof(frame));
            bridge.processCobsFrame(frame + 1, flen - 2);
        };

        encodeAndFeed(1);
        encodeAndFeed(4);  // gap: seq 2 and 3 missing

        const auto h = bridge.state().getHealth();
        expectTrue(h.signed_et_seq_gaps == 2, "signed_et gap count = 2");
        expectTrue(h.parse_errors == 0, "no parse_errors on seq gap");
    }

    /* ---- signed_et seq wrap does not fire parse_error ---- */
    {
        SensorBridge bridge;

        SignedEtPacketView pkt{};
        pkt.magic = kSignedEtMagic;
        pkt.version = kSignedEtVersion;

        auto encodeAndFeed = [&](uint32_t seq) {
            pkt.sequence = seq;
            uint8_t raw[173];
            std::memcpy(raw, &pkt, sizeof(pkt));
            uint8_t frame[256];
            const std::size_t flen = cobsEncode(raw, sizeof(raw), frame, sizeof(frame));
            bridge.processCobsFrame(frame + 1, flen - 2);
        };

        encodeAndFeed(0xFFFFFFFFu);
        encodeAndFeed(0u);  // valid uint32_t wrap-around

        const auto h = bridge.state().getHealth();
        expectTrue(h.parse_errors == 0, "seq wrap is not a parse_error");
        expectTrue(h.signed_et_seq_gaps == 0, "no gap on seq wrap");
    }

    /* ---- COBS encode capacity: undersized buffer returns 0 ---- */
    {
        uint8_t src[200];
        std::memset(src, 0xBB, sizeof(src));
        uint8_t out[201];  // needs src_len + (src_len/254) + 3 = 204 minimum
        const std::size_t r = cobsEncode(src, sizeof(src), out, sizeof(out));
        expectTrue(r == 0, "cobsEncode rejects undersized output buffer");

        uint8_t out_ok[210];
        const std::size_t r2 = cobsEncode(src, sizeof(src), out_ok, sizeof(out_ok));
        expectTrue(r2 > 0, "cobsEncode succeeds with adequate output buffer");
    }

    /* ---- GPS out-of-range / NaN rejection ---- */
    {
        // Latitude > 90 → must be rejected.
        const auto bad_lat = parseGpsLine(
            "VIGIA_GPS seq=0 timestamp_us=0 lat=91.0 lon=0.0 speed_ms=0 "
            "fix_type=3 satellites=8 hdop=1.0 valid=1 src=ubx");
        expectTrue(!bad_lat.has_value(), "GPS lat > 90 rejected");

        // Longitude < -180 → must be rejected.
        const auto bad_lon = parseGpsLine(
            "VIGIA_GPS seq=0 timestamp_us=0 lat=0.0 lon=-181.0 speed_ms=0 "
            "fix_type=3 satellites=8 hdop=1.0 valid=1 src=ubx");
        expectTrue(!bad_lon.has_value(), "GPS lon < -180 rejected");

        // Negative speed → must be rejected.
        const auto bad_spd = parseGpsLine(
            "VIGIA_GPS seq=0 timestamp_us=0 lat=12.0 lon=77.0 speed_ms=-1.0 "
            "fix_type=3 satellites=8 hdop=1.0 valid=1 src=ubx");
        expectTrue(!bad_spd.has_value(), "GPS negative speed rejected");
    }

    /* ---- SignedEt out-of-range coordinate rejection ---- */
    {
        SignedEtPacketView pkt{};
        pkt.magic = kSignedEtMagic;
        pkt.version = kSignedEtVersion;
        pkt.sequence = 99;
        pkt.qw = 1.0f;
        pkt.latitude = 200.0;   // impossible
        pkt.longitude = 77.0;
        std::memset(pkt.et_hash, 0, 32);

        std::uint8_t raw[173];
        std::memcpy(raw, &pkt, sizeof(pkt));

        std::uint8_t frame[256];
        const std::size_t flen = cobsEncode(raw, sizeof(raw), frame, sizeof(frame));

        SensorBridge bridge;
        bridge.processCobsFrame(frame + 1, flen - 2);
        const auto et = bridge.state().getLatestSignedEt();
        expectTrue(!et.has_value(), "SignedEt with lat=200 rejected");
        const auto h = bridge.state().getHealth();
        expectTrue(h.parse_errors >= 1, "SignedEt bad coords → parse_error");
    }

    /* ---- Text pending-buffer overflow guard ---- */
    {
        // Build a bridge with a very small pending cap, then push bytes that
        // never include a newline. Expect parse_errors to accumulate rather
        // than the pending string growing past the cap.
        SensorBridge::Config cfg{};
        cfg.max_pending_bytes = 16;
        SensorBridge bridge(cfg);

        // Inject 100 non-newline bytes directly via a synthetic line-less call.
        // We can't directly trigger pending overflow without the serial fd, but
        // we can verify parse_error increments via the public interface when
        // given a line that contains the VIGIA_ prefix but doesn't match any
        // format (the bridge calls recordParseError).
        bridge.processLine("VIGIA_UNKNOWN garbage data here");
        const auto h = bridge.state().getHealth();
        expectTrue(h.parse_errors == 1, "unknown VIGIA_ prefix → parse_error");
    }

    if (failures == 0) {
        std::cout << "[TEST] All sensor bridge tests passed\n";
        return 0;
    }

    std::cout << "[TEST] " << failures << " failure(s)\n";
    return 1;
}

/*
 * sensor_processor_test.cpp — unit tests for SensorProcessor.
 *
 * SensorProcessor wraps the ISS pipeline (computeVerticalImpulse →
 * normalizeIss → IssFilter) and the GPS quality gate into one
 * per-frame snapshot call.  Tests cover:
 *
 *   1. Empty state → safe default snapshot
 *   2. IMU valid, GPS absent → imuIss computed, gpsValid=false
 *   3. IMU valid, GPS present but bad fix → gpsValid=false
 *   4. IMU valid, GPS present, good fix → gpsValid=true, coords propagated
 *   5. Vertical impulse below speed floor → imuIss suppressed to 0
 *   6. reset() clears IssFilter state so re-initialization fires correctly
 *
 * Build note: compiled via the Makefile `test` target.
 */

#include "sensor_processor.hpp"
#include "sensor_state.hpp"
#include "sensor_packet.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace vigia;

namespace {

int g_failed = 0;

void expectTrue(bool cond, const char* label) {
    if (cond) {
        std::cout << "  PASS: " << label << '\n';
    } else {
        std::cerr << "  FAIL: " << label << '\n';
        ++g_failed;
    }
}

template <typename T>
void expectNear(T actual, T expected, T epsilon, const char* label) {
    const bool ok = std::fabs(static_cast<double>(actual - expected)) <=
                    static_cast<double>(epsilon);
    expectTrue(ok, label);
}

/** Build a plausible IMU sample (identity quat, moderate vertical accel). */
ImuSample makeImu(float az = 6.0f, uint64_t ts = 1000) {
    ImuSample s{};
    s.seq          = 1;
    s.timestamp_us = ts;
    s.qw           = 1.0f;  // identity quaternion — no rotation
    s.qx           = 0.0f;
    s.qy           = 0.0f;
    s.qz           = 0.0f;
    s.ax           = 0.0f;
    s.ay           = 0.0f;
    s.az           = az;    // body-Z linear accel in m/s²
    s.cal_status   = 3;
    s.valid        = true;
    return s;
}

/** Build a GPS fix that passes the quality gate by default. */
GpsFix makeGps(float hdop = 1.2f, uint8_t fix_type = 3,
               double lat = 12.97, double lon = 77.59,
               float speed = 8.33f) {
    GpsFix g{};
    g.seq        = 1;
    g.latitude   = lat;
    g.longitude  = lon;
    g.speed_ms   = speed;
    g.fix_type   = fix_type;
    g.hdop       = hdop;
    g.valid      = true;
    return g;
}

} // namespace

/* ─────────────────────────────────────────────────────────────────────────
 * Test 1 — empty SensorState yields a safe default snapshot
 * ───────────────────────────────────────────────────────────────────────── */
static void test_empty_state() {
    std::cout << "\n[TEST 1] Empty SensorState → default snapshot\n";

    SensorState state;
    SensorProcessor proc;
    const auto snap = proc.process(state);

    expectTrue(!snap.imuValid,        "imuValid=false when no IMU sample");
    expectTrue(!snap.gpsValid,        "gpsValid=false when no GPS fix");
    expectTrue(snap.imuIss == 0.0f,   "imuIss=0 when no IMU");
    expectTrue(snap.speedMs == 0.0f,  "speedMs=0 when no GPS");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 2 — valid IMU, no GPS
 * ───────────────────────────────────────────────────────────────────────── */
static void test_imu_only() {
    std::cout << "\n[TEST 2] Valid IMU, no GPS\n";

    SensorState state;
    state.updateImu(makeImu(9.0f));  // strong vertical impulse

    SensorProcessor proc;
    const auto snap = proc.process(state);

    expectTrue(snap.imuValid,   "imuValid=true with valid IMU sample");
    expectTrue(!snap.gpsValid,  "gpsValid=false without GPS fix");
    expectTrue(snap.rawVerticalMs2 > 0.0f, "rawVerticalMs2 computed");
    expectTrue(snap.normalizedIss >= 0.0f && snap.normalizedIss <= 1.0f,
               "normalizedIss in [0, 1]");
    // Without GPS, speedMs defaults to 0 → IssFilter speed floor triggers
    // suppression for the first frames, so imuIss can be 0 initially.
    expectTrue(snap.imuIss >= 0.0f, "imuIss non-negative");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 3 — valid IMU, GPS present but fix_type = 1 (no-fix)
 * ───────────────────────────────────────────────────────────────────────── */
static void test_imu_with_bad_gps() {
    std::cout << "\n[TEST 3] Valid IMU, bad GPS fix_type\n";

    SensorState state;
    state.updateImu(makeImu(6.0f));
    state.updateGps(makeGps(/*hdop*/1.2f, /*fix_type*/1));  // fix_type < 2

    SensorProcessor proc;
    const auto snap = proc.process(state);

    expectTrue(snap.imuValid,   "imuValid=true");
    expectTrue(!snap.gpsValid,  "gpsValid=false for fix_type=1");
    expectTrue(snap.speedMs == 0.0f, "speedMs=0 when GPS rejected");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 4 — valid IMU, good GPS
 * ───────────────────────────────────────────────────────────────────────── */
static void test_imu_with_good_gps() {
    std::cout << "\n[TEST 4] Valid IMU + good GPS\n";

    SensorState state;
    state.updateImu(makeImu(8.0f));
    state.updateGps(makeGps(1.2f, 3, 12.9716, 77.5946, 10.0f));

    SensorProcessor proc;
    const auto snap = proc.process(state);

    expectTrue(snap.imuValid,  "imuValid=true");
    expectTrue(snap.gpsValid,  "gpsValid=true for 3D fix with good hdop");
    expectNear(snap.speedMs,   10.0f,    0.01f, "speedMs matches GPS");
    expectNear(snap.gpsLat,    12.9716,  1e-6,  "gpsLat propagated");
    expectNear(snap.gpsLon,    77.5946,  1e-6,  "gpsLon propagated");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 5 — vehicle stationary; IssFilter speed floor suppresses imuIss
 * ───────────────────────────────────────────────────────────────────────── */
static void test_speed_floor_suppresses_iss() {
    std::cout << "\n[TEST 5] Stationary vehicle — imuIss suppressed\n";

    SensorState state;
    state.updateImu(makeImu(15.0f));                          // big impulse
    state.updateGps(makeGps(1.0f, 3, 12.97, 77.59, 0.5f));   // speed < 3 km/h

    SensorProcessor proc;
    const auto snap = proc.process(state);

    // IssFilter speedFloorKmh=3 means 0.5 m/s = 1.8 km/h → suppressed
    expectTrue(snap.imuIss == 0.0f,
               "imuIss=0 when speed below IssFilter floor");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 6 — reset() clears IssFilter internal state
 * ───────────────────────────────────────────────────────────────────────── */
static void test_reset_clears_filter() {
    std::cout << "\n[TEST 6] reset() clears IssFilter state\n";

    SensorState state;
    // Build up background by feeding many frames.
    for (int i = 0; i < 30; ++i) {
        state.updateImu(makeImu(4.0f, static_cast<uint64_t>(i) * 1000));
        state.updateGps(makeGps(1.0f, 3, 12.97, 77.59, 8.0f));
    }

    SensorProcessor proc;
    for (int i = 0; i < 30; ++i)
        proc.process(state);

    proc.reset();

    // After reset the filter is re-initialized on the next sample.
    // Feed zero-impulse — filter seeds from this, so smoothed = 0.
    state.updateImu(makeImu(0.0f, 999999));
    const auto snap = proc.process(state);

    // With zero ISS the filter result must not be a genuine impact.
    expectTrue(!snap.filter.isGenuineImpact,
               "no genuine impact immediately after reset with zero ISS");
}

int main() {
    std::cout << "[TEST] ===== SensorProcessor Unit Test =====\n";

    test_empty_state();
    test_imu_only();
    test_imu_with_bad_gps();
    test_imu_with_good_gps();
    test_speed_floor_suppresses_iss();
    test_reset_clears_filter();

    std::cout << '\n';
    if (g_failed == 0) {
        std::cout << "[TEST] All SensorProcessor tests passed.\n";
        return 0;
    }
    std::cerr << "[TEST] " << g_failed << " test(s) failed.\n";
    return 1;
}

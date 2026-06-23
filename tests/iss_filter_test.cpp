/*
 * iss_filter_test.cpp — Unit tests for IssFilter
 *
 * Test matrix maps to real Indian road scenarios.  All ISS values are
 * normalized [0, 1] as produced by normalizeIss() in iss_compute.cpp.
 *
 * Reference calibration (ISS_MAX = 3.0, V_MIN = 2.0 m/s):
 *
 *   Scenario                    approx awz     speed      ISS_norm
 *   ─────────────────────────── ────────────── ────────── ────────
 *   Engine idle vibration       0.3–0.8 m/s²   0 km/h     0.05–0.13  (gated)
 *   Rough road texture          0.5–2.5 m/s²   40 km/h    0.01–0.09
 *   Small undulation / joints   2–4   m/s²     40 km/h    0.06–0.12
 *   Speed breaker @ 20 km/h     8–15  m/s²     20 km/h    0.22–0.42  (want: PASS)
 *   Pothole @ 30 km/h           5–20  m/s²     30 km/h    0.06–0.24  (want: PASS)
 *   Chassis micro-jolt (1 frame) 10   m/s²     40 km/h    0.25       (want: REJECT)
 *
 * Build note: compiled via the CMakeLists test glob — no manual compilation needed.
 */

#include "iss_filter.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

static int g_failed = 0;

#define EXPECT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "  FAIL: " << (msg) << "\n"; \
            ++g_failed; \
        } else { \
            std::cout << "  PASS: " << (msg) << "\n"; \
        } \
    } while(0)

// Feed N identical frames into the filter and return the last result.
static vigia::IssFilter::Result feedSteady(
    vigia::IssFilter& f,
    float iss,
    float speedMs,
    int frames
) {
    vigia::IssFilter::Result r{};
    for (int i = 0; i < frames; ++i)
        r = f.update(iss, speedMs);
    return r;
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Test 1 — Speed floor gate
 * Vehicle is stationary or very slow (loading dock, garage, traffic jam stop).
 * ISS must be suppressed entirely regardless of IMU value.
 * ══════════════════════════════════════════════════════════════════════════════ */
static void test_speed_floor_gate()
{
    std::cout << "\n[TEST 1] Speed floor gate (parked / <3 km/h)\n";
    vigia::IssFilter f;

    // Simulate door-slam vibration at standstill: ISS_norm = 0.4 (very high)
    auto r = feedSteady(f, 0.4f, 0.3f, 10);   // 0.3 m/s = 1.1 km/h < 3 km/h floor
    EXPECT(!r.isGenuineImpact,  "no impact flag when parked");
    EXPECT(r.smoothedIss == 0.0f, "smoothed stays 0 below speed floor");
    EXPECT(r.detrendedIss == 0.0f, "detrended 0 below speed floor");
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Test 2 — Rough road texture (continuous vibration, Indian state roads)
 * ISS_norm ≈ 0.04–0.08 sustained for many frames.
 * The background tracker should absorb this; detrended should stay near 0.
 * ══════════════════════════════════════════════════════════════════════════════ */
static void test_rough_road_no_false_positive()
{
    std::cout << "\n[TEST 2] Rough road texture — no false positive\n";
    vigia::IssFilter f;

    const float speedMs = 11.1f;  // 40 km/h
    const float iss     = 0.06f;  // typical rough-surface ISS_norm

    // Run 300 frames (~20 s at 15 FPS) to let background settle
    feedSteady(f, iss, speedMs, 300);
    auto r = feedSteady(f, iss, speedMs, 10);

    EXPECT(!r.isGenuineImpact,    "rough road: no genuine impact after settling");
    EXPECT(r.backgroundIss > 0.03f, "background rose to track road noise");
    EXPECT(r.detrendedIss < 0.05f,  "detrended stays low on steady rough road");
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Test 3 — Single-frame chassis jolt (NOT a pothole)
 * A one-frame spike from a stone chip, manhole lip at speed, etc.
 * The consecutive-frame gate must reject it.
 * ══════════════════════════════════════════════════════════════════════════════ */
static void test_single_frame_spike_rejected()
{
    std::cout << "\n[TEST 3] Single-frame chassis jolt — must be rejected\n";
    vigia::IssFilter f;

    const float speedMs = 11.1f;

    // Let background settle on smooth road first
    feedSteady(f, 0.02f, speedMs, 200);

    // One very sharp spike
    auto r = f.update(0.80f, speedMs);
    EXPECT(!r.isGenuineImpact, "single spike does not pass consecutive-frame gate");
    EXPECT(r.detrendedIss > 0.0f, "detrended is non-zero (spike was real)");
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Test 4 — Genuine pothole @ 30 km/h
 * awz ≈ 10 m/s² sustained for ~200 ms (3 frames at 15 FPS).
 * ISS_norm ≈ 0.37 for 3 consecutive frames → must be flagged.
 * ══════════════════════════════════════════════════════════════════════════════ */
static void test_pothole_detected()
{
    std::cout << "\n[TEST 4] Genuine pothole @ 30 km/h — must fire\n";
    vigia::IssFilter f;

    const float speedMs = 8.33f;  // 30 km/h

    // Smooth road baseline
    feedSteady(f, 0.02f, speedMs, 200);

    // Pothole: ISS_norm ≈ 0.37 for 4 frames (~267 ms)
    vigia::IssFilter::Result r;
    for (int i = 0; i < 4; ++i)
        r = f.update(0.37f, speedMs);

    EXPECT(r.isGenuineImpact, "pothole sustained for 4 frames → genuine impact");
    EXPECT(r.detrendedIss > 0.10f, "detrended ISS meaningfully above background");
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Test 5 — Indian speed breaker (intentional road feature) @ 20 km/h
 * These are narrow and sharp; ISS_norm ≈ 0.30–0.60 for 3–5 frames.
 * The filter correctly reports it as a genuine impact (we let the vision
 * pipeline distinguish pothole vs. speed breaker by class ID).
 * ══════════════════════════════════════════════════════════════════════════════ */
static void test_speed_breaker_detected()
{
    std::cout << "\n[TEST 5] Speed breaker @ 20 km/h — must fire\n";
    vigia::IssFilter f;

    const float speedMs = 5.56f;  // 20 km/h

    feedSteady(f, 0.03f, speedMs, 200);

    vigia::IssFilter::Result r;
    for (int i = 0; i < 5; ++i)    // ~333 ms at 15 FPS
        r = f.update(0.50f, speedMs);

    EXPECT(r.isGenuineImpact, "speed breaker fires genuine impact");
    EXPECT(r.detrendedIss > 0.20f, "high detrended ISS for speed breaker");
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Test 6 — Hill climb without bump (gravity compensation check)
 * The BNO085 linear accel is already gravity-compensated; after quaternion
 * rotation, a clean hill climb produces awz ≈ 0 → ISS ≈ 0.
 * This test confirms the filter correctly passes through near-zero ISS.
 * ══════════════════════════════════════════════════════════════════════════════ */
static void test_hill_climb_no_false_positive()
{
    std::cout << "\n[TEST 6] Hill climb — gravity compensated, no false positive\n";
    vigia::IssFilter f;

    const float speedMs = 8.33f;   // 30 km/h uphill

    // Gravity-compensated ISS during steady climb should be near 0
    feedSteady(f, 0.00f, speedMs, 200);
    auto r = feedSteady(f, 0.01f, speedMs, 30);  // tiny residual from imperfect quat

    EXPECT(!r.isGenuineImpact, "hill climb alone does not trigger impact");
    EXPECT(r.detrendedIss < 0.05f, "detrended near 0 on smooth incline");
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Test 7 — Highway speed, raised adaptive threshold
 * At 100 km/h road noise is higher; spikeThreshold rises by +0.15.
 * A moderate ISS_norm = 0.12 (undulation) that would pass at city speed
 * should be suppressed at highway speed.
 * ══════════════════════════════════════════════════════════════════════════════ */
static void test_highway_adaptive_threshold()
{
    std::cout << "\n[TEST 7] Highway speed — adaptive threshold suppresses undulations\n";
    vigia::IssFilter f;

    const float speedMs = 27.8f;  // 100 km/h

    feedSteady(f, 0.03f, speedMs, 200);

    // Mild undulation ISS_norm = 0.12 sustained
    vigia::IssFilter::Result r;
    for (int i = 0; i < 5; ++i)
        r = f.update(0.12f, speedMs);

    // effectiveThreshold at 100 km/h = 0.10 + 0.0015 × 100 = 0.25
    // detrended ≈ 0.09 (< 0.25) → should NOT fire
    EXPECT(!r.isGenuineImpact, "highway undulation suppressed by raised threshold");
    EXPECT(r.effectiveThreshold > 0.20f, "adaptive threshold is elevated at 100 km/h");
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Test 8 — Reset clears all state
 * After a long run the background and smoothed values are non-zero.
 * reset() must return the filter to its initial condition.
 * ══════════════════════════════════════════════════════════════════════════════ */
static void test_reset()
{
    std::cout << "\n[TEST 8] reset() clears all filter state\n";
    vigia::IssFilter f;

    feedSteady(f, 0.40f, 11.1f, 300);
    f.reset();

    // After reset, a 0-ISS frame on a smooth road should give all zeros
    auto r = f.update(0.0f, 11.1f);
    EXPECT(r.smoothedIss    == 0.0f, "smoothed reset to 0");
    EXPECT(r.backgroundIss  == 0.0f, "background reset to 0");
    EXPECT(!r.isGenuineImpact,       "no impact immediately after reset");
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Test 9 — Background does not track genuine impacts
 * If a real 300-ms pothole hit occurs, the background should not jump —
 * otherwise the next pothole in the same stretch would be suppressed.
 * ══════════════════════════════════════════════════════════════════════════════ */
static void test_background_not_corrupted_by_impact()
{
    std::cout << "\n[TEST 9] Background unaffected by short-duration impact\n";
    vigia::IssFilter f;

    const float speedMs = 8.33f;
    feedSteady(f, 0.02f, speedMs, 300);

    auto before = f.update(0.02f, speedMs).backgroundIss;

    // Simulate a 300-ms pothole (4 frames at 15 FPS)
    for (int i = 0; i < 4; ++i)
        f.update(0.60f, speedMs);

    // Return to normal road — background should barely have moved
    auto after = f.update(0.02f, speedMs).backgroundIss;

    const float drift = std::abs(after - before);
    EXPECT(drift < 0.05f, "background drifts < 0.05 after a single pothole event");
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Test 10 — Vision-only path (bridge not connected)
 * When querySensors returns imuIss = 0 (no bridge), fusion still works.
 * This is implicitly tested by keeping imuIss = 0 (default), but we verify
 * the filter handles zero ISS for many frames without assertion or NaN.
 * ══════════════════════════════════════════════════════════════════════════════ */
static void test_zero_iss_is_stable()
{
    std::cout << "\n[TEST 10] Zero ISS (no sensor bridge) — numerically stable\n";
    vigia::IssFilter f;

    auto r = feedSteady(f, 0.0f, 10.0f, 1000);

    EXPECT(!r.isGenuineImpact,      "no false impact from sustained zero ISS");
    EXPECT(r.smoothedIss    == 0.0f, "smoothed stays 0");
    EXPECT(r.backgroundIss  == 0.0f, "background stays 0");
    EXPECT(r.detrendedIss   == 0.0f, "detrended stays 0");

    // Check for NaN
    EXPECT(r.smoothedIss == r.smoothedIss,   "smoothed is not NaN");
    EXPECT(r.backgroundIss == r.backgroundIss, "background is not NaN");
}

int main()
{
    std::cout << "[TEST] ===== IssFilter Unit Test — Indian Road Benchmark =====\n";

    test_speed_floor_gate();
    test_rough_road_no_false_positive();
    test_single_frame_spike_rejected();
    test_pothole_detected();
    test_speed_breaker_detected();
    test_hill_climb_no_false_positive();
    test_highway_adaptive_threshold();
    test_reset();
    test_background_not_corrupted_by_impact();
    test_zero_iss_is_stable();

    std::cout << '\n';
    if (g_failed == 0) {
        std::cout << "[TEST] ALL PASSED — IssFilter validated for Indian road conditions\n";
        return 0;
    }
    std::cerr << "[TEST] " << g_failed << " ASSERTION(S) FAILED\n";
    return 1;
}

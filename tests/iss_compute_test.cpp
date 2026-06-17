/*
 * iss_compute_test.cpp — quaternion gravity-compensation + ISS normalization
 *
 * Validates computeVerticalImpulse() with synthetic IMU vectors before
 * field testing.  No hardware or OpenVINO required.
 */

#include "iss_compute.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

static int g_failed = 0;

#define EXPECT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "  FAIL: " << (msg) << '\n'; \
            ++g_failed; \
        } else { \
            std::cout << "  PASS: " << (msg) << '\n'; \
        } \
    } while(0)

static vigia::ImuSample makeSample(
    float qw, float qx, float qy, float qz,
    float ax, float ay, float az)
{
    vigia::ImuSample s{};
    s.valid = true;
    s.qw = qw; s.qx = qx; s.qy = qy; s.qz = qz;
    s.ax = ax; s.ay = ay; s.az = az;
    return s;
}

/* Identity quaternion, zero linear accel → flat road at constant speed */
static void test_flat_road_zero_impulse()
{
    std::cout << "\n[TEST] Flat road — zero vertical impulse\n";
    auto s = makeSample(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    const float awz = vigia::computeVerticalImpulse(s);
    EXPECT(awz < 0.05f, "awz ≈ 0 on flat road with zero linear accel");
}

/*
 * Hill climb: vehicle pitched up 15° but no bump.
 * Body Z sees gravity projection if uncompensated; linear accel from BNO085
 * should still be ~0.  We simulate correct linear accel (near zero) with
 * a pitch quaternion.
 */
static void test_hill_climb_no_bump()
{
    std::cout << "\n[TEST] Hill climb — gravity compensated linear accel\n";
    const float pitch = 15.0f * static_cast<float>(M_PI) / 180.0f;
    const float half  = pitch * 0.5f;
    // Pitch about Y: q = [cos(θ/2), 0, sin(θ/2), 0]
    auto s = makeSample(std::cos(half), 0.0f, std::sin(half), 0.0f,
                        0.05f, 0.0f, 0.05f);  // tiny sensor noise
    const float awz = vigia::computeVerticalImpulse(s);
    EXPECT(awz < 0.15f, "hill climb without bump keeps awz low");
}

/* Sharp vertical bump: pure Z impulse in body frame, level vehicle */
static void test_vertical_bump()
{
    std::cout << "\n[TEST] Vertical bump — body Z impulse\n";
    auto s = makeSample(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 12.0f);
    const float awz = vigia::computeVerticalImpulse(s);
    EXPECT(awz > 10.0f, "12 m/s² body-Z bump produces high awz");

    const float norm = vigia::normalizeIss(awz, 8.33f);  // 30 km/h
    EXPECT(norm > 0.3f, "normalized ISS significant at 30 km/h");
}

/* Speed normalization — same impulse, higher speed → lower ISS */
static void test_speed_normalization()
{
    std::cout << "\n[TEST] Speed normalization\n";
    const float awz = 9.0f;
    const float slow = vigia::normalizeIss(awz, 5.56f);   // 20 km/h
    const float fast = vigia::normalizeIss(awz, 16.67f);  // 60 km/h
    EXPECT(slow > fast, "same bump at lower speed yields higher normalized ISS");
}

/* GPS gate */
static void test_gps_gate()
{
    std::cout << "\n[TEST] GPS usability gate\n";
    vigia::GpsFix good{};
    good.valid = true;
    good.fix_type = 3;
    good.hdop = 1.2f;
    EXPECT(vigia::isGpsFixUsable(good), "3D fix with good HDOP passes");

    vigia::GpsFix badHdop = good;
    badHdop.hdop = 4.0f;
    EXPECT(!vigia::isGpsFixUsable(badHdop), "high HDOP rejected");

    vigia::GpsFix noFix = good;
    noFix.fix_type = 1;
    EXPECT(!vigia::isGpsFixUsable(noFix), "fix_type < 2 rejected");
}

int main()
{
    std::cout << "[TEST] ===== IssCompute Unit Test =====\n";
    test_flat_road_zero_impulse();
    test_hill_climb_no_bump();
    test_vertical_bump();
    test_speed_normalization();
    test_gps_gate();

    if (g_failed == 0) {
        std::cout << "\n[TEST] ALL PASSED — IssCompute validated\n";
        return 0;
    }
    std::cerr << "\n[TEST] " << g_failed << " ASSERTION(S) FAILED\n";
    return 1;
}

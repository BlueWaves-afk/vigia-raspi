#include "fusion.hpp"
#include <cassert>
#include <cstdlib>
#include <iomanip>
#include <iostream>

using namespace vigia;

static int g_failed = 0;

static void printResult(const std::string& label, const FusionOutput& out)
{
    std::cout << label << "\n"
              << std::fixed << std::setprecision(4)
              << "  geometry  = " << out.geometryConfidence << "\n"
              << "  temporal  = " << out.temporalConfidence << "\n"
              << "  final     = " << out.finalConfidence    << "\n";
    if (out.gpsValid)
        std::cout << "  gps       = " << out.latitude << ", "
                  << out.longitude << "  spd=" << out.speedMs << " m/s\n";
}

#define EXPECT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "  FAIL: " << (msg) << "\n"; \
            ++g_failed; \
        } else { \
            std::cout << "  PASS: " << (msg) << "\n"; \
        } \
    } while(0)

int main()
{
    std::cout << "[TEST] ===== FusionEngine Unit Test (M6) =====\n\n";

    FusionEngine fusion;

    /* ============================================================
     * Group 1 — Vision-only path (backward compat, imuIss = 0)
     * Verifies that all pre-M6 tests still produce sane outputs.
     * ============================================================ */

    /* Test 1: Ideal pothole */
    {
        FusionInput in{};
        in.yoloConfidence  = 0.9f;
        in.depressionScore = 0.8f;
        in.roughness       = 0.02f;
        in.persistence     = 300.0f;
        in.stability       = 8000.0f;

        auto out = fusion.fuse(in);
        printResult("[TEST 1] Ideal pothole (vision-only)", out);
        EXPECT(out.finalConfidence > 0.5f, "final > 0.5 for ideal pothole");
        EXPECT(!out.gpsValid,              "no geo-tag without GPS input");
    }

    /* Test 2: Detector only (no geometry) */
    {
        FusionInput in{};
        in.yoloConfidence = 0.95f;

        auto out = fusion.fuse(in);
        printResult("[TEST 2] Detector-only signal", out);
        EXPECT(out.finalConfidence > 0.0f, "final > 0 for YOLO hit");
        EXPECT(out.finalConfidence < 0.5f, "final < 0.5 without geo/temporal");
    }

    /* Test 3: Geometry but noisy surface */
    {
        FusionInput in{};
        in.yoloConfidence  = 0.6f;
        in.depressionScore = 0.7f;
        in.roughness       = 0.5f;
        in.persistence     = 250.0f;
        in.stability       = 50.0f;

        auto out = fusion.fuse(in);
        printResult("[TEST 3] Noisy surface", out);
        EXPECT(out.geometryConfidence < 0.5f, "roughness penalises geometry");
    }

    /* Test 4: Temporal instability */
    {
        FusionInput in{};
        in.yoloConfidence  = 0.7f;
        in.depressionScore = 0.6f;
        in.roughness       = 0.05f;
        in.persistence     = 5.0f;
        in.stability       = 2.0f;

        auto out = fusion.fuse(in);
        printResult("[TEST 4] Temporal instability", out);
        EXPECT(out.temporalConfidence < 0.3f, "low temporal for weak persistence");
    }

    /* Test 5: Everything weak */
    {
        FusionInput in{};
        in.yoloConfidence  = 0.2f;
        in.depressionScore = 0.1f;
        in.roughness       = 0.8f;
        in.persistence     = 1.0f;
        in.stability       = 1.0f;

        auto out = fusion.fuse(in);
        printResult("[TEST 5] Weak signal", out);
        EXPECT(out.finalConfidence < 0.2f, "final < 0.2 for all-weak input");
    }

    /* ============================================================
     * Group 2 — ISS integration tests
     * Use synthetic imuIss values that mimic the coordinator's
     * output after quaternion rotation + normalization.
     * ============================================================ */

    /* Test 6: Flat road at speed — ISS near zero → no false bump */
    {
        FusionInput in{};
        in.yoloConfidence = 0.5f;
        in.imuIss         = 0.02f;   // ~0.06 m/s² awz at 30 km/h ≈ ISS ≈ 0
        in.speedMs        = 8.33f;   // 30 km/h

        auto out = fusion.fuse(in);
        printResult("[TEST 6] Flat road (ISS ≈ 0)", out);
        EXPECT(out.finalConfidence < 0.25f, "flat road keeps final low");
    }

    /* Test 7: Speed bump at 20 km/h — ISS should drive score up */
    {
        FusionInput in{};
        in.yoloConfidence = 0.7f;
        in.depressionScore = 0.5f;
        in.roughness       = 0.1f;
        in.persistence     = 200.0f;
        in.stability       = 3000.0f;
        in.imuIss          = 1.0f;   // fully saturated ISS (≥ 3g / 5.5m/s)
        in.speedMs         = 5.56f;  // 20 km/h

        auto out = fusion.fuse(in);
        printResult("[TEST 7] Speed bump at 20 km/h", out);
        EXPECT(out.finalConfidence > 0.65f, "bump: high ISS + YOLO → final > 0.65");
    }

    /* Test 8: Motion gate — ISS contribution suppressed when parked */
    {
        FusionInput in{};
        in.yoloConfidence = 0.8f;
        in.imuIss         = 1.0f;    // would be strong if moving
        in.speedMs        = 0.3f;    // parked / garage

        auto out = fusion.fuse(in);

        FusionInput inNoIss{};
        inNoIss.yoloConfidence = 0.8f;
        // imuIss = 0, speedMs = 0 → same expected result
        auto outNoIss = fusion.fuse(inNoIss);

        printResult("[TEST 8] Parked (ISS motion-gated)", out);
        EXPECT(std::abs(out.finalConfidence - outNoIss.finalConfidence) < 1e-4f,
               "parked: ISS gated out, identical to no-ISS path");
    }

    /* Test 9: GPS geo-tag attached when valid */
    {
        FusionInput in{};
        in.yoloConfidence = 0.8f;
        in.imuIss         = 0.6f;
        in.speedMs        = 8.0f;
        in.gpsLat         = 12.345678;
        in.gpsLon         = 77.123456;
        in.gpsValid       = true;

        auto out = fusion.fuse(in);
        printResult("[TEST 9] GPS geo-tag attached", out);
        EXPECT(out.gpsValid,                          "gpsValid propagated");
        EXPECT(std::abs(out.latitude  - 12.345678) < 1e-9, "latitude correct");
        EXPECT(std::abs(out.longitude - 77.123456) < 1e-9, "longitude correct");
        EXPECT(std::abs(out.speedMs - 8.0f) < 1e-4f,      "speed correct");
    }

    /* Test 10: GPS suppressed when gpsValid = false */
    {
        FusionInput in{};
        in.yoloConfidence = 0.8f;
        in.gpsLat         = 12.345678;
        in.gpsLon         = 77.123456;
        in.gpsValid       = false;   // caller rejected fix_type < 2 or hdop > 2.5

        auto out = fusion.fuse(in);
        printResult("[TEST 10] GPS not tagged (invalid fix)", out);
        EXPECT(!out.gpsValid,         "no geo-tag when gpsValid=false");
        EXPECT(out.latitude  == 0.0,  "latitude zero when not tagged");
        EXPECT(out.longitude == 0.0,  "longitude zero when not tagged");
    }

    std::cout << "\n";
    if (g_failed == 0) {
        std::cout << "[TEST] ALL PASSED — FusionEngine M6 validated\n";
        return 0;
    }
    std::cerr << "[TEST] " << g_failed << " ASSERTION(S) FAILED\n";
    return 1;
}

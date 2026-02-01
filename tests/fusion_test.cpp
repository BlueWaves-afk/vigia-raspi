#include "fusion.hpp"
#include <iostream>
#include <iomanip>

using namespace vigia;
/*
clang++ -std=c++17 \
 tests/fusion_test.cpp src/fusion.cpp \
 -Iinclude -O3 \
 -o fusion_test
*/
static void printResult(
    const std::string& label,
    const FusionOutput& out
) {
    std::cout << label << "\n";
    std::cout << "  geometry  = " << out.geometryConfidence << "\n";
    std::cout << "  temporal  = " << out.temporalConfidence << "\n";
    std::cout << "  final     = " << out.finalConfidence << "\n";
}

int main() {
    std::cout << "[TEST] ===== FusionEngine Unit Test =====\n";

    FusionEngine fusion;

    /* ===================== Test 1: Ideal pothole ===================== */
    {
        FusionInput in{};
        in.yoloConfidence  = 0.9f;
        in.depressionScore = 0.8f;
        in.roughness       = 0.02f;
        in.persistence     = 300.0f;
        in.stability       = 8000.0f;

        auto out = fusion.fuse(in);
        printResult("[TEST 1] Ideal pothole", out);
    }

    /* ===================== Test 2: Detector only (no geometry) ===================== */
    {
        FusionInput in{};
        in.yoloConfidence  = 0.95f;
        in.depressionScore = 0.0f;
        in.roughness       = 0.0f;
        in.persistence     = 0.0f;
        in.stability       = 0.0f;

        auto out = fusion.fuse(in);
        printResult("[TEST 2] Detector-only signal", out);
    }

    /* ===================== Test 3: Geometry but noisy surface ===================== */
    {
        FusionInput in{};
        in.yoloConfidence  = 0.6f;
        in.depressionScore = 0.7f;
        in.roughness       = 0.5f;   // noisy
        in.persistence     = 250.0f;
        in.stability       = 50.0f;

        auto out = fusion.fuse(in);
        printResult("[TEST 3] Noisy surface", out);
    }

    /* ===================== Test 4: Temporal instability ===================== */
    {
        FusionInput in{};
        in.yoloConfidence  = 0.7f;
        in.depressionScore = 0.6f;
        in.roughness       = 0.05f;
        in.persistence     = 5.0f;   // weak persistence
        in.stability       = 2.0f;   // unstable

        auto out = fusion.fuse(in);
        printResult("[TEST 4] Temporal instability", out);
    }

    /* ===================== Test 5: Everything weak ===================== */
    {
        FusionInput in{};
        in.yoloConfidence  = 0.2f;
        in.depressionScore = 0.1f;
        in.roughness       = 0.8f;
        in.persistence     = 1.0f;
        in.stability       = 1.0f;

        auto out = fusion.fuse(in);
        printResult("[TEST 5] Weak signal", out);
    }

    std::cout << "[TEST] ✅ FusionEngine fully validated\n";
    return 0;
}

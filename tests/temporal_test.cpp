#include "temporal.hpp"

#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

using namespace vigia;

/*
BUILD:
clang++ -std=c++17 \
 tests/temporal_test.cpp src/temporal.cpp \
 -Iinclude -O3 \
 -o temporal_test
*/

static bool nearlyEqual(float a, float b, float eps = 1e-3f) {
    return std::fabs(a - b) < eps;
}

int main() {
    std::cout << "[TEST] ===== TemporalAnalyzer Unit Test =====\n";

    /* =====================================================
     Test 1: Empty / single-frame behavior
     ===================================================== */
    {
        TemporalAnalyzer temporal(5);

        auto m1 = temporal.update(0.0f, 0.0f);

        std::cout << "[TEST 1] Single update\n";
        std::cout << "  persistence = " << m1.persistence << "\n";
        std::cout << "  stability   = " << m1.stability << "\n";

        assert(nearlyEqual(m1.persistence, 0.0f));
        assert(nearlyEqual(m1.stability, 0.0f));
    }

    /* =====================================================
     Test 2: Stable persistent depression
     ===================================================== */
    {
        TemporalAnalyzer temporal(10);

        TemporalMetrics last{};
        for (int i = 0; i < 10; ++i)
            last = temporal.update(0.05f, 0.01f);

        std::cout << "\n[TEST 2] Stable depression signal\n";
        std::cout << "  persistence = " << last.persistence << "\n";
        std::cout << "  stability   = " << last.stability << "\n";

        assert(last.persistence > 10.0f);   // strong persistence
        assert(last.stability > 50.0f);     // very stable roughness
    }

    /* =====================================================
     Test 3: Fluctuating depression (should reduce persistence)
     ===================================================== */
    {
        TemporalAnalyzer temporal(10);

        float vals[] = {0.0f, 0.06f, 0.01f, 0.07f, 0.0f,
                        0.05f, 0.02f, 0.06f, 0.01f, 0.04f};

        TemporalMetrics last{};
        for (float v : vals)
            last = temporal.update(v, 0.01f);

        std::cout << "\n[TEST 3] Fluctuating depression\n";
        std::cout << "  persistence = " << last.persistence << "\n";
        std::cout << "  stability   = " << last.stability << "\n";

        assert(last.persistence < 10.0f);   // penalized
        assert(last.stability > 50.0f);     // roughness stable
    }

    /* =====================================================
     Test 4: Roughness instability (should reduce stability)
     ===================================================== */
    {
        TemporalAnalyzer temporal(10);

        float rough[] = {0.01f, 0.03f, 0.08f, 0.02f, 0.07f,
                         0.01f, 0.09f, 0.02f, 0.06f, 0.01f};

        TemporalMetrics last{};
        for (float r : rough)
            last = temporal.update(0.04f, r);

        std::cout << "\n[TEST 4] Unstable roughness\n";
        std::cout << "  persistence = " << last.persistence << "\n";
        std::cout << "  stability   = " << last.stability << "\n";

        assert(last.persistence > 10.0f);   // depression still consistent
        assert(last.stability < 50.0f);     // instability detected
    }

    /* =====================================================
     Test 5: History eviction
     ===================================================== */
    {
        TemporalAnalyzer temporal(3);

        temporal.update(0.01f, 0.01f);
        temporal.update(0.02f, 0.01f);
        temporal.update(0.03f, 0.01f);
        auto m = temporal.update(0.10f, 0.01f); // pushes out first

        std::cout << "\n[TEST 5] History eviction\n";
        std::cout << "  persistence = " << m.persistence << "\n";

        assert(m.persistence > 0.0f);
    }

    /* =====================================================
     Test 6: Reset
     ===================================================== */
    {
        TemporalAnalyzer temporal(5);

        temporal.update(0.05f, 0.01f);
        temporal.update(0.05f, 0.01f);
        temporal.reset();

        auto m = temporal.update(0.05f, 0.01f);

        std::cout << "\n[TEST 6] Reset behavior\n";
        std::cout << "  persistence = " << m.persistence << "\n";
        std::cout << "  stability   = " << m.stability << "\n";

        assert(nearlyEqual(m.persistence, 0.0f));
        assert(nearlyEqual(m.stability, 0.0f));
    }

    std::cout << "\n[TEST] ✅ TemporalAnalyzer fully validated\n";
    return 0;
}

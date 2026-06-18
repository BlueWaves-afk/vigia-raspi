// Host-compilable unit test for vigia_rri.hpp — no ROS2 needed.
//   g++ -std=c++17 -I../include test_vigia_rri.cpp -o /tmp/t && /tmp/t
#include "vigia_edge_node/vigia_rri.hpp"
#include <cmath>
#include <cstdio>

using namespace vigia;

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)
#define NEAR(a,b) (std::fabs((a)-(b)) < 1e-5f)

int main() {
    RriWeights w;  // 0.35 / 0.25 / 0.15 / 0.25

    // ── nothing present -> 0, tier None ──────────────────────────────────────
    {
        RriInputs in;
        CHECK(NEAR(compute_rri(w, in), 0.0f));
        CHECK(classify_tier(in) == RriTier::kNone);
    }

    // ── vision only: renormalizes to yolo_conf (weight collapses to 1.0) ─────
    {
        RriInputs in;
        in.have_yolo = true; in.yolo_conf = 0.8f;
        CHECK(NEAR(compute_rri(w, in), 0.8f));      // 0.35*0.8 / 0.35 == 0.8
        CHECK(classify_tier(in) == RriTier::kVisionOnly);
    }

    // ── full fusion matches the explicit weighted sum (weights sum to 1) ─────
    {
        RriInputs in;
        in.have_yolo=true;     in.yolo_conf=0.9f;
        in.have_geometry=true; in.geo_conf =0.6f;
        in.have_temporal=true; in.temp_conf=0.4f;
        in.have_iss=true;      in.iss_norm =0.5f;
        float expected = 0.35f*0.9f + 0.25f*0.6f + 0.15f*0.4f + 0.25f*0.5f;
        CHECK(NEAR(compute_rri(w, in), expected));
        CHECK(classify_tier(in) == RriTier::kFull);
    }

    // ── vision + depth (no IMU) renormalizes over present weights ────────────
    {
        RriInputs in;
        in.have_yolo=true;     in.yolo_conf=1.0f;
        in.have_geometry=true; in.geo_conf =0.0f;
        // den = 0.35+0.25 = 0.6 ; num = 0.35*1.0 = 0.35 ; -> 0.5833...
        CHECK(NEAR(compute_rri(w, in), 0.35f/0.60f));
        CHECK(classify_tier(in) == RriTier::kVisionDepth);
    }

    // ── inputs are clamped to [0,1] before weighting ────────────────────────
    {
        RriInputs in;
        in.have_yolo=true; in.yolo_conf=5.0f;   // clamped to 1.0
        CHECK(NEAR(compute_rri(w, in), 1.0f));
        in.yolo_conf = -3.0f;                   // clamped to 0.0
        CHECK(NEAR(compute_rri(w, in), 0.0f));
    }

    if (failures == 0) std::printf("vigia_rri: ALL TESTS PASSED\n");
    else               std::printf("vigia_rri: %d FAILURE(S)\n", failures);
    return failures ? 1 : 0;
}

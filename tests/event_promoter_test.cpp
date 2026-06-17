#include "event_promoter.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

using namespace vigia;

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

static HazardObservation makeValidObs()
{
    HazardObservation obs{};
    obs.rri           = 0.80f;
    obs.iss           = 0.40f;
    obs.yolo_conf     = 0.90f;
    obs.geometry_conf = 0.70f;
    obs.temporal_conf = 0.60f;
    obs.bbox_x        = 100;
    obs.bbox_y        = 200;
    obs.bbox_w        = 80;
    obs.bbox_h        = 60;
    obs.lat           = 12.971600;
    obs.lon           = 77.594600;
    obs.speed_ms      = 8.3f;
    obs.hdop          = 1.2f;
    obs.gps_fix_type  = 3;
    obs.gps_valid     = true;
    obs.frame_index   = 42;
    return obs;
}

int main()
{
    std::cout << "[TEST] ===== EventPromoter Unit Test (M7a) =====\n\n";

    EventPromoter::Config cfg{};
    cfg.rri_threshold  = 0.75f;
    cfg.dedup_radius_m = 5.0f;
    cfg.dedup_window_s = 30.0f;
    cfg.require_gps    = true;
    cfg.max_hdop       = 2.5f;

    EventPromoter promoter(cfg);

    /* RRI gate */
    {
        auto obs = makeValidObs();
        obs.rri = 0.74f;
        EXPECT(!promoter.submit(obs), "rejects rri below threshold");
    }
    {
        auto obs = makeValidObs();
        obs.rri = 0.75f;
        EXPECT(promoter.submit(obs), "accepts rri at threshold");
    }

    /* Geometry gate — proves MiDaS path (blocks YOLO-only promotions) */
    {
        EventPromoter fresh(cfg);
        auto obs = makeValidObs();
        obs.geometry_conf = 0.0f;
        EXPECT(!fresh.submit(obs), "rejects zero geometry confidence");
    }
    {
        EventPromoter fresh(cfg);
        auto obs = makeValidObs();
        obs.geometry_conf = 0.01f;
        EXPECT(fresh.submit(obs), "accepts positive geometry confidence");
    }

    /* GPS gate */
    {
        EventPromoter fresh(cfg);
        auto obs = makeValidObs();
        obs.gps_valid = false;
        EXPECT(!fresh.submit(obs), "rejects invalid GPS when required");
    }
    {
        EventPromoter fresh(cfg);
        auto obs = makeValidObs();
        obs.hdop = 3.0f;
        EXPECT(!fresh.submit(obs), "rejects hdop above max");
    }
    {
        EventPromoter fresh(cfg);
        auto obs = makeValidObs();
        obs.gps_fix_type = 1;
        EXPECT(!fresh.submit(obs), "rejects fix_type below 2");
    }
    {
        EventPromoter::Config benchCfg = cfg;
        benchCfg.require_gps = false;
        EventPromoter bench(benchCfg);
        auto obs = makeValidObs();
        obs.gps_valid = false;
        EXPECT(bench.submit(obs), "bench mode allows invalid GPS");
    }

    /* Spatial dedup — same coords within 5 m / 30 s window */
    {
        EventPromoter fresh(cfg);
        auto obs = makeValidObs();
        obs.lat = 40.000000;
        obs.lon = -74.000000;
        EXPECT(fresh.submit(obs), "first observation at location promotes");

        auto dup = makeValidObs();
        dup.lat = 40.000010;
        dup.lon = -74.000010;
        EXPECT(!fresh.submit(dup), "nearby duplicate within dedup radius suppressed");

        auto far = makeValidObs();
        far.lat = 40.010000;
        far.lon = -74.010000;
        EXPECT(fresh.submit(far), "distant observation still promotes");
    }

    /* Dedup window expiry */
    {
        EventPromoter::Config shortWindow = cfg;
        shortWindow.dedup_window_s = 0.05f;
        EventPromoter fresh(shortWindow);

        auto obs = makeValidObs();
        obs.lat = 51.500000;
        obs.lon = -0.120000;
        EXPECT(fresh.submit(obs), "initial observation promotes");

        std::this_thread::sleep_for(std::chrono::milliseconds(80));

        auto retry = makeValidObs();
        retry.lat = 51.500001;
        retry.lon = -0.120001;
        EXPECT(fresh.submit(retry), "duplicate allowed after dedup window expires");
    }

    /* device_seq increments only on successful promotion */
    {
        EventPromoter fresh(cfg);
        EXPECT(fresh.deviceSeq() == 0, "device_seq starts at zero");

        auto obs = makeValidObs();
        obs.lat = 35.000000;
        obs.lon = 139.000000;
        EXPECT(fresh.submit(obs), "promotion succeeds");
        EXPECT(fresh.deviceSeq() == 1, "device_seq increments on promote");
    }

    std::cout << '\n';
    if (g_failed == 0) {
        std::cout << "[TEST] All EventPromoter tests passed.\n";
        return 0;
    }

    std::cerr << "[TEST] " << g_failed << " test(s) failed.\n";
    return 1;
}

#pragma once
//
// ttc_estimator.hpp — Monocular Time-To-Contact estimator (M11 FCW).
//
// Algorithm (§5.2 of 06_adas_transition_contracts.md):
//   Primary:  TTC_bbox = h / (dh/dt)  — bbox-height looming, scale-invariant, metric-free.
//   Guard:    MiDaS relative-depth gradient sign — rejects TTC_bbox when depth says "receding".
//   Noise:    3-frame median filter on bbox height before differentiating.
//             Suppress when bbox area < 0.5 % of frame area.
//             Suppress when ego yaw-rate > YAW_RATE_THRESHOLD_DEG_S (turning).
//
// One TtcEstimator instance per vehicle/pedestrian/cyclist track (keyed on classId in
// BleGattNode; for simplicity we track the single highest-confidence target per class).
//

#include <array>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <limits>

namespace vigia {

struct TtcResult {
    bool  valid{false};
    float ttc_s{std::numeric_limits<float>::infinity()};
    int   class_id{-1};
};

class TtcEstimator {
public:
    explicit TtcEstimator(int classId, float frameWidthPx, float frameHeightPx)
        : class_id_(classId)
        , frame_area_(frameWidthPx * frameHeightPx)
    {}

    // Feed one frame's bbox (height in pixels) and optional MiDaS relative depth
    // over the bbox region (median inverse-depth).  dt_s = elapsed since last frame.
    // Returns a TtcResult; valid == true only when the alert trigger fires.
    TtcResult update(
        float bbox_h_px,
        float bbox_area_px2,
        float midas_z_rel,          // pass NaN to skip depth guard
        float yaw_rate_deg_s,       // pass 0 if unavailable
        float dt_s,
        float ego_speed_ms,
        float base_ttc_s            // profile-scaled threshold (BaseTtc × sProfile)
    ) {
        // Guard: bbox too small (far away / noisy)
        if (bbox_area_px2 / frame_area_ < MIN_BBOX_AREA_FRAC)
            return reset();

        // Guard: ego is turning — bbox scale changes for geometric, not collision, reason
        if (std::abs(yaw_rate_deg_s) > YAW_RATE_THRESHOLD_DEG_S)
            return reset();

        // Guard: vehicle not moving (parking lot)
        if (ego_speed_ms < MIN_EGO_SPEED_MS)
            return reset();

        // Push to 3-frame ring buffer; compute median
        h_ring_[ring_head_] = bbox_h_px;
        ring_head_ = (ring_head_ + 1) % RING_SIZE;
        if (ring_count_ < RING_SIZE) { ++ring_count_; }
        if (ring_count_ < RING_SIZE) return {};  // need 3 frames before computing

        const float h_median = median3(h_ring_[0], h_ring_[1], h_ring_[2]);

        // dh/dt
        if (dt_s <= 0.f) return {};
        const float dhdt = (h_median - prev_h_median_) / dt_s;
        prev_h_median_ = h_median;

        // TTC_bbox
        if (dhdt <= 0.f) return {};   // bbox shrinking → receding, no collision
        const float ttc = h_median / dhdt;
        if (!std::isfinite(ttc) || ttc <= 0.f) return {};

        // Depth guard: if MiDaS inverse-depth is decreasing (target receding in depth), skip
        if (!std::isnan(midas_z_rel)) {
            const float dz = midas_z_rel - prev_midas_z_;
            prev_midas_z_ = midas_z_rel;
            if (frames_seen_ > 0 && dz > MIDAS_RECEDING_THRESHOLD) {
                // Depth is increasing (object further away) → suppress bbox-based alert
                return {};
            }
        }

        ++frames_seen_;

        if (ttc < base_ttc_s) {
            return TtcResult{true, ttc, class_id_};
        }
        return {};
    }

    void resetTrack() {
        ring_count_ = 0;
        ring_head_  = 0;
        frames_seen_ = 0;
        prev_h_median_ = 0.f;
        prev_midas_z_  = 0.f;
    }

private:
    TtcResult reset() { resetTrack(); return {}; }

    static float median3(float a, float b, float c) {
        if (a > b) std::swap(a, b);
        if (b > c) std::swap(b, c);
        if (a > b) std::swap(a, b);
        return b;
    }

    static constexpr int   RING_SIZE                 = 3;
    static constexpr float MIN_BBOX_AREA_FRAC        = 0.005f;   // 0.5 % of frame
    static constexpr float YAW_RATE_THRESHOLD_DEG_S  = 8.0f;     // suppress during turns
    static constexpr float MIN_EGO_SPEED_MS          = 5.0f;     // ~18 km/h
    static constexpr float MIDAS_RECEDING_THRESHOLD  = 0.05f;    // inv-depth increasing

    int   class_id_;
    float frame_area_;

    std::array<float, RING_SIZE> h_ring_{};
    int   ring_head_{0};
    int   ring_count_{0};
    int   frames_seen_{0};
    float prev_h_median_{0.f};
    float prev_midas_z_{0.f};
};

}  // namespace vigia

#include "iss_filter.hpp"

#include <algorithm>
#include <cmath>

namespace vigia {

IssFilter::IssFilter(Config cfg)
    : cfg_(cfg)
{}

IssFilter::Result IssFilter::update(float normalizedIss, float speedMs)
{
    Result r{};

    // ── Speed floor gate ──────────────────────────────────────────────────────
    // Below the minimum speed the vehicle is effectively stationary — door
    // slams, engine idle shudder, and loading vibration all produce ISS > 0.
    // None of those are road-hazard events.
    const float speedKmh = speedMs * 3.6f;
    if (speedKmh < cfg_.speedFloorKmh) {
        consecutiveAbove_ = 0;
        // Keep background tracking even while stopped so it doesn't drift.
        // (Don't update smoothed so we don't corrupt it with parking noise.)
        r.backgroundIss = background_;
        return r;
    }

    // ── Stage 1: EMA smoother ─────────────────────────────────────────────────
    // Seed from the first real sample to avoid a long ramp-up at boot.
    if (!initialized_) {
        smoothed_    = normalizedIss;
        background_  = normalizedIss;
        initialized_ = true;
    }

    smoothed_ = cfg_.emaAlpha * normalizedIss
              + (1.0f - cfg_.emaAlpha) * smoothed_;

    // ── Stage 2: Background noise-floor tracker ───────────────────────────────
    // Updates on every in-motion frame.  The very slow α means a genuine
    // pothole impact (≤ 1 s duration) barely moves the background — the
    // baseline only tracks road texture and engine vibration.
    background_ = cfg_.bgAlpha       * background_
                + (1.0f - cfg_.bgAlpha) * smoothed_;

    // Clamp background so it can never exceed the smoothed value
    // (prevents runaway if ISS drops suddenly, e.g. smooth highway after bumpy lane).
    background_ = std::min(background_, smoothed_);

    const float detrended = std::max(0.0f, smoothed_ - background_);

    // ── Stage 3: Speed-adaptive threshold + consecutive-frame gate ───────────
    // Raising the bar at highway speeds compensates for the intrinsically
    // higher road-induced vibration on NH-level motorway surfaces.
    const float threshold = cfg_.spikeThreshold
                          + cfg_.speedAdaptiveFactor * speedKmh;

    if (detrended >= threshold) {
        ++consecutiveAbove_;
    } else {
        consecutiveAbove_ = 0;
    }

    r.smoothedIss        = smoothed_;
    r.backgroundIss      = background_;
    r.detrendedIss       = detrended;
    r.effectiveThreshold = threshold;
    r.isGenuineImpact    = (consecutiveAbove_ >= cfg_.minConsecutiveFrames);

    return r;
}

void IssFilter::reset()
{
    smoothed_         = 0.0f;
    background_       = 0.0f;
    consecutiveAbove_ = 0;
    initialized_      = false;
}

} // namespace vigia

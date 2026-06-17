#pragma once

namespace vigia {

/*
 * IssFilter — three-stage noise filter for the Impact Severity Score
 * ==================================================================
 *
 * Indian road context (calibration basis)
 * ----------------------------------------
 * Engine/tyre idle vibration  :  awz ≈ 0.3–1.5 m/s²  →  ISS_norm ≈ 0.02–0.06
 * Rough road texture          :  awz ≈ 0.5–2.5 m/s²  →  ISS_norm ≈ 0.03–0.10
 * Small road undulation       :  awz ≈ 2–4   m/s²    →  ISS_norm ≈ 0.08–0.16
 * Speed breaker @ 20 km/h     :  awz ≈ 8–15  m/s²    →  ISS_norm ≈ 0.30–0.60
 * Pothole @ 30 km/h           :  awz ≈ 5–20  m/s²    →  ISS_norm ≈ 0.20–0.80
 *
 * (ISS_norm = clamp(|awz| / (max(v_ms,2.0) × ISS_MAX), 0, 1), ISS_MAX = 3.0)
 *
 * Filter pipeline (per frame, ~15 Hz)
 * ------------------------------------
 * Stage 1 — EMA smoother
 *   smoothed = α·raw + (1−α)·prev_smoothed
 *   Kills single-frame chassis jolts.  α = 0.4 → ~2-frame half-life.
 *
 * Stage 2 — Background noise-floor tracker
 *   background = β·background + (1−β)·smoothed
 *   β = 0.997 → settles in ~10 s at 15 FPS.  Tracks road texture and engine
 *   vibration without following a genuine impact (which lasts < 1 s).
 *   detrended = max(0, smoothed − background)
 *
 * Stage 3 — Consecutive-frame gate
 *   Requires detrended ≥ adaptive_threshold for at least minFrames consecutive
 *   frames before marking isGenuineImpact = true.  At 15 FPS, minFrames = 2
 *   means the impact must last ≥ 133 ms — long enough for any real pothole but
 *   shorter than the longest Indian speed breaker.
 *
 * Speed-adaptive threshold
 *   threshold = spikeThreshold + speedAdaptiveFactor × speed_kmh
 *   At highway speeds road noise is intrinsically higher; raising the bar
 *   prevents false positives on motorway rumble strips.
 *
 * Usage
 * -----
 *   IssFilter filter;                              // one instance per Coordinator
 *   auto res = filter.update(normalizedIss, speedMs);
 *   if (res.isGenuineImpact)
 *       fin.imuIss = res.detrendedIss;             // background-subtracted score
 *   else
 *       fin.imuIss = 0.0f;
 */

class IssFilter {
public:
    struct Config {
        // Stage 1 — EMA smoother
        // Weight on the incoming sample.  Higher = faster response, more noise.
        // 0.4 → half-life ≈ 2 frames (~130 ms at 15 FPS).
        float emaAlpha{0.40f};

        // Stage 2 — Background tracker
        // Update rate for the noise floor.  Closer to 1 = slower adaptation.
        // 0.997 → ~10 s for step response to settle at 15 FPS.
        float bgAlpha{0.997f};

        // Stage 3 — Detrended ISS needed to start counting consecutive frames.
        // Baseline: 0.10 = 10% of ISS_MAX above road texture noise floor.
        float spikeThreshold{0.10f};

        // Speed-adaptive component added to spikeThreshold.
        // 0.0015 × 60 km/h = +0.09 → threshold = 0.19 at 60 km/h.
        // Compensates for higher road noise at highway speeds (NH-level roads).
        float speedAdaptiveFactor{0.0015f};

        // Number of consecutive frames the detrended ISS must exceed the
        // threshold to be flagged as a genuine impact.
        // 2 frames = 133 ms at 15 FPS — filters chassis micro-jolts.
        int minConsecutiveFrames{2};

        // Speed floor below which ISS is suppressed entirely (km/h).
        // Prevents false detections from door slams / loading at standstill.
        float speedFloorKmh{3.0f};
    };

    struct Result {
        float smoothedIss{0.0f};        // Stage-1 EMA output (no background sub)
        float backgroundIss{0.0f};      // Estimated road-noise floor
        float detrendedIss{0.0f};       // smoothed − background, clamped to ≥ 0
        float effectiveThreshold{0.0f}; // Speed-adaptive threshold that was applied
        bool  isGenuineImpact{false};   // All three stages passed — use for fusion
    };

    explicit IssFilter(Config cfg = Config{});

    // Feed one normalized ISS value (output of normalizeISS()) and the
    // current GPS speed in m/s.  Must be called once per frame in order.
    Result update(float normalizedIss, float speedMs);

    // Reset all internal state (e.g. after a long stop or bridge reconnect).
    void reset();

    const Config& config() const { return cfg_; }

private:
    Config cfg_;
    float  smoothed_{0.0f};
    float  background_{0.0f};
    int    consecutiveAbove_{0};
    bool   initialized_{false};
};

} // namespace vigia

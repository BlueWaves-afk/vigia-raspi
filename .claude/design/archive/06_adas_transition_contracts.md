# VIGIA ADAS DePIN Edge Node
## ADAS Capability Transition Contracts
**Document:** `06_adas_transition_contracts.md`
**Depends on:** `01`–`05` (APPROVED), M7/M8 hazard-event + BLE transport (SHIPPED)
**Status:** AWAITING APPROVAL — No implementation until sign-off
**Scope:** Phase 6 — Transition from a passive hazard-logging blackbox to an **active advisory ADAS** without drive-by-wire integration.

> **Phase-numbering note.** In `05` the label "Phase 6" referred to DePIN Security & Attestation. That work shipped in M7 (hazard event logging, PostGIS ingest, HMAC signing) and M8 (BLE transport, degraded-mode RRI). Phase 6 is hereby re-scoped to the **ADAS Capability Transition** described below. The DePIN signing contracts remain canonical in `04_phase2_depin_signing_contracts.md` and `05`.

---

## 0. Thesis: Active Contextual Analytics, not Beeps

Western ADAS (Mobileye, Bosch, Tesla) is brittle on Indian roads because its perception stack assumes painted lanes, uniform traffic, and signed speed limits. VIGIA inverts the dependency: it treats the **road itself** as the primary signal (RRI, ISS, spatial latent) and the camera as a secondary cue. This document specifies the five software modules and two hardware add-ons that move VIGIA from *"records a pothole and pays you for it"* to *"warns you about the pothole, coaches you through it, and knows you're tired before you do."*

The transition adds **zero mandatory hardware** for Modules 1–5; the BOM upgrades in §8 are strictly additive and independently shippable.

### 0.1 Competitive topology

| Architectural layer | Industry-leader pattern | VIGIA Phase-6 moat |
|---|---|---|
| Crowd-sourced mapping | Mobileye REM — proprietary compressed map snippets, fleet-gated | **DePIN geohash network** — decentralized nodes index RRI + spatial latents into PostGIS; capture is *profitable* via micro-incentives |
| Surface-friction analytics | Bosch + NIRA Dynamics — wheel-slip algorithms, enterprise-gated | **Edge IMU classification** — `SensorProcessor` categorizes vertical-G anomalies (pothole vs. seam vs. speed-bump) at a fraction of BOM cost |
| Conversational control | Tesla FSD + Grok — voice → path planner (needs drive-by-wire) | **Advisory co-pilot** — voice → hazard avoidance, routing, coaching; no actuation, no CAN write |

---

## 1. Non-Negotiable Invariants

| Invariant | Enforcement |
|---|---|
| **Advisory-only — VIGIA never actuates the vehicle** | No CAN-bus *write*. OBD-II dongle (§8.1) is read-only. All outputs are voice/visual via the companion app. This keeps VIGIA out of ISO 26262 ASIL-D actuation liability while still delivering Level-0/Level-1 advisory value. |
| **All Phase-6 inference runs on existing topics** | Modules 1, 2, 5 consume only `ImuSample`, `GpsFix`, `FusionOutput`. Module 3 (TTC) reuses the live `PerceptionResult` (YOLO) + `DepthGeometryMetrics` (MiDaS). No new perception model is trained for Phase 6. |
| **Driver-profile scale factor is global and hot-reloadable** | `S_profile` (§3) is a single config value read at session start and on BLE control-char write. No threshold is hardcoded in a node; every warning window derives from `S_profile × base`. |
| **Safety alerts preempt conversation** | A CRITICAL advisory (TTC < 3 s, §5) issues `TextToSpeech.QUEUE_FLUSH` and cancels any in-flight Sarvam stream — identical to the M8 hazard-alert path. Conversation resumes after. |
| **No raw video leaves the edge** | TTC and DMS run on-device. Only the *event* (TTC value, drowsiness score) crosses BLE/MQTT, never frames. Consistent with `05` §0. |
| **Reaction-lag math uses `steady_clock`** | All Δt measurements (§4) use `std::chrono::steady_clock` to immunize against NTP jumps, per `05` §0. |

---

## 2. System Integration Map

```
EDGE (Pico 2 → Pi, C++ Coordinator)            COMPANION APP (Kotlin)               CLOUD (AWS)
─────────────────────────────────             ──────────────────────              ──────────────
SensorBridge → SensorProcessor                ContextAggregator                    road-ahead λ
  │ ImuSample (quat + accel)                     │ GPS 1Hz + BLE telemetry            │ HazardsTable
  │ GpsFix (lat/lon/speed/hdop)                  │                                    │ (geohash PK)
  ▼                                             ▼                                    ▼
FusionEngine ── imuIss, RRI ──► BleGatt ──BLE──► BleDataStreamer ──► RouteAheadMonitor ──► /v1/road-ahead
  │                                             │                     LaneDriftDetector
  ├─ M3 TtcEstimator (NEW)  ──────────BLE──────►│ FatigueProxyScorer (NEW, Module 2)
  └─ PerceptionResult + DepthGeometryMetrics    │ DriverProfile S_profile (NEW, Module 1)
                                                │ HarshEventLogger → SQLite (NEW, Module 5)
                                                │ SpeedCurveAdvisor ◄── Overpass (NEW, Module 4)
                                                ▼
                                          TtsManager (profile-scaled rate/gain)
```

**Ownership split.** Modules 1, 2, 4, 5 live in the **companion app** (`:core:sensor` + `:feature:copilot`) because they fuse cloud + GPS + conversational state. Module 3 (TTC) lives on the **edge** because it must run at frame rate against the live depth tensor and cannot tolerate BLE round-trip latency.

---

## 3. Module 1 — Driver Profile System

A single global scale factor reshapes every advisory threshold and the TTS delivery, so the same binary serves a nervous 18-year-old and a 75-year-old with a 0.4 s slower reaction time.

### 3.1 Profile definition

```kotlin
// :core:model/DriverProfile.kt
enum class DriverProfile(val sProfile: Float, val ttsRate: Float, val ttsGainDb: Float) {
    EXPERT (sProfile = 0.5f, ttsRate = 1.10f, ttsGainDb = 0f),   // terse, only CRITICAL
    NEW    (sProfile = 1.5f, ttsRate = 0.95f, ttsGainDb = +2f),  // verbose coaching
    ELDERLY(sProfile = 3.0f, ttsRate = 0.80f, ttsGainDb = +4f),  // early, slow, loud
}
```

### 3.2 Threshold derivation (the only formula that matters)

Every distance/time threshold in Modules 3–5 is expressed as a base value scaled by `S_profile`:

```
WarningDistance  = BaseWarningDistance × S_profile
WarningLeadTime  = BaseLeadTime        × S_profile
TtcAlertThreshold = BaseTtc            × S_profile      // EXPERT 1.5s, NEW 4.5s, ELDERLY 9.0s
DriftSensitivity  = BaseDriftDeg       ÷ S_profile      // ELDERLY trips at smaller oscillation
```

Worked example — route-ahead pothole warning (base 200 m):

| Profile | `S_profile` | Warns at | TTS rate | Rationale |
|---|---|---|---|---|
| Expert | 0.5 | 100 m | 1.10× | minimal chatter, fast speech |
| New | 1.5 | 300 m | 0.95× | time to process + react |
| Elderly | 3.0 | 600 m | 0.80× | compensate slower reaction speed |

### 3.3 Wiring points
- `RouteAheadMonitor.LOOK_AHEAD_DISTANCES_M` becomes `baseDistances.map { it * profile.sProfile }`.
- `LaneDriftDetector.DRIFT_MIN_DEG` becomes `BASE_DRIFT_DEG / profile.sProfile`.
- `TtsManager` applies `ttsRate` to `SarvamTtsClient` (`speaking_rate` param) and `ttsGainDb` to the `AudioTrack` volume.
- Profile is selectable in onboarding; persisted in DataStore; pushed to edge via BLE control-char so the edge TTC threshold (§5) scales too.

### 3.4 Acceptance criteria
- [ ] Switching profile at runtime re-derives all four thresholds within one sensor tick (no restart).
- [ ] ELDERLY profile produces a pothole warning ≥ 2× the lead distance of EXPERT for the identical hazard.
- [ ] TTS speaking rate audibly differs across profiles (measured WPM delta ≥ 15%).

---

## 4. Module 2 — Fatigue Proxy Score (hardware-free)

Drowsiness is inferred from the **widening reaction lag** between a road event and the driver's corrective response, fused with velocity micro-variance. No camera, no wearable.

### 4.1 The reaction-lag signal

When the wheels hit a pothole/seam, `SensorProcessor` emits a vertical-impulse spike (the same `computeVerticalImpulse()` already used for ISS). An alert driver corrects within a few hundred ms (a lift-off, a micro-brake, a steering nudge); a fatigued driver lags.

```
Δt_react = t(GPS speed/bearing inflection)  −  t(IMU vertical-G spike > τ_spike)
```

- `t(spike)` is the `steady_clock` timestamp when `|a_world.z|` exceeds `τ_spike` (reuse `kIssMax`-relative threshold, e.g. 0.5·kIssMax).
- `t(inflection)` is the first GPS sample within a 2 s window after the spike where `|d(speed)/dt|` or `|d(bearing)/dt|` exceeds a small response threshold.
- If no inflection occurs in 2 s, `Δt_react` is capped at 2.0 s (the driver did not react at all — strongest fatigue signal).

### 4.2 Velocity micro-variance

Fatigued drivers hold speed less steadily (unconscious drift up/down). Over a sliding 30 s window:

```
σ_v = stddev(speed_ms over last 30 s)   // normalized by mean speed → coefficient of variation
```

### 4.3 Fatigue score fusion

A trend, not an instant value. Maintain EWMA baselines established in the first 10 min of the trip, then score deviation:

```
F = w1 · clamp(Δt_react_ewma / Δt_baseline − 1, 0, 1)
  + w2 · clamp(σ_v / σ_v_baseline − 1, 0, 1)
  + w3 · clamp(drift_freq / drift_baseline − 1, 0, 1)      // from LaneDriftDetector, last 30 min

   w1 = 0.45, w2 = 0.25, w3 = 0.30      // reaction lag dominates
```

- `F < 0.4` — nominal, silent.
- `0.4 ≤ F < 0.6` — gentle nudge: *"You seem a little less sharp than earlier. All okay?"*
- `F ≥ 0.6` — escalate, profile-scaled lead: *"You're showing signs of fatigue. There's a rest stop in 4 km — want me to guide you there?"* (triggers Module 4 amenity query).

### 4.4 Why this beats a naive timer
A 2-hour timer fires for everyone identically. `F` fires *only when the driver actually degrades*, and the reaction-lag term is a direct physiological proxy — it is the same quantity (response latency) that EEG/HRV studies correlate with KSS drowsiness, achievable here from two sensors you already have.

### 4.5 Acceptance criteria
- [ ] `Δt_react` is measured per qualifying spike and EWMA-smoothed; raw values logged to SQLite for tuning.
- [ ] Baseline established in first 10 min; no fatigue alert can fire before baseline is set.
- [ ] Synthetic replay (injected widening Δt) drives `F` past 0.6 and fires exactly one escalation with 30 s debounce.

---

## 5. Module 3 — Forward Collision Time-to-Contact (edge)

This is the software replacement for the Mobileye FCW chip. It runs in the `Coordinator` against the live perception + depth outputs — **not** over BLE.

### 5.1 Inputs (already produced per frame)
- `PerceptionResult` → `Detection{ bbox, yoloConfidence, class }` for vehicles/pedestrians/cyclists.
- `DepthGeometryMetrics` → MiDaS **relative** inverse-depth for the bbox region.
- `GpsFix.speed_ms` → ego forward speed.

### 5.2 Closing-velocity estimation (two independent estimators, fused)

MiDaS is relative-only, so absolute distance is unavailable. We use two cues and fuse:

**(a) Bounding-box scale expansion** (monocular looming — scale-invariant, needs no metric depth):
```
TTC_bbox = h / (dh/dt)
```
where `h` is bbox height and `dh/dt` its growth rate across consecutive frames. A target on a collision course grows hyperbolically; `TTC_bbox` is the classic monocular time-to-contact and requires **no distance calibration**.

**(b) Depth-gradient cue** (sign/consistency check from MiDaS):
```
TTC_depth = Z_rel / (dZ_rel/dt)      // Z_rel = median inverse-depth over bbox
```
used to validate sign and reject `TTC_bbox` artifacts from bbox jitter.

**Fusion:** accept the alert only if both estimators agree the target is approaching and `TTC_bbox` is finite and positive:
```
TTC = TTC_bbox          when sign(dZ_rel/dt) confirms approach
      ∞ (no alert)      otherwise
```

### 5.3 Trigger
```
if (TTC < BaseTtc × S_profile) and (yoloConfidence > 0.55) and (ego speed_ms > 5):
    emit ForwardCollisionEvent{ ttc, class, bbox_center }   // BLE control-char, CRITICAL
```
The companion app renders it via the M8 CRITICAL path (`QUEUE_FLUSH`, orb → Alert): *"Brake — vehicle stopping ahead."*

### 5.4 Calibration & honesty
`TTC_bbox` is genuinely metric-free, but bbox-height noise makes raw `dh/dt` jittery. Required mitigations:
- 3-frame median filter on `h` before differentiating.
- Reject targets with bbox area < 0.5% of frame (too far / too noisy).
- Suppress during ego yaw-rate spikes (turning) where bbox scale changes for non-collision reasons.

### 5.5 Acceptance criteria
- [ ] On a stationary-target approach clip at 30 km/h, TTC converges monotonically and fires once below threshold.
- [ ] No false fire during a 10-min urban turning sequence (bbox scale changes without true closing).
- [ ] End-to-end latency spike → app audio < 250 ms (edge compute + BLE).

---

## 6. Module 4 — OpenStreetMap Speed & Curve Advisory (cloud-assisted)

Adds "see around the corner" foresight using free map geometry, served through the existing Python ingest backend so the edge/app never call Overpass directly.

### 6.1 Server side (backend extension to road-ahead path)
When the app's `RoadAheadClient` requests a geohash-6 sector, the backend additionally:
1. Queries the **Overpass API** for `highway=*` ways within the sector bbox (cached per geohash, 24 h TTL in DynamoDB to respect Overpass rate limits).
2. Extracts `maxspeed` tags and computes **curvature** from way node polylines (radius of curvature `R` via three-point circumfit).
3. Packages a compact advisory array into the `/v1/road-ahead` response:

```json
"road_geometry": [
  { "type": "speed_limit", "value_kmh": 40, "distance_m": 420 },
  { "type": "curve", "radius_m": 85, "advised_kmh": 35, "distance_m": 260, "direction": "left" }
]
```

### 6.2 Advised curve speed
```
advised_kmh = sqrt(μ · g · R) × 3.6        // μ = 0.6 dry lateral grip assumption, conservative
```
If `μ` should drop (Module: road-surface-state, future), `advised_kmh` scales down. Advisory only — never enforced.

### 6.3 App side
`SpeedCurveAdvisor` consumes `road_geometry`, applies `S_profile` lead distance, and emits proactive TTS through the existing `RouteAheadMonitor.ProactiveEvent` channel:
- *"Speed limit drops to 40 in 400 meters."*
- *"Sharp left curve in 250 meters — ease down to 35."*

### 6.4 Acceptance criteria
- [ ] Overpass results cached; a repeated sector request issues zero external calls within TTL.
- [ ] Curve advisory fires before the curve apex with profile-scaled lead.
- [ ] Graceful degradation: Overpass timeout returns hazards-only (no `road_geometry`), no user-visible error.

---

## 7. Module 5 — Harsh-Event Coaching & Trip Debrief

Closes the behavioral loop. Minor events are logged locally (not streamed) and compiled into a spoken summary at engine-off.

### 7.1 Local logging
`HarshEventLogger` writes to the app's SQLite (Room) during the trip — no cellular cost:

```kotlin
@Entity data class HarshEvent(
    val tripId: String, val type: Type, val magnitude: Float,
    val lat: Double, val lon: Double, val timestampMs: Long,
)   // Type: HARSH_BRAKE, HARSH_ACCEL, SHARP_TURN, ROAD_IMPACT
```

- `HARSH_BRAKE/ACCEL` — longitudinal-G from `ImuSample` (or, when present, OBD-II brake pressure §8.1) beyond profile-scaled threshold.
- `ROAD_IMPACT` — the same vertical-impulse spikes Module 2 already detects, geo-tagged.

### 7.2 Trip-end trigger
Engine-off is detected by **BLE disconnect** of the blackbox (M8 transport) or OBD-II ignition-off. On trigger, compile and speak:

> *"Trip summary: 3 harsh braking events on Bellary Road — your following distance may be short. Road quality there is consistently poor; I'll route around it tomorrow if you like. You logged 12 verified hazards today — that's ₹2.40 added to your wallet."*

### 7.3 The DePIN tie-in
The debrief is the natural surface to surface **earnings** (M7 rewards ledger) alongside safety — uniquely VIGIA: every harsh-event geo-tag is also a candidate hazard observation feeding `EventPromoter`. Safety coaching and token rewards share one data path.

### 7.4 Acceptance criteria
- [ ] Events logged locally with zero network calls during the trip.
- [ ] Debrief compiles within 1 s of engine-off and speaks a profile-scaled summary.
- [ ] Earnings figure reconciles with the M7 rewards-balance endpoint.

---

## 8. Hardware Upgrade Sequence (additive, $15–$40 BOM)

Ship in this exact order for maximum utility per dollar.

### 8.1 Priority 1 — OBD-II BLE dongle (~$25, read-only)
```
[Blackbox] ──BLE──► [OBD-II dongle] ──CAN(read)──► vehicle
```
Unlocks ground truth that no camera/IMU can provide:
- **Brake pressure** → disambiguates IMU spikes: chassis shakes + brake pressure 0 ⇒ road hazard; brake pressure pegged ⇒ validated harsh-brake event (Module 5/2 calibration).
- **Wheel speed / RPM** → cross-checks GPS speed for ISS normalization when GPS HDOP is poor (`isGpsFixUsable` fallback today uses `kVMin`; OBD wheel-speed is a far better floor).
- **Throttle position** → harsh-accel ground truth.

**Invariant:** OBD reads only. PIDs are polled; no UDS write, no actuation. Integrates as a new `SensorBridge` source feeding `FusionInput`.

### 8.2 Priority 2 — Wide-angle cabin IR camera (~$20)
```
[Cabin IR cam] ──► on-device DMS ──► drowsiness/distraction score
```
- Adds **direct** eye-closure (PERCLOS) and head-nod detection — the EEG/HRV-grade signal — fused with the Module 2 proxy `F` for an ironclad fatigue score.
- IR (no IR-cut filter) for night operation.
- Runs a small face/eye model on the existing ONNX runtime; **only the score** crosses to the app, never frames (§1 invariant).

### 8.3 Deferred (note only)
Rear/side USB-C camera (blind-spot), ultrasonic/radar (metric distance), steering-wheel ECG. Not in Phase 6 scope.

---

## 9. Phased Rollout & Milestones

| Milestone | Modules | HW | Demo-able outcome |
|---|---|---|---|
| **M9** | 1 (Driver Profile) + 5 (Harsh-event/Debrief) | none | Profile selector; spoken trip debrief with earnings — pure software, instantly demo-able |
| **M10** | 2 (Fatigue Proxy) + 4 (Speed/Curve advisory) | none | "You seem tired, rest stop in 4 km"; "sharp curve ahead, slow to 35" |
| **M11** | 3 (Edge TTC FCW) | none | Forward-collision voice warning on approach clip |
| **M12** | OBD-II integration | dongle | Brake-pressure-validated events; OBD speed floor for ISS |
| **M13** | Cabin DMS | IR cam | PERCLOS-fused fatigue score — Level-1 driver-monitoring claim |

M9–M11 require **no hardware** and are the highest-leverage demos for the **IITM Road Safety Hackathon finals (15–16 July)** and Samsung SFT rounds.

---

## 10. SFT / Hackathon Positioning

Lead the jury narrative with the moat, not the feature list:

1. **"Western ADAS is disabled by 85% of Indian users because it expects painted lanes."** VIGIA is built from the road up, for chaos.
2. **"Every node makes the network smarter — and pays the driver for it."** This is the Tesla fleet-learning flywheel, applied to road *quality* on roads no OEM maps, with a DePIN incentive Tesla doesn't have.
3. **"We deliver Mobileye-class FCW and Bosch-class surface analytics at a fraction of the BOM, with no drive-by-wire."** Advisory-only is a feature, not a limitation — it ships today on any vehicle.

---

## 11. Open Questions (resolve before implementation)
- [ ] `τ_spike` and `Δt_baseline` defaults — tune from logged SQLite traces (Module 2) before fixing constants.
- [ ] Curve grip assumption `μ` — single conservative constant for Phase 6, or per-surface from future road-state module?
- [ ] OBD-II PID set & poll rate — confirm against target test vehicle's supported PIDs.
- [ ] Cabin-DMS model — license a pre-trained PERCLOS model or train on an Indian-cabin dataset?
- [ ] Edge TTC — does the `Coordinator` frame budget absorb the extra differencing pass, or does TTC run on a decimated stream (e.g. every 2nd frame)?

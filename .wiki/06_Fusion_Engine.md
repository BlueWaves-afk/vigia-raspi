# Fusion Engine

`FusionEngine` combines four confidence signals into a single `finalConfidence`
score (`rri`) that gates whether an observation becomes a `HazardObservation`.

## Input Signals

| Field | Source | Weight |
|-------|--------|--------|
| `yoloConfidence` | YOLO model output | 0.35 |
| `depressionScore + roughness` → `geometryConfidence` | MiDaS depth | 0.25 |
| `persistence + stability` → `temporalConfidence` | `TemporalAnalyzer` | 0.15 |
| `imuIss` (ISS motion gate) | BNO085 via SensorBridge | 0.25 |

Weights sum to 1.0.

## Geometry Confidence

```
geometryConfidence = clamp01(depression × exp(-roughness × 10))
```

- Depression normalized to [0, 1].
- Exponential roughness penalty suppresses noisy surfaces that look like
  potholes in depth but are road texture or gravel.

## Temporal Confidence

```
temporalConfidence = clamp01(0.5 × tanh(persistence × 0.1) +
                             0.5 × tanh(stability × 0.01))
```

- `tanh` squashes unbounded EMA accumulators to [0, 1].
- Rewards signals that persist across frames and have low variance.

## ISS Motion Gate

```
issContrib = (speedMs < 1.0) ? 0.0 : clamp01(imuIss)
```

When vehicle speed is below 1 m/s (parked, idling), the IMU contribution is
zeroed. This prevents engine-idle vibration or door slams from inflating scores.

## GPS Passthrough

`FusionOutput.latitude/longitude/speedMs/gpsValid` are set directly from
`FusionInput` when `gpsValid` is true. `FusionEngine` performs no coordinate
transformation — it trusts the caller's GPS validity flag.

## Two-Phase Fusion in Coordinator

The Coordinator runs fusion **twice** for potholes detected on MiDaS-strided frames:

1. **YOLO-only** (processLoop, synchronous): geometry and temporal are zero.
   Published immediately for low-latency monitoring.
2. **Full fusion** (midasLoop, async): all four signals available.
   Result is submitted to `EventStore` for upload. This is the authoritative score.

Backlinks: [[04_Coordinator_And_Threads]] | [[05_Event_Pipeline]] | [[00_Index]]

# Coordinator and Thread Architecture

## Responsibility

`Coordinator` is the central orchestrator. It owns the camera frame pipeline,
routes work to inference agents, fuses results, and forwards detections to the
event pipeline. It does **not** own the sensor hardware — that is delegated to
`SensorBridge`.

## Thread Topology

```
┌─────────────────────────────────────────────────────────────────────┐
│  Coordinator                                                        │
│                                                                     │
│  captureLoop (Core 0)    processLoop (Core 1)    midasLoop (Core 2) │
│       │                        │                       │            │
│  perception_                   │                 analytical_        │
│  .captureFrame()        YOLO runInference()      .runInference()    │
│       │                        │                       │            │
│  frameBuffer_[]     querySensors()               temporal_.update() │
│  (ring, 4 slots)    (→ SensorBridge)             FusionEngine.fuse()│
│       │                        │ push to               │            │
│       │                 midasQueue_ ───────────────────┘            │
│       │                        │                                    │
│       └────────────────────────┘                                    │
└─────────────────────────────────────────────────────────────────────┘
```

### captureLoop (Core 0)
- Calls `PerceptionAgent::captureFrame()` — blocks until a new frame is ready.
- Clones the frame outside the mutex to prevent blocking `processLoop`.
- Stores in a 4-slot circular `frameBuffer_[]`; increments `frameIndex_`.
- Exits cleanly on camera disconnect (`captureFrame()` returns false) or exception.

### processLoop (Core 1)
- Runs at target FPS (default configurable); regulated by `frameLimiter()`.
- Reads the latest frame from `frameBuffer_[]` under `bufferMutex_`.
- Calls `querySensors()` once per frame — atomic snapshot of `SensorBridge` state.
- Runs YOLO (`PerceptionAgent::runInference()`).
- If `frameIndex_ % midasStride_ == 0`, pushes a `MidasWork` packet to `midasQueue_`.
- Also performs YOLO-only baseline fusion directly (no MiDaS geometry yet).

### midasLoop (Core 2)
- Blocks on `midasQueue_.wait_and_pop()` — only wakes when processLoop pushes work.
- Runs MiDaS (`AnalyticalAgent::runInference()`), scales ROI, extracts depth.
- Calls `TemporalAnalyzer::update()` to compute persistence/stability metrics.
- Calls `FusionEngine::fuse()` with full YOLO + geometry + temporal + IMU-ISS input.
- On detection, fills `HazardObservation` and submits to `EventStore::promoter()`.

## Adaptive Throttle (Thermal + Load)

`adaptiveControl(elapsedMs)` is called every process-loop iteration:

| Condition | `midasStride_` |
|-----------|----------------|
| Temp > 85 °C | 5 (run MiDaS every 5th frame) |
| Temp > 75 °C | 3 |
| Frame time > 150 ms (< ~7 FPS) | Ramp +1 up to 5 |
| Otherwise | Ramp -1 down to 1 |

Temperature is read from sysfs at most once per second to avoid I/O overhead.

## Sensor Snapshot

`querySensors()` is called once per frame from processLoop. It calls
`SensorProcessor::process()` to fuse the latest `SensorBridge::state()` into a
flat `SensorSnapshot` struct (no heap allocation). The snapshot is also embedded
in each `MidasWork` item so the async midasLoop uses sensor data from the
*same frame* the depth was triggered on, not current state.

## Exception Safety

All three loops are wrapped in `try/catch(std::exception&)` and `catch(...)`.
Any unhandled exception sets `running_ = false` and logs to stderr, causing the
other loops to drain naturally on their next iteration check.

Backlinks: [[01_Hardware_Constraints]] | [[05_Event_Pipeline]] | [[06_Fusion_Engine]] | [[00_Index]]

# Architecture Steering — VIGIA-ARM

## Pipeline Invariants (Do Not Break)
1. **YOLO runs every frame.** MiDaS runs at stride N (1–5). This is non-negotiable — YOLO is the primary detection signal.
2. **MiDaS is always FP32.** INT8 MiDaS produces a uniform black depth map (dynamic range collapse). See `docs/int8_midas_failure.md`.
3. **RRI threshold = 0.75.** Lowering this increases false positives from shadows/debris. Raising it misses genuine hazards. Change only with experimental evidence.
4. **Zero heap allocation in the inference hot path.** `inputTensor_` and `chwBuffer_` are pre-allocated at model load time. Do not add `new`/`malloc` inside `runInference()`.
5. **Shared `ov::Core`.** Both `PerceptionAgent` and `AnalyticalAgent` accept `ov::Core&` in their constructors. Never construct a second core.

## Coordinator Responsibilities
- Frame dispatch from ring buffer (`frameBuffer_`, size 4)
- Thermal monitoring (sysfs read throttled to 1Hz via `cachedTemp_`)
- Adaptive MiDaS stride (1/3/5 based on temp thresholds 75°C/85°C)
- `publishResult()` — currently stdout only; extend here for telemetry

## InstrumentationBus Contract
- `beginFrame(idx, frame)` — opens a slot. Must be called before `storeDetections`.
- `storeDetections(idx, dets)` — stores YOLO output. Does NOT finalize.
- `recordDepth(idx, depth)` — stores MiDaS output. Does NOT finalize.
- `recordFusion(idx, input, output)` — finalizes slot when all potholes are fused.
- `notifyProcessingComplete(idx)` — finalizes no-pothole or fully-fused slots.
- Slots time out after 100ms via `tryPopFrame()` — bounding boxes are always drawn.

## Thermal Tiers
| State | Threshold | MiDaS Stride | YOLO |
|---|---|---|---|
| Normal | <75°C | 1 | Every frame |
| Warm | 75–85°C | 3 | Every frame |
| Critical | >85°C | 5 | Every frame |

## Frame Buffer
- Size: 4 slots (`FRAME_BUFFER_SIZE`)
- Protected by `bufferMutex_` (std::mutex)
- `captureLoop` writes; `processLoop` reads latest frame
- Frames are `.clone()`d into the buffer — the clone cost (~0.9ms) is negligible vs YOLO (~83ms)

## Known Bottlenecks (in priority order)
1. **MiDaS FP32 at 525ms** — dominant compute constraint. Sub-200ms MiDaS is the primary future target.
2. **`cv::imshow` + VNC** — 3.4× throughput degradation. Production must be headless.
3. **`bufferMutex_` contention** — low (capture vs process at different cadences), but a lock-free ring buffer would eliminate it entirely.
4. **`publishResult()` stdout** — `std::cout` with `<<` chains holds a lock. Replace with a lock-free telemetry sink for production.
5. **`computeDepthResiduals()` double-precision** — uses `double` for plane fitting on a CPU-only device. Converting to `float` saves ~15% on this function.

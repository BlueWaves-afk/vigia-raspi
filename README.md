# VIGIA: Physics-Aware Pothole Detection for Edge AI

## Overview
- Realtime, edge-optimized pothole detection that fuses vision, monocular depth, geometry, and temporal reasoning.
- Robust against false positives by interpreting 3D structure, surface deviations, and persistence over time.

## System Architecture
```
Camera Frame
     |
     v
+---------------+
| Perception    |  (YOLO)
| Agent         |
| ------------- |
| Bounding Box  |
| Confidence    |
+------+--------+
       | ROI
       v
+---------------+
| Analytical    |  (MiDaS + Geometry)
| Agent         |
| ------------- |
| Depth Map     |
| Plane Fit     |
| Residuals     |
| Depression    |
| Roughness     |
+------+--------+
       |
       v
+---------------+
| Temporal      |
| Analyzer      |
| ------------- |
| Persistence   |
| Stability     |
+------+--------+
       |
       v
+---------------+
| Fusion        |
| Engine        |
| ------------- |
| Final Score   |
| Confidence    |
+---------------+
```

## Core Design Principles
- **Physics-aware vision**: Depth maps feed geometric checks, using a planar road assumption to surface real depressions.
- **Temporal reasoning**: Sliding windows ensure true potholes persist across frames while noise fades out.
- **Explainable fusion**: Each intermediate signal carries explicit weights, avoiding black-box classifications.
- **Edge-first execution**: OpenVINO inference, fixed buffers, and CPU-friendly paths keep performance deterministic.

## Module Breakdown
| Module | Responsibilities |
| --- | --- |
| `perception/` | YOLO inference that outputs bounding boxes and detection confidence. |
| `analytical/` | MiDaS depth inference, ROI extraction, plane fitting, and metrics such as `depressionScore` and `roughness`. |
| `temporal/` | Maintains sliding-window history, persistence scoring, and stability estimates; fully unit tested. |
| `fusion/` | Blends YOLO, geometry, and temporal confidences into a final detection score. |
| `coordinator/` *(planned)* | Manages threading, FPS limiting, thermal awareness, and agent scheduling. |

## Folder Reference
### include/
- [include/analytical.hpp](include/analytical.hpp) – Declares the AnalyticalAgent API and supporting geometry metrics shared across modules.
- [include/coordinator.hpp](include/coordinator.hpp) – Interface contract for the pipeline coordinator orchestrating agents and timing.
- [include/fusion.hpp](include/fusion.hpp) – FusionEngine signatures and data structures for combining YOLO, depth, and temporal evidence.
- [include/perception.hpp](include/perception.hpp) – PerceptionAgent interface exposing OpenVINO-based YOLO inference routines.
- [include/roi_utils.hpp](include/roi_utils.hpp) – Helper utilities for safely clamping detection ROIs before depth analysis.
- [include/safe_queue.hpp](include/safe_queue.hpp) – Thread-safe queue abstraction used for cross-agent communication.
- [include/temporal.hpp](include/temporal.hpp) – TemporalAnalyzer definitions covering persistence scoring and stability tracking.

### models/
- [models/midasv21/openvino_midas_v21_small_256.xml](models/midasv21/openvino_midas_v21_small_256.xml) – MiDaS depth network graph compiled for OpenVINO.
- [models/midasv21/openvino_midas_v21_small_256.bin](models/midasv21/openvino_midas_v21_small_256.bin) – Binary weights associated with the MiDaS depth graph.
- [models/yolo26/yolo26_model.xml](models/yolo26/yolo26_model.xml) – YOLOv2.6 detection network graph optimized for edge inference.
- [models/yolo26/yolo26_model.bin](models/yolo26/yolo26_model.bin) – Weight file paired with the YOLO graph for perception inference.
- [models/yolo26/metadata.yaml](models/yolo26/metadata.yaml) – Model metadata describing label set, preprocessing, and postprocessing parameters.

### src/
- [src/analytical.cpp](src/analytical.cpp) – Implements MiDaS inference, plane fitting, and depression/roughness calculations feeding fusion.
- [src/coordinator.cpp](src/coordinator.cpp) – Coordinates perception, analytical, temporal, and fusion agents while managing frame cadence.
- [src/fusion.cpp](src/fusion.cpp) – Applies weighted fusion logic to generate final pothole confidences and hazard flags.
- [src/main.cpp](src/main.cpp) – Entry point wiring the coordinator with configured agents for end-to-end execution.
- [src/perception.cpp](src/perception.cpp) – Runs YOLO inference, filters detections, and publishes pothole candidates to downstream stages.
- [src/temporal.cpp](src/temporal.cpp) – Maintains temporal buffers and persistence scoring to suppress transient noise.

### tests/
- [tests/analytical_test.cpp](tests/analytical_test.cpp) – Unit tests validating depth metrics, plane fitting, and depression scoring.
- [tests/coordinator_test.cpp](tests/coordinator_test.cpp) – Harness exercising coordinator scheduling, queue handoff, and rate limiting.
- [tests/fusion_test.cpp](tests/fusion_test.cpp) – Covers fusion weighting, thresholding, and hazard decision logic.
- [tests/perception_test.cpp](tests/perception_test.cpp) – Checks YOLO inference configuration and detection filtering behavior.
- [tests/perception_video_test.cpp](tests/perception_video_test.cpp) – CLI utility for playing back videos through the perception stack for inspection.
- [tests/system_visual_test.cpp](tests/system_visual_test.cpp) – End-to-end visualization dashboard combining detection, depth, insights, and event log panels.
- [tests/temporal_test.cpp](tests/temporal_test.cpp) – Ensures temporal persistence calculations and decay curves behave as expected.

## Robustness Playbook
| Failure Mode | Mitigation |
| --- | --- |
| Shadows | Depth checks and plane fitting reject false depressions. |
| Paint or stains | Requires matching depth depression before flagging. |
| Camera shake | Temporal stability filtering mitigates jitter. |
| False YOLO detections | Fusion re-weights scores using geometric confidence. |
| Flickering noise | Persistence filtering removes non-persistent signals. |

## Edge Optimization Strategy
- Single-batch inference with fixed tensor layouts.
- OpenVINO affinity hints pin CPU execution for predictability.
- Temporal buffering avoids redundant depth inference.
- Lightweight fusion math keeps latency low.

## Current Status
- [x] Perception Agent
- [x] Analytical Agent
- [x] Temporal Analyzer
- [x] Fusion Engine
- [ ] Coordinator (in progress)

## Why VIGIA Stands Out
- Goes beyond object detection by reasoning in 3D and time.
- Transparent fusion logic with interpretable signals.
- Built for real-world roads and edge hardware constraints.

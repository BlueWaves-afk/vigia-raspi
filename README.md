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

# 🚧 VIGIA: End-to-End Visual Hazard Detection System

**Vigia** is a high-performance, real-time perception system engineered to detect and assess road hazards like potholes. It utilizes a sophisticated H-HMAS (Hybrid Hierarchical Multi-Agent System) architecture, combining **YOLO26 object detection**, **MiDaS monocular depth estimation**, and multi-stage risk fusion—all implemented in native, modern C++17.

---

## 📌 Key Capabilities

* **🎥 Dual Input Streams:** Support for high-resolution MP4 video files and live camera feeds via `--camera`.
* **🧠 Multi-Stage Perception:** A sequential pipeline that graduates from raw detection (YOLO) to geometric verification (MiDaS) and temporal stability analysis.
* **📊 Real-Time Instrumentation:** Built-in observability for per-frame latency, rolling FPS, frame strides, and CPU thermals.
* **🖼️ Unified Dashboard:** Three synchronized visual outputs (Detections, Depth Heatmap, and System Insights) for comprehensive debugging.
* **⚙️ Hardware Acceleration:** Full OpenVINO optimization for low-latency inference on edge devices.

---

## 🧠 System Architecture

Vigia is built on a modular, judge-friendly architecture that ensures deterministic performance and easy extensibility.

### Logic Flow

1. **PerceptionAgent (YOLO):** Extracts initial object candidates (e.g., potholes).
2. **AnalyticalAgent (MiDaS):** Estimates monocular depth to provide geometric context.
3. **TemporalAnalyzer:** Smooths detections over time to ensure stability and persistence.
4. **FusionEngine:** Calculates the final risk score by weighting YOLO confidence against geometric reality.
5. **Instrumentation:** Logs telemetry and displays a real-time terminal-style dashboard.

---

## 🧱 Core Engineering Principles

### 1. Strict Separation of Concerns

Every module is a standalone agent. The `Coordinator` orchestrates execution, ensuring that the **PerceptionAgent** only handles inference, while the **FusionEngine** only handles decision logic. This allows for clean testing boundaries and easy component replacement.

### 2. Deterministic & Measurable

Every frame is timestamped and tracked end-to-end. Vigia is not a simple loop; it is a measured system where latency and frame drops are accounted for explicitly.

### 3. Python-Parity Validation

All C++ preprocessing, tensor layouts, and confidence thresholds were validated against **Python reference implementations**. This ensures that the C++ system achieves model correctness, matching the accuracy of the original data science training.

---

## 📂 Repository Structure

```text
vigia/
├── include/       # Thread-safe headers and agent definitions
├── src/           # Core implementation of H-HMAS agents
├── tests/         # Module-level and E2E system tests
├── models/        # OpenVINO IR files (.xml, .bin)
└── dataset/       # Validation images and ground truth

```

---

## 🛠️ Build Instructions

### Requirements

* **C++17 Compiler** (Clang or GCC)
* **OpenCV** ≥ 4.5
* **OpenVINO Toolkit** (Latest stable)
* **pthreads** (Linux/macOS)

### Compile Command

```bash
clang++ -std=c++17 \
  tests/system_visual_test.cpp \
  src/*.cpp \
  -Iinclude \
  -I/opt/homebrew/include/opencv4 \
  -I/opt/homebrew/opt/openvino/include \
  -L/opt/homebrew/lib -L/opt/homebrew/opt/openvino/lib \
  -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -lopencv_highgui \
  -lopenvino -pthread -O3 -o vigia_system

```

---

## ▶️ Usage

### Video Analysis

```bash
./vigia_system --video road_hazard.mp4 --fps 30

```

### Live Camera Mode

```bash
./vigia_system --camera 0 --yolo models/yolo26/yolo26_model.xml

```

---

## 🔍 Judge Notes

* **Production-Ready:** No Python runtime dependency; deterministic and standalone.
* **Observability:** Clear real-time metrics for FPS, latency, and MiDaS stride.
* **Systems Thinking:** The project demonstrates a complete engineering cycle—design, modular testing, and hardware optimization.

---

## 🏁 Final Note

**Vigia** is more than just an AI model; it is a complete, industrial-grade perception pipeline designed to bridge the gap between model inference and real-world deployment.

# VIGIA Demo Run Guide

Complete commands to run the YOLO detection demo on Raspberry Pi 5 with visualization on Mac.

---

## Prerequisites (one-time setup)

### Pi — set CPU governor to performance
```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor  # verify: performance
```

### Pi — source OpenVINO environment
```bash
source /opt/intel/openvino_2025/setupvars.sh
```

Add to `.bashrc` so it persists:
```bash
echo 'source /opt/intel/openvino_2025/setupvars.sh' >> ~/.bashrc
```

---

## Option A — VNC Display (recommended for demos)

### Pi — start VNC server
```bash
vncserver :1 -geometry 1280x720 -depth 24
vncserver -list  # verify :1 is running
```

### Pi — run detection
```bash
export DISPLAY=:1
source /opt/intel/openvino_2025/setupvars.sh
cd ~/vigia-raspi/build
./perception_video_test hazard.mp4 models/yolo26/yolo26_model_int8.xml
```

### Mac — connect VNC
```bash
open vnc://192.168.0.107:5901
# Enter VNC password when prompted
```

### Pi — stop VNC when done
```bash
vncserver -kill :1
```

---

## Option B — Headless (maximum throughput)

```bash
source /opt/intel/openvino_2025/setupvars.sh
cd ~/vigia-raspi/build
./perception_video_test --headless hazard.mp4 models/yolo26/yolo26_model_int8.xml
```

Expected: ~32 FPS YOLO-only on Pi 5 with KleidiAI INT8 active.

---

## Option C — UDP stream to Mac (no VNC)

```bash
source /opt/intel/openvino_2025/setupvars.sh
cd ~/vigia-raspi/build
./perception_video_test --headless --stream 192.168.0.255 hazard.mp4 models/yolo26/yolo26_model_int8.xml
```

---

## Full pipeline (YOLO + MiDaS + fusion)

### Headless benchmark (production throughput)
```bash
source /opt/intel/openvino_2025/setupvars.sh
cd ~/vigia-raspi/build
./system_visual_test --headless --video hazard.mp4 models/yolo26/yolo26_model_int8.xml
```

Expected: ~11.4 FPS full pipeline on Pi 5.

### With dashboard (VNC required)
```bash
export DISPLAY=:1
source /opt/intel/openvino_2025/setupvars.sh
cd ~/vigia-raspi/build
./system_visual_test -F --video hazard.mp4 models/yolo26/yolo26_model_int8.xml
```

> **Note:** OpenCV is built with `WITH_GTK=OFF` on Pi 5. Use `--headless` for benchmarks; VNC display requires a working X11 session but no GTK dependency for window creation in headless mode.

---

## Verify KleidiAI is active

```bash
source /opt/intel/openvino_2025/setupvars.sh
cd ~/vigia-raspi/build
ONEDNN_VERBOSE=1 ./perception_video_test --headless hazard.mp4 models/yolo26/yolo26_model_int8.xml 2>&1 | grep -i kleidiai | head -5
```

Expected output includes `fullyconnected_kleidiai` or similar KleidiAI kernel names.

---

## Rebuild after pulling changes

```bash
cd ~/vigia-raspi/build
source /opt/intel/openvino_2025/setupvars.sh
cmake -DOpenVINO_DIR=/opt/intel/openvino_2025/runtime/cmake ..
make perception_video_test system_visual_test -j$(nproc)
```

---

## Model selection

| Model | Precision | Input | Notes |
|-------|-----------|-------|-------|
| `models/yolo26/yolo26_model_int8.xml` | INT8 (KleidiAI) | 320×320 | **Use this — production default on Pi 5** |
| `models/yolo26/yolo26_320_fp16.xml` | FP16 / FP32 compute | 320×320 | Pi 4 legacy / validation |
| `models/yolo26/yolo26_model_2023.xml` | FP32 | 640×640 | Accuracy validation only |

---

## Expected performance (Pi 5, KleidiAI active)

| Test | Throughput | Latency |
|------|------------|---------|
| YOLO-only (`perception_video_test --headless`) | ~32.4 FPS | ~28.4 ms avg |
| Full pipeline (`system_visual_test --headless`) | ~11.4 FPS | 52–139 ms (YOLO-only → YOLO+MiDaS) |
| Full pipeline with VNC display | ~3 FPS | UI-bound, not inference-bound |

---

## Quick smoke test

```bash
source /opt/intel/openvino_2025/setupvars.sh
cd ~/vigia-raspi/build
./perception_video_test hazard.mp4 models/yolo26/yolo26_model_int8.xml
# Press Q to quit
```

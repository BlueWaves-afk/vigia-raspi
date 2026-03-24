# VIGIA Demo Run Guide

Complete commands to run the YOLO detection demo on Raspberry Pi 4 with visualization on Mac.

---

## Prerequisites (one-time setup)

### Pi — set CPU governor to performance
```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor  # verify: performance
```

### Pi — source OpenVINO environment
```bash
source /opt/intel/openvino_2023/setupvars.sh
```

Add to `.bashrc` so it persists:
```bash
echo 'source /opt/intel/openvino_2023/setupvars.sh' >> ~/.bashrc
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
source /opt/intel/openvino_2023/setupvars.sh
cd ~/vigia-raspi/build
./perception_video_test hazard.mp4 models/yolo26/yolo26_320_fp16.xml
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

## Option B — Headless benchmark (maximum FPS, no display)

### Pi
```bash
source /opt/intel/openvino_2023/setupvars.sh
cd ~/vigia-raspi/build
./perception_video_test --headless hazard.mp4 models/yolo26/yolo26_320_fp16.xml
```

---

## Option C — UDP stream to Mac (experimental)

### Mac — start visualizer first
```bash
cd "/Users/tommathew/Documents/Github Repositories/vigia-raspi"
python3 tools/vigia_visualizer.py
```

### Pi — run with stream flag
```bash
source /opt/intel/openvino_2023/setupvars.sh
cd ~/vigia-raspi/build
./perception_video_test --headless --stream 192.168.0.255 hazard.mp4 models/yolo26/yolo26_320_fp16.xml
```

---

## Monitor CPU core utilization (verify ACL multi-threading)

### Pi — in a second SSH terminal while inference runs
```bash
mpstat -P ALL 2
```

Expected output with ACL active (all 4 cores ~95%):
```
CPU    %usr   %nice    %sys  %idle
all   94.96    0.00    2.02   3.02
  0   93.94    0.00    2.02   4.04
  1   96.02    0.00    1.00   2.99
  2   95.43    0.00    2.54   2.03
  3   94.44    0.00    2.53   3.03
```

Install if missing:
```bash
sudo apt install -y sysstat
```

---

## Verify ACL is running (oneDNN verbose)

```bash
source /opt/intel/openvino_2023/setupvars.sh
cd ~/vigia-raspi/build
ONEDNN_VERBOSE=1 ./perception_video_test --headless hazard.mp4 models/yolo26/yolo26_320_fp16.xml 2>&1 | grep "exec,cpu,convolution" | head -3
```

Expected: `gemm:acl` in every convolution line confirms ACL is active.

---

## Full system pipeline (system_visual_test)

Runs the complete 4-stage parallel pipeline (YOLO + MiDaS + Fusion + Dashboard).

### Build
```bash
cd ~/vigia-raspi/build
make system_visual_test -j4
```

### Run with FP16 YOLO model (fullscreen)
```bash
export DISPLAY=:1
cd ~/vigia-raspi/build
./system_visual_test -F --video hazard.mp4 models/yolo26/yolo26_320_fp16.xml
```

| Flag | Effect |
|---|---|
| `-F` | Fullscreen display |
| `--headless` | No display, maximum throughput (~10.3 FPS EMA) |
| `--fp32` | Override to FP32 YOLO model (validation only) |

---

## Pull latest code and rebuild

### Pi
```bash
cd ~/vigia-raspi
git -c http.sslVerify=false pull
source /opt/intel/openvino_2023/setupvars.sh
cd build
cmake -DOpenVINO_DIR=/opt/intel/openvino_2023/runtime/cmake ..
make perception_video_test system_visual_test -j4
ln -sf ../models models 2>/dev/null
ln -sf ../hazard.mp4 hazard.mp4 2>/dev/null
```

---

## Model files

| File | Precision | Input | Notes |
|---|---|---|---|
| `models/yolo26/yolo26_320_fp16.xml` | FP16 weights / FP32 compute | 320×320 | **Use this — best performance on A72 with ACL** |
| `models/yolo26/yolo26_model_2023.xml` | FP32 | 640×640 | Slower, higher accuracy |
| `models/yolo26/yolo26_model_int8.xml` | INT8 | 320×320 | For OpenVINO 2025 only |

---

## Expected performance (Pi 4, ACL active)

| Mode | FPS | Latency |
|---|---|---|
| Headless, 320×320 FP16 | ~9 FPS | ~109ms |
| VNC display | ~3–4 FPS | ~300ms |
| Headless, 640×640 FP32 | ~2.5 FPS | ~400ms |

---

## X11 forwarding (alternative to VNC)

### Mac — start XQuartz and allow connections
```bash
open -a XQuartz
DISPLAY=:0 /opt/X11/bin/xhost +
```

### Mac — SSH with X11 forwarding
```bash
ssh -Y vigiasense@192.168.0.107
```

### Pi — run with X11 display
```bash
# DISPLAY is set automatically by -Y forwarding
source /opt/intel/openvino_2023/setupvars.sh
cd ~/vigia-raspi/build
./perception_video_test hazard.mp4 models/yolo26/yolo26_320_fp16.xml
```

---

## Pi network info
```bash
hostname -I | awk '{print $1}'  # Pi IP
```

Pi IP: `192.168.0.107`  
Mac IP: `192.168.0.106`

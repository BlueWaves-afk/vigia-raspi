# Vigia-Raspi — Session Handoff (2026-06-18)

## Branch
`claude/great-jepsen-4d776c` (worktree at `.claude/worktrees/great-jepsen-4d776c`)

---

## What Was Built This Session

### New Files (untracked — must commit)

| File | Purpose |
|---|---|
| `docs/app_dashcam_integration.md` | Full Pi↔Android BLE design spec — 6 ratified decisions, 12-step impl plan |
| `vigia_ws/src/vigia_edge_node/include/vigia_edge_node/ble_frame_codec.hpp` | Wire encoder/decoder matching Android `BleDataStreamerImpl.kt` exactly |
| `vigia_ws/src/vigia_edge_node/include/vigia_edge_node/ble_gatt_constants.hpp` | Pi-side GATT UUIDs/opcodes mirroring Android `GattConstants.kt` |
| `vigia_ws/src/vigia_edge_node/include/vigia_edge_node/vigia_rri.hpp` | Shared RRI computation with `RriInputs` presence flags (degraded-mode aware) |
| `vigia_ws/src/vigia_edge_node/test/test_ble_frame_codec.cpp` | Host-compilable unit tests (8 cases) — **passed on Cortex-A76** |
| `vigia_ws/src/vigia_edge_node/test/test_vigia_rri.cpp` | RRI unit tests (5 cases) — **passed on Cortex-A76** |
| `models/yolo26/vigia_v26n_int8_convonly.onnx` | INT8 conv-only YOLO model (2.9 MB) transferred from training machine |

### Modified Files

| File | Change |
|---|---|
| `vigia_ws/src/vigia_msgs/msg/HazardEvent.msg` | Added `degraded bool` + `rri_tier uint8` fields |
| `vigia_ws/src/vigia_edge_node/src/fusion_node.cpp` | IMU now optional; uses `vigia_rri.hpp::compute_rri()` |
| `vigia_ws/src/vigia_edge_node/src/anti_death_node.cpp` | Removed geohash from signed JSON (D5); fixed pynacl comment |
| `vigia_ws/src/vigia_edge_node/CMakeLists.txt` | Added `BUILD_TESTING` block for unit tests |

---

## Pi State (100.114.1.98 / vigiasense)

### ROS2 Jazzy Build
- **Status:** RUNNING (PID 1939, `~/ros2_build13.log`)
- **Root cause fixed:** `eProsima/Fast-DDS` was on `3.1.x`; checked out `2.14.x` to match Jazzy pin
- **When done:** `~/ros2_jazzy/install/lib/rclcpp/` will exist
- **Check:** `ssh vigiasense@100.114.1.98 "ls ~/ros2_jazzy/install/lib/rclcpp/ 2>/dev/null && echo DONE || pgrep colcon"`

### After ROS2 Completes (in order — do NOT overlap, 4-core Pi)

**Step 1 — Build vigia packages:**
```bash
ssh vigiasense@100.114.1.98
source ~/ros2_jazzy/install/setup.bash
cd ~/vigia_ws
colcon build --packages-select vigia_msgs vigia_edge_node
```

**Step 2 — ORT ACL+KleidiAI rebuild** (after colcon succeeds):
```bash
cd ~/ort-src
nohup ./build.sh \
  --config Release \
  --use_acl --use_xnnpack \
  --skip_tests \
  --cmake_extra_defines \
    onnxruntime_USE_ACL_HOME=$HOME/acl-src \
    CMAKE_INSTALL_PREFIX=/opt/onnxruntime-kleidiai \
  --parallel $(nproc) \
  > ~/ort_kleidiai_build.log 2>&1 &
```

### ONNX Runtime State
- `/opt/onnxruntime-kleidiai` — **incomplete** (headers + `libonnxruntime_providers_shared.so` only, no `libonnxruntime.so`)
- `/opt/onnxruntime-linux-aarch64-1.20.1/` — **working CPU-EP fallback** (what the CMake finds now)

### Workspace on Pi
- Staged at `~/vigia_ws/src/` (rsynced 2026-06-18 with all new BLE/RRI headers)
- SSH: `vigiasense@100.114.1.98`, Tailscale VPN, passwordless sudo via `/etc/sudoers.d/vigiasense-nopasswd`

---

## Key Architecture Decisions (Ratified)

| ID | Decision |
|---|---|
| D1 | BLE GATT server: C++ with `sdbus-c++` (BlueZ D-Bus) |
| D2 | Auth: ECDH P-256 mutual auth (NOT symmetric HMAC — StrongBox can't export key) |
| D3 | Provisioning: QR label on Pi hardware → CompanionDeviceManager auto-bond |
| D4 | Default telemetry: 256-D latent (Dynamic Dims Scaling: 512↔256↔RRI-only) |
| D5 | Geohash dropped from signed message (server recomputes from GPS coords) |
| D6 | New 128-bit UUIDs for all GATT characteristics |

**Fatal flaw fixed:** Android `KeystoreManager` used `PURPOSE_SIGN` (non-exportable) → HMAC impossible. Fix requires P-256 `PURPOSE_AGREE_KEY` on Android side.

---

## Production Architecture Decisions

- **AWS (not M7) for production telemetry** — AWS uses Ed25519 device-held keys (asymmetric DePIN trust). M7 uses symmetric HMAC (server can forge). M7 = offline/LAN demo mirror only.
- **2% VLM sampling** — NOT implemented yet. Production: cheap policy gate always, VLM-verify ~2% + all Discovery events. Wire in `vigia-amazon` `OrchestratorFunction`.

---

## What Remains (Blocked / Not Started)

| Task | Blocker |
|---|---|
| `BleGattNode` implementation (~500 lines C++) | Needs Pi's BlueZ + `sdbus-c++` installed |
| ECDH handshake (Pi side: mbedTLS P-256) | Android coordination needed first |
| Anti-spoof rolling beacon | ATECC608A wiring by Ben (GP2=SDA, GP3=SCL) |
| YOLO `latent_layer_name` identification | Export model, inspect with Netron |
| `HazardUplinkNode` (continuous Ed25519→AWS) | Replaces AntiDeathNode emergency-only path |
| 2% VLM sampling in vigia-amazon | vigia-amazon OrchestratorFunction |
| PREEMPT_RT boot | Physical Pi access to update `/boot/firmware/config.txt` |

---

## Pending Commit

All changes above are uncommitted. To create the PR:
```bash
cd /Users/tommathew/Documents/Github\ Repositories/vigia-raspi/.claude/worktrees/great-jepsen-4d776c
git add docs/app_dashcam_integration.md docs/HANDOFF.md \
  vigia_ws/src/vigia_edge_node/include/vigia_edge_node/ \
  vigia_ws/src/vigia_edge_node/test/ \
  vigia_ws/src/vigia_msgs/msg/HazardEvent.msg \
  vigia_ws/src/vigia_edge_node/src/fusion_node.cpp \
  vigia_ws/src/vigia_edge_node/src/anti_death_node.cpp \
  vigia_ws/src/vigia_edge_node/CMakeLists.txt \
  models/yolo26/vigia_v26n_int8_convonly.onnx
git commit -m "M8: BLE GATT codec, RRI degraded mode, design spec, INT8 model"
gh pr create --title "M8: BLE transport layer, degraded-mode RRI, app integration spec"
```

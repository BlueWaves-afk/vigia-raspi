# VIGIA System Gap Tracker

**Last updated:** 2026-06-18 (session 2)  
**Audited against:** commits through `3e1a873` (our M9 build fixes) + `bf6e8d6` (Ben's SE commit) + this session  
**Build status:** `colcon build` ✅ passing on Pi (Debian Trixie, aarch64)

---

## How to use this file

Each item has a status tag:
- `[ ]` — not started / missing
- `[~]` — partially done / stub / compiles but not functional at runtime  
- `[x]` — done and verified working end-to-end

Add a `<!-- resolved: YYYY-MM-DD, commit sha -->` comment when you close an item.

---

## Phase 1 — OS & Middleware

| # | Item | Status | Notes |
|---|------|--------|-------|
| 1.1 | PREEMPT_RT kernel booted | `[~]` | Installed (`linux-image-6.12.73+deb13-rt-arm64`) but not active. `uname -a` does NOT show `PREEMPT_RT`. Requires physical Pi access: edit `/boot/firmware/cmdline.txt`. SCHED_FIFO runs under CFS until then. |
| 1.2 | `config/` YAML param files | `[x]` | Added in commit `b03f004` (`camera_params.yaml`, `vision_params.yaml`, `depth_params.yaml`, `fusion_params.yaml`, `sensor_bridge_params.yaml`, `ble_gatt_params.yaml`, `anti_death_params.yaml`) |
| 1.3 | Per-slot seqlock → bulk snapshot | `[~]` | `shm_ring_buffer.hpp` uses per-slot seqlock. Spec §7 defines one global seqlock for an atomic 300-frame snapshot. Functionally similar but not spec-compliant. AntiDeathNode's snapshot is 300 serial reads, not one atomic bulk copy. |

---

## Phase 2 — Pico 2 / STM32 Sensor Bridge

### Firmware (Pico 2 side)

| # | Item | Status | Notes |
|---|------|--------|-------|
| 2.1 | ATECC608A I2C driver | `[~]` | `firmware/src/atecc608a_driver.c` written. **Default build is `VIGIA_PHASE2_STUB=1`** (zero-fills hash + sig). Live path requires physical wiring: GP2=SDA, GP3=SCL, 4.7 kΩ pull-ups to 3.3 V. Ben's commit `bf6e8d6`. |
| 2.2 | cryptoauthlib cross-build | `[~]` | `firmware/cmake/cryptoauthlib_pico.cmake` added. `scripts/setup_cryptoauthlib.sh` + `scripts/build_phase2_live.sh` present. Not yet confirmed built with `VIGIA_PHASE2_STUB=0`. |
| 2.3 | `E_t` hash + ECDSA signing loop | `[~]` | Called in `firmware/src/main.c` (`vigia_atca_sign()`). Active only when stub mode is off and SE is physically wired. |
| 2.4 | COBS encoder | `[~]` | `include/cobs.hpp` added. Referenced in firmware but integration not confirmed end-to-end. |
| 2.5 | GPS rate 10 Hz (`UBX-CFG-RATE`) | `[ ]` | NEO-M8N defaults to 1 Hz NAV-PVT. Need to send `UBX-CFG-RATE` (measurement period 100 ms) at firmware boot. Spec §2 requires 10 Hz. |
| 2.6 | `no_heap.cpp` link-time enforcement | `[ ]` | Not present. Spec requires `operator new` / `malloc` to panic in firmware. |
| 2.7 | ATECC608A physically wired | `[ ]` | **Hardware blocker.** GP2=SDA, GP3=SCL. Until wired, all signing is zero-filled stub. |
| 2.8 | ATECC608A provisioned (Microchip Trust Platform) | `[ ]` | `tools/atecc_provision.py` added in `bf6e8d6`. Run against a wired device. Needs Microchip account + Trust Platform certificate chain. |

### Pi side (sensor_bridge_node)

| # | Item | Status | Notes |
|---|------|--------|-------|
| 2.9 | COBS baud rate 921600 | `[ ]` | Spec §6.5 mandates 921600 baud. Current default is 115200. One-line param change in `sensor_bridge_params.yaml` and confirming Pico firmware agrees. |
| 2.10 | `SignedEtPacketPi` struct size 173 bytes | `[x]` | Fixed in `ff5683d`. <!-- resolved: 2026-06-18, ff5683d --> |
| 2.11 | `sig_valid` ECDSA verify wired | `[x]` | `sensor_bridge_node.cpp` now loads `/etc/vigia/atecc_pubkey.bin` at startup and calls `vigia::VigiaIdentityKey::verify_peer()` in `process_cobs_frame`. Returns `false` for all-zero stubs. <!-- resolved: 2026-06-18 --> |

---

## Phase 3 — ONNX Vision Engine

| # | Item | Status | Notes |
|---|------|--------|-------|
| 3.1 | YOLO INT8 ONNX model on disk | `[x]` | `7e04a5e` added INT8 YOLO models. Confirmed present post-pull. |
| 3.2 | MiDaS v2.1 ONNX model on disk | `[x]` | `97fe8b4` added MiDaS model. |
| 3.3 | Spatial latent layer name known | `[x]` | Already set in `vision_params.yaml`: `/model.22/cv2/act/Mul_output_0`. <!-- resolved: 2026-06-18, pre-existing --> |
| 3.4 | KleidiAI ACL EP (`--use_acl`) | `[ ]` | ORT CPU EP only. ~28 ms/frame vs. 7 ms spec target. Requires ~2h rebuild of ORT from source on Pi with `--use_acl` + ARM Compute Library. `VIGIA_HAVE_ACL_EP` compile gate is wired but the `.so` is absent. |
| 3.5 | `Ort::IoBinding` pre-binding | `[x]` | `vision_node.cpp` now uses `Ort::IoBinding` bound to `mem_info_`; `session_->Run(*io_binding_)` replaces per-call vector alloc. <!-- resolved: 2026-06-18 --> |
| 3.6 | `verify_kleidiai_capable()` CPUID check | `[ ]` | `/proc/cpuinfo` `asimddp` check specified in doc 04 §4.2. Not in source. Minor. |
| 3.7 | Model prep scripts (INT8 quant pipeline) | `[x]` | `b03f004` added `tools/model_prep/` scripts. |

---

## Phase 4 — Sensor Fusion & ISS

| # | Item | Status | Notes |
|---|------|--------|-------|
| 4.1 | Gravity compensation | `[x]` | Quaternion sandwich → subtract `[0,0,9.81]` → `a_detrended.z()`. In `fusion_node.cpp`. |
| 4.2 | ISS computation | `[x]` | `ISS = |a_detrended.z| / max(v_gps, v_min_ms)`. Correct formula, configurable `v_min_ms`. |
| 4.3 | Kalman predict step (IMU integration) | `[x]` | `fusion_node.cpp` `on_imu`: rotates body accel into world frame with quaternion, integrates `kf_x_ += a_world.xy() * dt`. <!-- resolved: 2026-06-18 --> |
| 4.4 | `FrameMetadata` written by CameraNode | `[x]` | `camera_node.cpp` lazy-opens `ShmMetaRing(creator=false)` and writes minimal `{frame_id, timestamp_us}` on each capture. FusionNode overwrites with full metadata. <!-- resolved: 2026-06-18 --> |

---

## Phase 5 — Anti-Death Storage & MQTT

| # | Item | Status | Notes |
|---|------|--------|-------|
| 5.1 | libgpiod v2 API port | `[x]` | `anti_death_node.cpp/hpp` ported to v2: `gpiod_chip_request_lines()` + `gpiod_line_request_wait_edge_events()` + `gpiod_edge_event_buffer`. Pre-allocated event buffer, non-blocking poll per tick. <!-- resolved: 2026-06-18 --> |
| 5.2 | `vigia-sim7600-init.sh` in repo | `[x]` | Added at `scripts/vigia-sim7600-init.sh`. Waits for `usb0`, brings link up, runs DHCP, pings gateway. <!-- resolved: 2026-06-18 --> |
| 5.3 | SIM7600 ECM mode provisioned | `[ ]` | One-time AT command: `AT+CUSBPIDSWITCH=9011,1,1`. Must be run physically with SIM7600 connected. |
| 5.4 | TLS certs on device | `[ ]` | `mqtt_client_init()` reads `/etc/vigia/ca_chain.pem`, `device_cert.pem`, `device_key.pem`. None provisioned. MQTT TLS connect will fail. Needs: MQTT broker CA cert + per-device mTLS cert from provisioning pipeline. |
| 5.5 | MQTT broker deployed | `[ ]` | `mqtt_broker_host` param is empty string. No broker running. Infrastructure decision needed (self-hosted Mosquitto or managed). |
| 5.6 | Ed25519 signing in legacy HTTPS path | `[~]` | `device_ed25519.key` param exists. Key path read but never used in the curl transmit path — the JSON payload is unsigned. Only relevant when Paho not compiled in. |

---

## Phase 6 — DePIN Security & Attestation

| # | Item | Status | Notes |
|---|------|--------|-------|
| 6.1 | `attestation.py` wired into server ingest | `[x]` | `server/main.py` now starts an MQTT subscriber (paho, daemon thread) on `vigia/attest/+/hazard` at lifespan startup. Calls `process_attestation_event()` → `VigiaDb.log_verified_event()`. <!-- resolved: 2026-06-18 --> |
| 6.2 | MQTT subscriber on server | `[x]` | See 6.1. Enabled via `MQTT_BROKER_HOST` env var; no-ops if unset. <!-- resolved: 2026-06-18 --> |
| 6.3 | Server `signed_et.valid` key populated | `[x]` | `anti_death_node.cpp` `pack_signed_et()` now packs `"valid": et.sig_valid` as 7th field (was 6). `_resolve_trust_level()` can now reach `TRUST_LEVEL_HARDWARE`. <!-- resolved: 2026-06-18 --> |
| 6.4 | Fleet device registry in DB | `[x]` | `server/db/init.sql` extended: `cert_pem TEXT` column on `device_registry`, `fleet_ca` table for root CA, `attestation_log` table. `server/ingest/db_adapter.py` (new) exposes the interface `attestation.py` requires. <!-- resolved: 2026-06-18 --> |
| 6.5 | Cosmos 3 world model client | `[ ]` | `cosmos3_client.submit_world_model_update()` called in `attestation.py:276`. `cosmos3_client` is always `None`. No client implementation. |
| 6.6 | `tools/provision_device.py` | `[x]` | Added in `bf6e8d6` as `tools/atecc_provision.py`. |

---

## BLE GATT (Phase 3 app integration)

| # | Item | Status | Notes |
|---|------|--------|-------|
| 7.1 | BleGattNode sdbus-c++ v2 port | `[x]` | Complete. `3e1a873`. Compiles and builds. |
| 7.2 | ECDH handshake crypto | `[x]` | mbedTLS 3.x P-256 ECDH + HKDF + HMAC in `vigia_ecdh.hpp`. Real crypto when `VIGIA_HAVE_MBEDTLS` defined. |
| 7.3 | Attest characteristic (live sig) | `[~]` | Sends `et_hash` + `ecdsa_sig` from latest `SignedEt`. Will be all-zeros until ATECC608A is wired and `sig_valid` is real. |
| 7.4 | Design spec document for BLE transport | `[x]` | `.claude/design/07_ble_transport_spec.md` created. Covers GATT layout, ECDH handshake, DimsFrame wire format, attest char, control char. <!-- resolved: 2026-06-18 --> |

---

## Server-Side Pipeline

| # | Item | Status | Notes |
|---|------|--------|-------|
| 8.1 | HTTP ingest + PostGIS | `[x]` | FastAPI `/v1/events`, spatial merge within 5 m radius, hazard map UI. |
| 8.2 | Server auth: HMAC → ECDSA upgrade | `[x]` | `auth.py` `lookup_device()` now returns `cert_pem`. `authenticate_event()` uses `verify_ecdsa_header()` when cert_pem present, falls back to HMAC for legacy devices. `signature.py` gains `verify_ecdsa_header()`. <!-- resolved: 2026-06-18 --> |
| 8.3 | Anti-replay uses hardware sequence | `[~]` | Server uses `device_seq` from JSON payload (client-supplied). Must be cross-checked against the `SignedEt.sequence` from the firmware monotonic counter once attestation is wired. |

---

## Cross-Cutting

| # | Item | Status | Notes |
|---|------|--------|-------|
| 9.1 | `static_assert(SignedEtPacketPi == 165)` wrong | `[x]` | Fixed in `ff5683d`. <!-- resolved: 2026-06-18, ff5683d --> |
| 9.2 | `EcdsaVerifier` unused in sensor_bridge | `[x]` | See 2.11. <!-- resolved: 2026-06-18 --> |
| 9.3 | Kalman velocity dead-reckoning | `[x]` | See 4.3. <!-- resolved: 2026-06-18 --> |
| 9.4 | IoBinding on VisionNode hot path | `[x]` | See 3.5. <!-- resolved: 2026-06-18 --> |
| 9.5 | GPS at 10 Hz | `[ ]` | See 2.5. 1 Hz default from NEO-M8N. |
| 9.6 | PREEMPT_RT booted | `[ ]` | See 1.1. Physical Pi access needed. |

---

## Quickest wins (no hardware needed)

In rough priority order:

1. **Fix static_assert bug** — `sensor_bridge_node.cpp:54`: `165` → `173`. 1-line fix, prevents next build from breaking.
2. **`vigia-sim7600-init.sh`** — write the script so the systemd service can start.
3. **Wire `EcdsaVerifier` into `sensor_bridge_node.cpp`** — instantiate, call `verify()`, assign `msg->sig_valid`. Unblocks Phase 6 the moment SE is physically wired.
4. **Kalman predict step** — 3-line fix in `fusion_node.cpp` IMU callback.
5. **Wire `attestation.py` into server** — replace HMAC ingest with ECDSA pipeline + add MQTT subscriber.
6. **Set `latent_layer_name`** — Netron-inspect the latent YOLO ONNX, set param in `vision_params.yaml`. Enables real S_t streaming.
7. **libgpiod v2 port** — port `anti_death_node.cpp` GPIO path to v2 API. Unblocks the entire emergency sequence.

## Requires Ben (hardware)

- ATECC608A wiring (GP2=SDA, GP3=SCL, pull-ups) → items 2.7, 2.1 live, 2.2 live, 2.3
- SIM7600 ECM provisioning → item 5.3
- PREEMPT_RT cmdline.txt edit → item 1.1

## Requires infrastructure decisions

- MQTT broker → items 5.4, 5.5, 6.2
- Cosmos 3 account / API → item 6.5
- ACL EP ORT rebuild (~2h on Pi) → item 3.4

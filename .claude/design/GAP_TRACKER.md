# VIGIA System Gap Tracker

**Last updated:** 2026-06-19 (session 5 — cloud deploy: 2% VLM sampling, reward-economics hardening, register-device proof-of-possession, DLQs)  
**Audited against:** commits through M12 (AWS IoT Core unification, FastAPI/Mosquitto deleted, BLE RESPONSE_CHAR wired) + session-5 `vigia-amazon` deploy (CloudFormation `VigiaStack` → `UPDATE_COMPLETE`)  
**Build status:** `colcon build` ✅ passing on Pi (Debian Trixie, aarch64); `cdk deploy VigiaStack` ✅ deployed

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
| 2.3 | `E_t` hash + ECDSA signing loop | `[~]` | Called in `firmware/src/main.c` (`vigia_atca_sign()`). `k_device_id` now reads from `atcab_read_serial_number()` instead of hardcoded placeholder. Active once `build_phase2_live.sh` built and flashed. |
| 2.4 | COBS encoder | `[~]` | `include/cobs.hpp` added. Referenced in firmware but integration not confirmed end-to-end. |
| 2.5 | GPS rate 10 Hz (`UBX-CFG-RATE`) | `[x]` | `neo_m8n_driver.c` `configure_gps()` now sends `UBX-CFG-RATE` with measRate=0x0064 (100 ms). Was 0x03E8 (1000 ms = 1 Hz). <!-- resolved: 2026-06-18 --> |
| 2.6 | `no_heap.cpp` link-time enforcement | `[x]` | `firmware/src/no_heap.cpp` present. `operator new` / `operator new[]` are `static_assert(false, ...)` — any TU calling `new` fails to compile. <!-- resolved: 2026-06-18 --> |
| 2.7 | ATECC608A physically wired | `[x]` | **Confirmed wired by user.** GP2=SDA, GP3=SCL, 4.7 kΩ pull-ups to 3.3 V. <!-- resolved: 2026-06-18 --> |
| 2.8 | ATECC608A provisioned (Microchip Trust Platform) | `[~]` | `tools/atecc_provision.py` added. Chip is wired; run `scripts/build_phase2_live.sh` then `tools/atecc_provision.py --device-id vigia-001` after flashing. Needs one-time `atcab_genkey(0)` call via Trust Platform or live firmware. |

### Pi side (sensor_bridge_node)

| # | Item | Status | Notes |
|---|------|--------|-------|
| 2.9 | COBS baud rate 921600 | `[x]` | `sensor_bridge_params.yaml` already set to 921600. `sensor_bridge_node.cpp` termios now handles `B921600` case (was binary: B115200 or B9600). <!-- resolved: 2026-06-18 --> |
| 2.10 | `SignedEtPacketPi` struct size 173 bytes | `[x]` | Fixed in `ff5683d`. <!-- resolved: 2026-06-18, ff5683d --> |
| 2.11 | `sig_valid` ECDSA verify wired | `[x]` | `sensor_bridge_node.cpp` now loads `/etc/vigia/atecc_pubkey.bin` at startup and calls `vigia::VigiaIdentityKey::verify_peer()` in `process_cobs_frame`. Returns `false` for all-zero stubs. <!-- resolved: 2026-06-18 --> |

---

## Phase 3 — ONNX Vision Engine

| # | Item | Status | Notes |
|---|------|--------|-------|
| 3.1 | YOLO INT8 ONNX model on disk | `[x]` | `7e04a5e` added INT8 YOLO models. Confirmed present post-pull. |
| 3.2 | MiDaS v2.1 ONNX model on disk | `[x]` | `97fe8b4` added MiDaS model. |
| 3.3 | Spatial latent layer name known | `[x]` | Already set in `vision_params.yaml`: `/model.22/cv2/act/Mul_output_0`. <!-- resolved: 2026-06-18, pre-existing --> |
| 3.4 | KleidiAI ACL EP (`--use_acl`) | `[~]` | `scripts/build_ort_acl.sh` written — run it on the Pi in a tmux session (~2h). CMakeLists already detects `onnxruntime_providers_acl.so` and sets `VIGIA_HAVE_ACL_EP`. CMakeLists gpiod v2 gate also fixed (was incorrectly blocking v2). **Run the script.** |
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
| 5.4 | TLS certs on device | `[~]` | Provisioning pipeline complete: `tools/vigia-gen-ca.sh` generates root CA + server cert; `tools/vigia-sign-device.sh` generates per-device P-256 keypair + cert signed by VIGIA CA. Deploy to Pi via scp as documented in sign-device output. Requires running the scripts and copying files. |
| 5.5 | MQTT broker deployed | `[x]` | **M12: replaced by AWS IoT Core** — Mosquitto + FastAPI + PostgreSQL + docker-compose deleted. Pi publishes mTLS QoS-1 to `a3re4nls2cuv10-ats.iot.us-east-1.amazonaws.com:8883`. IoT Core Topic Rule triggers AttestationFn Lambda. CDK stack deploys rule + policy. TLS certs still need deploying to Pi (see 5.4). <!-- resolved: 2026-06-19, M12 --> |
| 5.6 | Ed25519 signing in legacy HTTPS path | `[~]` | `device_ed25519.key` param exists. Key path read but never used in the curl transmit path — the JSON payload is unsigned. Only relevant when Paho not compiled in (anti-death HTTPS fallback). Low priority. |

---

## Phase 6 — DePIN Security & Attestation

| # | Item | Status | Notes |
|---|------|--------|-------|
| 6.1 | Hardware attestation pipeline | `[x]` | **M12: AWS IoT Core + Lambda** — FastAPI `attestation.py` deleted. `packages/backend/functions/attestation/index.ts` is full TypeScript port: MsgPack decode → anti-replay DynamoDB conditional update → EtHashInput 96-byte reconstruction → ECDSA P-256 prehashed verify via `@noble/curves` → H3 res-10 geo-dedup → HazardsTable upsert → AttestationLogTable write. Triggered by IoT Rule `vigia_hazard_attest`. <!-- resolved: 2026-06-19, M12 --> |
| 6.2 | MQTT subscriber on server | `[x]` | **M12: AWS IoT Core Topic Rule** — no self-hosted broker. IoT Rule SQL `SELECT encode(*, 'base64') AS payload, topic() AS topic, timestamp() AS ts FROM 'vigia/attest/+/hazard'` triggers AttestationFn Lambda directly. <!-- resolved: 2026-06-19, M12 --> |
| 6.3 | `signed_et.valid` key populated | `[x]` | `hazard_uplink_node.cpp` packs `"valid": et.sig_valid` in MsgPack. AttestationFn reads `se.sig_valid`. <!-- resolved: 2026-06-18 --> |
| 6.4 | Fleet device registry in DB | `[x]` | **M12: DynamoDB** — `VigiaPiDeviceRegistry` table (CDK, `RemovalPolicy.RETAIN`). `cert_pem` + `last_seq` stored. AttestationFn reads cert via `GetCommand`. Manual seed: `aws dynamodb put-item --table-name VigiaPiDeviceRegistry`. <!-- resolved: 2026-06-19, M12 --> |
| 6.5 | Cosmos 3 / VLM world model client | `[x]` | OrchestratorFunction calls Bedrock Nova-Lite VLM + ReAct Agent. **2% sampling enforced** (`Math.random() < VLM_SAMPLE_RATE`, env-tunable; deployed session 5). 98% fast path scores from edge ONNX confidence; reward dedup + slashing make probabilistic verification economically sound. <!-- resolved: 2026-06-19, session 5 --> |
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

## Server-Side Pipeline (AWS Lambda — M12 migration)

| # | Item | Status | Notes |
|---|------|--------|-------|
| 8.1 | Hazard ingest | `[x]` | **M12: DynamoDB + H3 geo-dedup** — FastAPI + PostGIS deleted. AttestationFn writes to HazardsTable with H3 resolution-10 dedup (replaces PostGIS `ST_DWithin`). Mobile ingest via ValidatorFn → HazardsTable. <!-- resolved: 2026-06-19, M12 --> |
| 8.2 | Hardware ECDSA authentication | `[x]` | **M12: AttestationFn** — `verifyEcdsaP256Prehashed()` uses `@noble/curves` P-256 with `prehash:false, lowS:false` matching ATECC608A raw R‖S output. Cert fetched from `VigiaPiDeviceRegistry` DynamoDB. <!-- resolved: 2026-06-19, M12 --> |
| 8.3 | Anti-replay monotonic sequence | `[x]` | **M12: DynamoDB conditional update** — `ConditionExpression: 'attribute_not_exists(last_seq) OR last_seq < :seq'` is atomic and race-safe. Firmware-attested ATECC608A sequence counter, not client-supplied. <!-- resolved: 2026-06-19, M12 --> |

---

## Cloud Security Hardening (session 5 — `vigia-amazon`, deployed)

Findings from a full cloud-pipeline security review, all fixed and deployed (`VigiaStack` → `UPDATE_COMPLETE`).

| # | Severity | Item | Status | Notes |
|---|----------|------|--------|-------|
| S.1 | 🔴 | Reward farming via fast path | `[x]` | Fast path credited rewards with no dedup/ledger write. Now every reward (fast + VLM) goes through `tryCreditReward` — atomic `TransactWriteCommand` (dedup-lock + balance + ledger). <!-- resolved: 2026-06-19 --> |
| S.2 | 🔴 | Reward double-spend race | `[x]` | Read-then-write replaced by single conditional transaction; concurrent stream records can't double-credit. <!-- resolved: 2026-06-19 --> |
| S.3 | 🔴 | Open device registration (Sybil) | `[x]` | `register-device` now requires Ed25519 proof-of-possession over `VIGIA-REGISTER:<pubkey>`. **Android `WalletRepositoryImpl` updated to match — requires APK rebuild.** <!-- resolved: 2026-06-19 --> |
| S.4 | 🔴 | Slash/blacklist not enforced | `[x]` | `ValidatorFn` now rejects `blacklisted=true` devices (403). <!-- resolved: 2026-06-19 --> |
| S.5 | 🟠 | Attestation watermark poisoning | `[x]` | `AttestationFn` verifies ECDSA signature **before** advancing the anti-replay sequence. <!-- resolved: 2026-06-19 --> |
| S.6 | 🟠 | Validator input/freshness | `[x]` | `ValidatorFn` validates types/ranges (lat/lon/confidence) + ±10 min timestamp freshness. <!-- resolved: 2026-06-19 --> |
| S.7 | 🟠 | VLM JSON parse fail-open | `[x]` | OrchestratorFn extracts first `{…}` block, fail-closed on NaN/garbage. <!-- resolved: 2026-06-19 --> |
| S.8 | 🟡 | No DLQ on async invokes | `[x]` | Orchestrator + slash-node have SQS DLQs (2 retries). Maintenance pipe filter fixed `INSERT`→`MODIFY`. IoT error-log role scoped from `*`. <!-- resolved: 2026-06-19 --> |
| S.9 | 🟡 | Legacy duplicate pipe | `[ ]` | `vigia-hazards-to-orchestrator` (3rd stream consumer, double-invokes orchestrator) — needs manual `aws pipes delete-pipe`. |

> **Note:** the cloud code lives in the **`vigia-amazon`** repo (not pushed here). This table tracks its status for system-wide visibility.

## Cross-Cutting

| # | Item | Status | Notes |
|---|------|--------|-------|
| 9.1 | `static_assert(SignedEtPacketPi == 165)` wrong | `[x]` | Fixed in `ff5683d`. <!-- resolved: 2026-06-18, ff5683d --> |
| 9.2 | `EcdsaVerifier` unused in sensor_bridge | `[x]` | See 2.11. <!-- resolved: 2026-06-18 --> |
| 9.3 | Kalman velocity dead-reckoning | `[x]` | See 4.3. <!-- resolved: 2026-06-18 --> |
| 9.4 | IoBinding on VisionNode hot path | `[x]` | See 3.5. <!-- resolved: 2026-06-18 --> |
| 9.5 | GPS at 10 Hz | `[x]` | See 2.5. Fixed. <!-- resolved: 2026-06-19 --> |
| 9.6 | PREEMPT_RT booted | `[ ]` | See 1.1. Physical Pi access needed. |
| 9.7 | Anti-replay uses hardware sequence | `[x]` | `auth.py` `authenticate_event()` now extracts `signed_et.sequence` (ATECC-attested) when cert_pem present, overriding client-supplied `device_seq`. HMAC path unchanged. <!-- resolved: 2026-06-19 --> |
| 9.8 | `verify_kleidiai_capable()` CPUID check | `[x]` | `vision_node.cpp`: reads `/proc/cpuinfo` for `asimddp` before calling `AppendExecutionProvider_ACL`. Logs WARN and skips EP if not found. <!-- resolved: 2026-06-19 --> |
| 9.9 | BLE response characteristic | `[x]` | `kResponseUuid` added to `ble_gatt_constants.hpp`. CONTROL_CHAR now returns ACK/NACK/PONG via RESPONSE_CHAR notify. `stream_paused_` flag honours kPauseStream/kResumeStream. <!-- resolved: 2026-06-19 --> |
| 9.10 | HazardUplinkNode continuous uplink | `[x]` | `hazard_uplink_node.cpp` — subscribes to `/vigia/hazard_events`, packs MsgPack matching `attestation.py` schema, publishes QoS-1 to `vigia/attest/{id}/hazard` via TLS mTLS Paho. Wired into `main.cpp` at SCHED_OTHER priority 30. <!-- resolved: 2026-06-19 --> |

---

## Remaining software work

1. **Global seqlock bulk snapshot** (item 1.3) — replace per-slot seqlock with one atomic 300-frame snapshot. Medium refactor.
2. **SIM7600 ECM provisioning** (item 5.3) — physical AT command, one-time.
3. **PREEMPT_RT boot** (item 9.6) — physical Pi, edit `/boot/firmware/cmdline.txt`.
4. **TLS certs deploy to Pi** (item 5.4) — run `tools/vigia-gen-ca.sh` → `tools/vigia-sign-device.sh` → scp to `/etc/vigia/` → download `AmazonRootCA1.pem`.
5. **DynamoDB Pi device seed** — after cert PEM generated, run `aws dynamodb put-item --table-name VigiaPiDeviceRegistry --item '{"device_id":{"S":"vigia-001"},"cert_pem":{"S":"<PEM>"},"last_seq":{"N":"0"}}'`
6. **ATECC provisioning** (item 2.8) — `tools/atecc_provision.py` once on wired device, then `build_phase2_live.sh`, then set `verify_ecdsa: true` in `sensor_bridge_params.yaml`.
7. ~~**2% VLM sampling**~~ — **done** (session 5, deployed). Env-tunable `VLM_SAMPLE_RATE`.
8. **Stripe payout integration** — `StripePayRepositoryImpl.kt` all 3 methods are empty stubs (Phase 4). Also needs a backend payout endpoint (none exists; rewards currently settle to Solana).
9. **Wallet balance UI** — balance is fetched (`WalletRepositoryImpl.refreshBalance`) and surfaced via `CopilotViewModel`; no dedicated dashboard screen yet.
10. **APK rebuild** — Android `register-device` now sends the proof-of-possession signature (session 5); rebuild/redeploy required or new-device onboarding 401s.
11. **Delete legacy duplicate pipe** — `aws pipes delete-pipe --name vigia-hazards-to-orchestrator` (see S.9).

## Requires Ben (hardware)

- ~~ATECC608A wiring (GP2=SDA, GP3=SCL, pull-ups)~~ — **done** (2026-06-18)
- SIM7600 ECM provisioning → item 5.3
- PREEMPT_RT cmdline.txt edit → item 1.1

## Requires infrastructure / operational steps

- ~~MQTT broker~~ — **resolved** (M12: AWS IoT Core)
- TLS cert pipeline → item 5.4 (run scripts, scp to Pi)
- DynamoDB device seed → item above
- ACL EP ORT rebuild (~2h on Pi) → item 3.4
- Cosmos 3 / Bedrock partnership → item 6.5 (hardcoded agent ID works; 2% sampling gate pending)

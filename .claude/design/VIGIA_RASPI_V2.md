# VIGIA-RASPI — Master Design Spec V2 (rev 2.2)

**Status:** Active, internally reconciled. Supersedes v2.0/v2.1 and everything in `.claude/design/archive/`.
**Scope:** the hardware edge node — Pico 2 firmware, ROS 2 `vigia_edge_node`, ONNX vision, fusion, anti-death storage, BLE transport, DePIN uplink.
**Audited against:** `main` + `fix/v2-p0-edge@293a477`. Two independent cross-reviews (Codex ×2) plus first-party source re-verification.
**Companion specs:** [vigia-amazon V2](../../../vigia-amazon/docs/design/VIGIA_AMAZON_V2.md) · [vigia-public V2](../../../vigia-public/docs/design/VIGIA_PUBLIC_V2.md) · [vigia2 V2](../../../../AndroidStudioProjects/vigia2/docs/design/VIGIA2_V2.md).
**Cross-repo protocol findings** (identity, sequencing, pairing) live in §5 and are shared verbatim across all four specs.

---

## 0. How to read this document

**Finding status** (single source of truth — a finding appears once, with one status):
`OPEN` · `IMPLEMENTED` · `IMPLEMENTED-PARTIAL` · `RETRACTED` (was wrong) · `SUPERSEDED` (folded into another ID) · `RECLASSIFIED` · `CLOSED` (verified already-correct).

**Severity:** `P0` data-integrity/security/false-pitch-claim · `P1` real defect/hardening · `P2` quality.

Every OPEN/PARTIAL finding carries: file:line · failure scenario · fix · **acceptance criteria** · dependencies. Resolved items (§6) carry only status + rationale — they are **not** scheduled in the work plan (§7). The work plan lists only OPEN/PARTIAL work. The status matrix (§1) is authoritative for what is done vs pending.

---

## 1. Implementation status matrix (authoritative, keyed by branch@commit)

| ID | Title | Sev | Status | Where |
|----|-------|-----|--------|-------|
| R-CRIT-4 | Uplink topic/QoS mismatch | P0 | IMPLEMENTED | `fix/v2-p0-edge@293a477` (topic+QoS unified; needs on-Pi colcon run) |
| R-CRIT-5 | systemd doesn't load MQTT config | P0 | IMPLEMENTED-PARTIAL | `fix/v2-p0-edge@293a477` (config now loaded; TLS paths still wrong → R-CRIT-7) |
| R-CRIT-7 | Provisioned TLS paths don't match config | P0 | OPEN | new; blocks uplink even after R-CRIT-5 |
| R-CRIT-1 | Continuous uplink drops events offline | P0 | OPEN | no spool yet; unblocked once R-CRIT-4/5/7 land |
| R-CRIT-3 | Legacy HTTPS path sends unsigned JSON | P0 | OPEN | — |
| R-CRIT-6 | Canonical device id + reboot/QoS1-safe sequence | P0 | OPEN | cross-repo, see §5.1 |
| R-SEC-7 | Verifies supplied hash, not the telemetry envelope | P1 | OPEN | supersedes R-SEC-5 |
| R-SEC-6 | Pi authenticates any phone (no allow-list) | P1 | OPEN | cross-repo, see §5.2 |
| R-BUG-3 | YAML keys don't match declared ROS params | P1 | OPEN | broad silent-default drift |
| R-SEC-1 | RNG unchecked fread + threading | P2 | OPEN | (softened from v2.0) |
| R-BUG-4 | MQTT client leaked on reconnect | P2 | OPEN | — |
| R-QUAL-9 | Missing broker on startup should be fatal | P2 | OPEN | fold into R-CRIT-7 |
| R-QUAL-1 | CLAUDE.md merge conflict committed | P2 | OPEN | — |
| R-QUAL-2..8 | Hardening batch | P2 | OPEN | see §3 |
| R-SEC-4 | Frame-image hash binding | P2 | OPEN (re-scoped) | needs new protocol, see §3 |
| R-BUG-1 | COBS decode bug | — | RETRACTED | false; §6 |
| R-CRIT-2 / R-BUG-2 | Emergency snapshot deadline | — | RECLASSIFIED | dead code; §6 |
| R-SEC-3 | Persist Pi watermark | — | SUPERSEDED | by R-CRIT-6; §6 |
| R-SEC-5 | Watermark advanced before verify | — | SUPERSEDED | by R-SEC-7; §6 |

---

## 2. Architecture recap (as-built, verified)

```
Pico 2 (no-heap C): BNO085 + NEO-M8N + ATECC608A → E_t hash + ECDSA R‖S → COBS@921600 →USB→
Pi 5 (ROS 2 Jazzy): sensor_bridge → /vigia/{imu,gps,signed_et}
  camera → shm_ring → vision (YOLO26 INT8 + MiDaS, ONNX+ACL/KleidiAI) → depth → fusion
  fusion (grav-comp ISS, Kalman, degraded RRI) → /vigia/hazard_event →
    hazard_uplink_node (continuous QoS1) ─┐
    anti_death_node    (emergency, UPS GPIO) ─┼→ AWS IoT Core (mTLS) → AttestationFn
    ble_gatt_node      (ECDH P-256, DimsFrame) → Android app
```
Verified-correct (keep): gravity-compensated ISS, `Ort::IoBinding` hot path, per-slot seqlock ring, `vigia_ecdh.hpp` crypto, libgpiod v2 port, COBS codec (see R-BUG-1 retraction).

---

## 3. Open findings

### R-CRIT-7 — Documented TLS provisioning cannot start the uplink (P0, OPEN)
**Files:** `config/hazard_uplink_params.yaml:9` (`mqtt_key_path: /etc/vigia/device.key`, `mqtt_ca_path: /etc/vigia/AmazonRootCA1.pem`) vs `tools/vigia-sign-device.sh:20,78` (installs `/etc/vigia/device_key.pem`) vs `anti_death_node.cpp:~305` (hardcoded `/etc/vigia/ca_chain.pem`, `/etc/vigia/device_cert.pem`).
**Failure:** three components name three different key/CA paths. A Pi provisioned exactly as documented loads the MQTT params (R-CRIT-5) but the Paho TLS handshake can't find the private key / CA, so both continuous and emergency uplinks silently fail to connect. This is why R-CRIT-5 is only partial.
**Fix:** one canonical path set (recommend `device_cert.pem`, `device_key.pem`, `AmazonRootCA1.pem` under `/etc/vigia/`); update `hazard_uplink_params.yaml`, make `vigia-sign-device.sh` emit exactly those, and **parameterize anti-death's TLS paths** (stop hardcoding `ca_chain.pem`). Add boot-time fatal validation of broker host, cert, key, CA existence + perms (0600 key) + canonical device id (see R-QUAL-9).
**Acceptance:** on a fresh Pi, only the documented provisioning steps → `hazard_uplink_node` connects and a test event reaches IoT Core; wrong/missing key path → node exits non-zero at boot with a clear message (not a silent WARN).
**Depends on:** R-CRIT-6 (canonical device id).

### R-CRIT-1 — Continuous uplink drops events when offline (P0, OPEN)
**File:** `hazard_uplink_node.cpp` `on_hazard` (early-return + WARN when disconnected).
**Failure:** in an LTE dead zone every hazard between disconnect and reconnect is lost. Contradicts the "store-and-forward, works in dead zones" pitch. Only the emergency path has QoS-1 persistence, and only for its single event.
**Fix:** durable on-disk spool (`/var/lib/vigia/uplink_spool/`, bounded, FIFO-evict). Append packed frame+topic on disconnect; drain oldest-first on reconnect. Frames are already signed + monotonic-sequenced, so cloud idempotency (R-CRIT-6 / [A-CRIT-3](../../../vigia-amazon/docs/design/VIGIA_AMAZON_V2.md)) makes replay-on-reconnect safe.
**Acceptance:** antenna-pull mid-drive → zero event loss, correct ordering after reconnect, spool survives reboot, bounded disk use.
**Depends on:** R-CRIT-4/5/7, R-CRIT-6.

### R-CRIT-3 — Legacy HTTPS fallback transmits unsigned JSON (P0, OPEN)
**File:** `anti_death_node.cpp` `#else !VIGIA_HAVE_PAHO_MQTT` branch.
**Failure:** builds without Paho POST an unsigned `{hazardType,lat,lon,timestamp,confidence}` — spoofable; breaks "the road signed the evidence."
**Fix:** delete the legacy path and make Paho a hard build dependency (preferred), or sign the body with the device key and verify server-side. One authenticated path only.
**Acceptance:** server rejects a forged/unsigned legacy POST (401); build fails fast if Paho absent.

### R-SEC-7 — Pi verifies a supplied hash, not the telemetry envelope (P1, OPEN — supersedes R-SEC-5)
**File:** `sensor_bridge_node.cpp:418` (advances `last_et_seq_`), `:455-465` (`verify_peer(pubkey, pkt.et_hash, 32, pkt.ecdsa_sig, 64)`).
**Failure:** (a) watermark + IMU history mutated *before* verification (forged high-seq packet poisons the watermark, suppressing genuine frames); (b) the Pi verifies the signature over the **supplied** `pkt.et_hash` but never reconstructs the canonical 96-byte `EtHashInput` from packet fields to compare. An attacker transplants a valid `(et_hash, ecdsa_sig)` onto a packet with modified sequence/IMU/GPS; the Pi accepts, publishes, and fuses tampered telemetry locally (even if the cloud later rejects on recompute).
**Fix (strict order):** 1) reconstruct `EtHashInput` from packet fields using the canonical device id; 2) SHA-256 + constant-time compare vs `pkt.et_hash`; 3) verify ECDSA; 4) validate boot epoch + sequence (R-CRIT-6); 5) only then advance watermark / update IMU history / publish. Drop+count on any failure.
**Acceptance:** unit test — a packet with a valid transplanted hash+sig but mutated GPS is rejected and not published; watermark unchanged after a rejected packet.
**Depends on:** R-CRIT-6, the firmware `EtHashInput` layout.

### R-BUG-3 — YAML parameter keys don't match declared ROS params (P1, OPEN)
**Evidence:** `fusion_params.yaml` uses `rri_w_vision/rri_w_depth/rri_w_imu/rri_w_temporal` but code declares `w_yolo/w_geometry/w_temporal/w_iss`; `vision_params.yaml` uses `nms_threshold` vs code `nms_iou_threshold`; `sensor_bridge_params.yaml` uses `serial_port` vs code `serial_device` (and `verify_ecdsa`, `protocol_hint`, `imu_history_size`, health-rate keys are not declared/consumed).
**Failure:** intended production values silently fall back to hardcoded defaults — RRI weighting, NMS threshold, serial device, etc. are not what the YAML says.
**Fix:** one authoritative parameter schema per node; rename YAML keys (or code) to match; launch-time check that fails on undeclared YAML keys and missing required params.
**Acceptance:** a launch test asserts every YAML key maps to a declared param and no required param uses a silent default; a deliberately misspelled key fails the test.

### R-SEC-4 — Frame-image hash binding (P2, OPEN — re-scoped)
**Correction:** not implementable as first written — the Pico signs `EtHashInput` and cannot sign a Pi-side camera-frame hash without a **new bidirectional Pi↔Pico signing protocol**, and the cloud cannot recompute a frame hash unless the exact hashed bytes are uploaded. Re-scope: either (a) add a Pi-side device-Ed25519 signature over `sha256(anonymized_frame)` carried alongside the ATECC envelope, cloud recomputes from the uploaded blurred frame; or (b) drop the claim. Decide before promising it on stage.
**Depends on:** R-AZ-2 (anonymization) if frames are exported.

### R-SEC-1 — BLE nonce RNG hardening (P2, OPEN — softened)
`ble_gatt_node.cpp:57-80`. mbedTLS CTR-DRBG auto-reseeds (v2.0 "never reseeded" dropped). Remaining defects: unchecked `fread` in the `/dev/urandom` fallback (short read → zero nonce), unchecked DRBG return codes, undocumented single-thread invariant. **Fix:** route randomness through the mutex-guarded `VigiaIdentityKey` DRBG; hard-fail the handshake on any RNG error/short read; document threading.

### R-BUG-4 — MQTT client leaked on reconnect (P2, OPEN)
`hazard_uplink_node.cpp:79-80` — `connect_mqtt()` `new`s a client into `mqtt_client_` without deleting the previous one; every reconnect leaks. **Fix:** reset/delete the existing client (or hold a `unique_ptr`) before constructing a new one, or reuse `reconnect()`.

### R-QUAL-9 — Missing broker on startup should be fatal (P2, OPEN)
`hazard_uplink_node` returns early (partially-alive node) when the broker is unset. Required production config should fail startup, not WARN. Fold into R-CRIT-7 boot validation.

### R-QUAL-1..8 (P2, OPEN)
CLAUDE.md merge conflict (R-QUAL-1); TLS 1.3 where supported (R-QUAL-2); key-file permission checks (R-QUAL-3); remove dead `to_hex` (R-QUAL-4); firmware boot self-test rejecting all-zero sig in production builds (R-QUAL-5); centralize magic sizes (R-QUAL-6); document per-node threading (R-QUAL-7); fusion depth-ROI `orig_w/h` TODO (R-QUAL-8).

---

## 5. Cross-repo protocol findings (shared across all four specs)

### 5.1 — Canonical device identity + reboot/QoS1-safe sequencing (P0, OPEN) — R-CRIT-6 / A-CRIT-3
**Evidence:** firmware signs the 16-byte zero-padded ATECC serial (`firmware/src/main.c:140`); Pi packs the string `"vigia-001"` (`hazard_uplink_node.cpp:27`); cloud requires `device_id` to parse as exactly 16-byte hex (`vigia-amazon .../attestation/index.ts:54`, throws otherwise). Firmware `s_et_seq` resets to 0 on every reboot while the cloud durably requires `seq > last_seq`.
**Two problems:** (1) three inconsistent identities → attestation can't validate with shipped defaults; (2) sequencing is neither reboot-safe nor QoS1-safe.
**QoS1 nuance:** QoS-1 legitimately redelivers on a lost PUBACK, so the cloud must be **idempotent**, not purely anti-replay:
- Accept the same `(deviceId, bootEpoch, sequence, payloadHash)` idempotently (credit once).
- Reject the same `(deviceId, bootEpoch, sequence)` with a *different* payloadHash (tamper).
- Reject older sequences within an epoch; reject reused/revoked boot epochs.
- Atomically record accepted event + sequence state.
**Fix:** ONE ATECC-derived device id used for signatures, certs, DynamoDB registry, MQTT topic, and pairing QR. Replace reset-to-0 with **signed boot epoch + monotonic sequence** (or secure persistent monotonic counter); include `bootEpoch` in `EtHashInput` and payload; cloud keeps a used-epoch registry.
**Acceptance:** reboot the Pico → new epoch, sequence from 0, all genuine events accepted; replayed old-epoch packet rejected; duplicate QoS1 redelivery credited exactly once; identity string identical across firmware sig, cert CN, registry key, MQTT client-id, and QR.
**Owners:** firmware + Pi (R-SEC-7, R-CRIT-7) + cloud ([A-CRIT-3](../../../vigia-amazon/docs/design/VIGIA_AMAZON_V2.md)).

### 5.2 — Wallet/device/phone key hierarchy + pairing/delegation protocol (P0, OPEN) — R-SEC-6 / M-CRIT-2 / A-SEC-6
Three distinct keys are currently conflated: **wallet** Ed25519 (Android `Ed25519KeyStore`), **Android BLE handset identity** P-256 (Android Keystore, hardware), **device telemetry identity** P-256 (Pico/ATECC). The cloud claim binds wallet↔device but never binds/delegates the Android BLE key, so the Pi (R-SEC-6) has nothing to build a phone allow-list from. Bootstrap loop: Android needs a device signature to claim, but a phone-allow-list model would reject an unclaimed phone before it can request that signature.
**Protocol to define (before coding any side):**
- A physical-presence/bootstrap pairing state (QR-gated) permitting the first handshake.
- A Pi↔Pico command letting the ATECC sign the binding challenge (unblocks Android `deviceSig`, [M-CRIT-2](../../../../AndroidStudioProjects/vigia2/docs/design/VIGIA2_V2.md)).
- A signed delegation from wallet Ed25519 → Android BLE P-256, recorded in the claim ([A-SEC-6](../../../vigia-amazon/docs/design/VIGIA_AMAZON_V2.md)).
- Persistent **per-central** handshake state (R-SEC-6), revocation, replacement-phone recovery, retry semantics.
**Acceptance:** a claimed phone (and only a claimed phone) completes the handshake; a rogue central is rejected; a replacement phone recovers via bootstrap; the Pi builds its allow-list from the claim record.

---

## 6. Resolved / retracted / reclassified (not scheduled)

- **R-BUG-1 — RETRACTED (false).** Hand-trace of `cobs_tx_driver.c` ↔ `decode_cobs` + 60k-round-trip fuzz: decoder correct; the trailing-zero strip removes COBS's implicit final-group delimiter and genuine payload trailing zeros are preserved. Keep a property test only.
- **R-CRIT-2 / R-BUG-2 — RECLASSIFIED (dead code).** `snapshot_all()` has no caller; the anti-death path captures metadata+latent+signed-telemetry+latest-hazard, not the pixel ring. If raw black-box imagery is required, the defect is that it is never captured; otherwise `snapshot_all` + global seqlock are removable.
- **R-SEC-3 — SUPERSEDED by R-CRIT-6** (persisting only the Pi watermark worsens reboot lockout).
- **R-SEC-5 — SUPERSEDED by R-SEC-7.**

---

## 7. Priority-ordered work plan (OPEN/PARTIAL only)

1. **R-CRIT-6 / §5.1** canonical device id + boot-epoch/QoS1-safe sequencing (cross-repo).
2. **R-CRIT-7** unify TLS paths + fatal boot validation (finishes R-CRIT-5).
3. **R-SEC-7** verify-before-mutate with canonical `EtHash` reconstruction.
4. **R-CRIT-1** on-disk store-and-forward spool.
5. **R-CRIT-3** delete/sign the legacy HTTPS path.
6. **§5.2** pairing/delegation protocol → R-SEC-6 phone allow-list.
7. **R-BUG-3** YAML/param schema + launch test.
8. **R-SEC-1, R-BUG-4, R-QUAL-1..9** hardening batch.
9. **R-SEC-4** frame-hash (design-dependent, after R-AZ-2).

Each item is "done" only when its acceptance criteria pass on real hardware (ROS/colcon can't run in the review sandbox).

---

## 8. Azure / IC-2027 transition

- **R-AZ-1 — IoT Hub is not a broker-string swap.** Azure IoT Hub mandates a specific MQTT username/topic format with feature/size constraints → needs a **transport adapter**, not just a host change. DPS X.509 enrolment reuses the ATECC identity (R-CRIT-6).
- **R-AZ-2** on-device face/plate anonymization before any frame leaves the node (DPDP, May 13 2027); prerequisite for a real R-SEC-4.
- **R-AZ-3** `phi_edge_node` via Foundry Local (Linux ARM64), gated on the August spike; pin ≤2 cores, reuse Dynamic-Dims degradation.
- **R-AZ-4** store-and-forward (R-CRIT-1) precedes the offline-copilot story.

---

## Appendix — verification method
Findings re-verified by reading cited files at `fix/v2-p0-edge@293a477` / `design/v2-specs`. COBS checked by hand-trace + fuzz. ROS runtime + firmware could not be built in the review environment; those items are marked "needs on-Pi verification."

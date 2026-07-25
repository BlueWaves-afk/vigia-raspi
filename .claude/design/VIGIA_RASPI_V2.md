# VIGIA-RASPI — Master Design Spec V2

**Status:** Active · supersedes everything in `.claude/design/archive/` (the numbered `0x_*` contracts, `GAP_TRACKER.md`, `SYSTEM_AUDIT_2026-06-20.md`, `07_ble_transport_spec.md`).
**Scope:** the hardware edge node — Pico 2 firmware, ROS 2 `vigia_edge_node`, ONNX vision, sensor fusion, anti-death storage, BLE transport, DePIN uplink.
**Audited:** 2026-07-25 against `main` (commit `90abcd9`). This is a review + improvement spec, not an implementation. Nothing here is built yet.
**Companion specs:** [vigia-amazon V2](../../../vigia-amazon/docs/design/VIGIA_AMAZON_V2.md), [vigia-public V2](../../../vigia-public/docs/design/VIGIA_PUBLIC_V2.md), [vigia2 V2](../../../../AndroidStudioProjects/vigia2/docs/design/VIGIA2_V2.md).

---

## 0. How to read this document

Every finding carries a stable ID (`R-CRIT-n`, `R-SEC-n`, `R-BUG-n`, `R-QUAL-n`), a severity, a file:line anchor, the concrete failure scenario, and the V2 fix. Severities:

- **P0 (critical)** — data-integrity, security, or a pitch claim that is currently false. Fix before any pilot or IC demo.
- **P1 (high)** — real defect or hardening gap; fix during the Aug–Nov build.
- **P2 (medium)** — quality / maintainability / defense-in-depth.

Traceability to the shipped audit: the archived `GAP_TRACKER.md` tracked *feature completeness*. This spec tracks *correctness and security of what exists*. Where an item was marked done there but is still risky here, it is re-opened with a new ID.

---

## Review Reconciliation (v2.1 — cross-reviewed and verified against source, 2026-07-25)

An independent second review (Codex) cross-checked this spec against source; every item below was then re-verified by reading the cited files. **This section is authoritative where it conflicts with the original findings.**

### Findings RETRACTED / REVISED

- **R-BUG-1 (COBS decode) — RETRACTED (false).** Hand-tracing `cobs_tx_driver.c:cobs_encode` against `sensor_bridge_node.cpp:decode_cobs`, plus 60k-round-trip fuzzing, shows the decoder is correct: the unconditional trailing-`0x00` strip removes COBS's implicit final-group delimiter, and genuine payload trailing zeros are preserved (`[0x11,0x00]` round-trips intact). Keep the proposed property test as a regression guard; there is no bug.
- **R-CRIT-2 & R-BUG-2 (emergency snapshot) — RECLASSIFIED to dead-code / design-drift.** `ShmRingBuffer::snapshot_all()` has **no caller** (verified by grep). The anti-death path captures compact frame metadata, a latent pointer, signed telemetry, and the latest hazard — not the raw pixel ring — so there is no active emergency-deadline failure. If raw black-box imagery is a requirement, the real defect is that it is never captured; otherwise `snapshot_all` + the global seqlock are removable dead code.
- **R-SEC-1 (BLE RNG) — SOFTENED.** mbedTLS CTR-DRBG auto-reseeds at its configured interval, so "seeded once, never reseeded" is inaccurate. Retained valid defects: unchecked `fread` in the `/dev/urandom` fallback (short read → zero nonce), unchecked DRBG return codes, and the undocumented single-thread assumption.
- **R-SEC-3 (edge anti-replay) — SUPERSEDED by R-CRIT-6.** Persisting only the Pi's `last_et_seq_` would *worsen* the reboot lockout in R-CRIT-6. Do not persist the Pi watermark in isolation.

### New CONFIRMED findings

- **R-CRIT-4 — Continuous uplink topic mismatch (P0).** `fusion_node.cpp:38` publishes `/vigia/hazard_event` (singular); `hazard_uplink_node.cpp:52` subscribes `/vigia/hazard_events` (plural). No other publisher of the plural topic exists → the continuous uplink receives **zero** events, online or offline (this makes R-CRIT-1 moot until fixed). Fix: unify the topic via a shared constant used by both nodes.
- **R-CRIT-5 — systemd never loads the MQTT config (P0).** `mqtt_broker_host` is defined only in `config/params.yaml`, which neither `config/vigia-edge.service` nor `systemd/vigia-edge.service` passes via `--params-file`. `HazardUplinkNode` sees an empty broker host and disables itself; the anti-death emergency path likewise lacks broker config. The deployed unit can silently lose both continuous and emergency uploads. Fix: load `params.yaml` (correct node namespace) or consolidate uplink params into a loaded file; add a boot assertion that the broker host is non-empty.
- **R-CRIT-6 — Canonical device identity + reboot-safe sequencing (P0, cross-repo).** Three identities disagree: firmware signs the 16-byte zero-padded ATECC serial (`main.c:140`), the Pi packs the string `"vigia-001"` (`hazard_uplink_node.cpp:27`), and the server requires `device_id` to parse as exactly 16-byte hex (`vigia-amazon .../attestation/index.ts:54`, throws otherwise). Separately, firmware `s_et_seq` resets to 0 on every reboot (`main.c:140`) while the server durably requires `seq > last_seq`, so genuine post-reboot events are rejected until the old watermark is passed. Fix: ONE ATECC-derived device id used for signatures, certs, registry, MQTT topic, and pairing QR; and a signed boot-epoch + sequence (or secure persistent monotonic counter). Coordinate with vigia-amazon + vigia2. **Supersedes R-SEC-3.**
- **R-SEC-5 — Local watermark advanced before ECDSA verify (P1).** `sensor_bridge_node.cpp:419` advances `last_et_seq_` before the signature check at `:457`. A forged high-sequence COBS frame poisons the local watermark and suppresses subsequent genuine frames. Fix: verify signature first, then advance atomically.
- **R-SEC-6 — Pi authenticates any phone (P1).** `ble_gatt_node.cpp:283` verifies the RESPONSE signature using the public key supplied *in that same response* and never checks it against an authorized phone identity; handshake state is global rather than per-central. Any central can complete the handshake. Fix: pin/allow-list authorized phone identities (from the claim-device binding), key handshake state per central, reject unknown peers. (Also the Pi has no "sign binding challenge" command — required by vigia2 M-CRIT-2.)

### Revised priority (raspi)

1. R-CRIT-4 topic unify + R-CRIT-5 config load — restore uplink at all.
2. R-CRIT-6 canonical identity + reboot-safe sequence (cross-repo).
3. R-CRIT-1 store-and-forward; R-CRIT-3 delete/sign legacy path.
4. R-SEC-5 verify-before-advance; R-SEC-6 phone authorization; R-SEC-1/2 hardening.
5. R-SEC-4 frame hash; R-QUAL-1 CLAUDE.md; remaining hardening.

Removed from scope: R-BUG-1; R-CRIT-2/R-BUG-2 (unless raw black-box imagery is a requirement).

---

## 1. Architecture recap (as-built, verified)

```
Pico 2 (no-heap C)                     Raspberry Pi 5 (ROS 2 Jazzy, C++)                 Cloud
─────────────────                      ────────────────────────────────                 ─────
BNO085 IMU  ─┐                         sensor_bridge_node ─┬─> /vigia/imu
NEO-M8N GPS ─┼─ E_t hash (ATECC608A) ─ COBS@921600 ─USB─►  ├─> /vigia/gps
ATECC608A   ─┘   ECDSA R‖S sign                            └─> /vigia/signed_et ─┐
                                        camera_node ─► shm_ring ─► vision_node    │
                                        (YOLO26 INT8 + MiDaS, ONNX + ACL/KleidiAI)│
                                        depth_node, fusion_node (grav-comp ISS,   │
                                          Kalman, degraded-mode RRI) ─────────────┤
                                        ┌─────────────────────────────────────────┤
                                        │ hazard_uplink_node (continuous, QoS1) ───┼─► AWS IoT Core
                                        │ anti_death_node (emergency, GPIO UPS) ───┼─► (mTLS)  → AttestationFn
                                        │ ble_gatt_node (ECDH P-256, DimsFrame) ───┼─► Android app
                                        └─────────────────────────────────────────┘
```

Verified-correct and load-bearing (keep as-is): gravity-compensated ISS, `Ort::IoBinding` hot-path, seqlock ring, ECDH/HKDF/HMAC in `vigia_ecdh.hpp`, ECDSA verify wiring in `sensor_bridge_node`, mTLS QoS-1 uplink, libgpiod v2 port.

---

## 2. P0 — Critical findings

### R-CRIT-1 — Continuous uplink drops events when offline (contradicts the offline-resilience pitch)
**File:** `vigia_ws/src/vigia_edge_node/src/hazard_uplink_node.cpp:187-194`
**Failure:** `on_hazard()` returns early and logs `"MQTT not connected — dropping event"` whenever the broker is unreachable. Indian highways drop LTE constantly; on any dead zone every hazard between disconnect and reconnect is **lost forever**. The pitch says *"store-and-forward with signed, timestamped buffering… the copilot works in a dead zone."* The continuous path does not do this — only the emergency `anti_death_node` has QoS-1 persistence, and even that only covers the single emergency event.
**V2 fix:** add a durable on-disk ring buffer (`/var/lib/vigia/uplink_spool/`, bounded, e.g. 512 MB, FIFO-evict oldest). On disconnect, append the packed MsgPack frame + topic; on reconnect, drain oldest-first before publishing live. Because each frame is already ATECC-signed and carries a monotonic `sequence`, the server's DynamoDB anti-replay makes replay-on-reconnect safe and idempotent. Persist across reboot. This turns a false claim into a true, demoable one.
**Acceptance:** pull the SIM antenna mid-drive; confirm zero event loss and correct ordering after reconnect.

### R-CRIT-2 — Emergency bulk snapshot can miss its hard deadline under load (per-frame global seqlock)
**File:** `vigia_ws/src/vigia_edge_node/include/vigia_edge_node/shm_ring_buffer.hpp:82-147`
**Failure:** `write_frame()` bumps the **global** seqlock on *every* camera frame. `snapshot_all()` (called by `anti_death_node` on power-loss, on a hard T+ budget) spins retrying while the global seq is odd or changed mid-copy. At full frame rate with a 300-frame full-resolution snapshot, the writer touches the ring far faster than the reader can copy it, so the reader can retry many times — worst case it blows the `kConnectDeadlineS`/`kTransmitDeadlineS` budget and the "black box" captures nothing. The archived GAP_TRACKER (item 1.3) already flagged this as not spec-compliant; V2 escalates it because it defeats the anti-death guarantee.
**V2 fix:** decouple reader from writer. Options, preferred first: (a) **double-buffer / generation snapshot** — writer flips an atomic `active_bank` index; reader copies the inactive bank, so the writer never invalidates an in-progress read; (b) bound reader retries and, on exhaustion, capture the most-recent N frames via per-slot seqlock (accept a slightly torn ring rather than nothing); (c) freeze the writer for the duration of the emergency snapshot (camera_node subscribes to an `emergency_freeze` latch). Whatever the choice, the emergency path must have a **provable upper bound** on snapshot latency.
**Acceptance:** synthetic load test at target FPS; measure worst-case `snapshot_all` latency over 10k iterations; assert < emergency budget.

### R-CRIT-3 — Legacy HTTPS fallback transmits UNSIGNED, unauthenticated hazard JSON
**File:** `vigia_ws/src/vigia_edge_node/src/anti_death_node.cpp:800-861` (the `#else !VIGIA_HAVE_PAHO_MQTT` branch)
**Failure:** the curl path builds `{"hazardType","lat","lon","timestamp","confidence"}` and POSTs it with **no Ed25519/ECDSA signature**, despite reading `device_key_path`. Any event delivered via this path is spoofable — it breaks the entire "the road itself signed the evidence" claim for builds compiled without Paho. A judge probing "what if MQTT is down?" lands directly on this.
**V2 fix:** either (a) sign the JSON body with the device Ed25519 key and have the server verify it (mirror the mobile telemetry signer), or (b) **delete the legacy path entirely** and make Paho a hard build dependency. Recommendation: (b) — one authenticated path is easier to defend than two. If (a), the signed payload must include `sha256(frame)` (see R-SEC-4 cross-repo).
**Acceptance:** server rejects an unsigned/forged legacy POST with 401.

---

## 3. P1 — High findings

### R-SEC-1 — BLE nonce RNG: seed-once static DRBG, unchecked urandom fallback
**File:** `vigia_ws/src/vigia_edge_node/src/ble_gatt_node.cpp:57-80`
**Failure:** `random_bytes()` uses a function-local `static` mbedTLS CTR_DRBG seeded exactly once and never reseeded, and the `#else` branch reads `/dev/urandom` **without checking `fread`'s return** — on a short read the handshake nonce is partially or fully zero, collapsing handshake unpredictability. Also the static contexts are not mutex-guarded (safe today only because one D-Bus thread drives them — an implicit, undocumented invariant).
**V2 fix:** route all randomness through `VigiaIdentityKey`'s owned, mutex-guarded DRBG (add a `random_bytes(n)` method there), reseed per policy, and hard-fail (reject the handshake) if the RNG returns non-success or a short read. Remove the static locals.

### R-SEC-2 — BLE handshake state is unguarded shared state with no expiry / anti-flood
**File:** `vigia_ws/src/vigia_edge_node/src/ble_gatt_node.cpp:222-355` (`hs_state_`, `hs_nonce_pi_`, `session_key_`)
**Failure:** handshake state lives in node members mutated inside D-Bus vtable lambdas with no mutex and no nonce expiry. A malicious central can (a) spam `HELLO` to churn state and force key re-derivation, (b) hold a half-open handshake indefinitely, (c) exploit the fact that ROS callbacks touch the mailbox under `mailbox_mutex_` but the handshake path has no equivalent lock — the threading contract is undocumented.
**V2 fix:** (1) document the single-threaded D-Bus invariant explicitly, or add a `handshake_mutex_`; (2) stamp `hs_nonce_pi_` with a monotonic deadline (e.g. 10 s) and reject `RESPONSE` past it; (3) rate-limit `HELLO` per central address; (4) cap concurrent half-open handshakes.

### R-SEC-3 — Edge anti-replay is in-memory only; resets to 0 on node restart
**File:** `vigia_ws/src/vigia_edge_node/src/sensor_bridge_node.cpp:419-423` (`last_et_seq_`) and `:333` (`last_imu_seq_`)
**Failure:** after a `sensor_bridge_node` restart, `last_et_seq_ == 0`, so previously-seen COBS frames replay-accepted until the counter catches up. The durable guard is server-side (DynamoDB conditional update), which is correct — but the edge check advertises protection it doesn't durably provide.
**V2 fix:** persist `last_et_seq_` to `/var/lib/vigia/seq_state` (fsync on advance), reload on boot; document explicitly that the **server DynamoDB conditional write is authoritative** and the edge check is best-effort dedup. (This is a documentation + small-persistence fix, not a redesign.)

### R-BUG-1 — COBS decode relies on a delimiter heuristic that silently drops valid frames
**File:** `vigia_ws/src/vigia_edge_node/src/sensor_bridge_node.cpp:111-132` + `:400-416`
**Failure:** `decode_cobs()` unconditionally strips a trailing `0x00` (line 130) as a "COBS artefact." If the real `SignedEtPacketPi` ends in a legitimate `0x00` (e.g. `_wire_pad`), the decoded length comes out one short, fails the `dec_len != sizeof(...)` check, and the frame is dropped as a parse error. Signed sensor frames are then intermittently lost with no diagnostic beyond a bumped `parse_errors` counter.
**V2 fix:** replace the delimiter-heuristic decode with a length-prefixed or standards-correct COBS (encode/decode symmetric, no post-hoc trailing-zero surgery). The firmware `cobs_tx_driver.c` and Pi decoder must be changed together and covered by a round-trip unit test over the exact 173-byte struct including trailing-zero payloads.
**Acceptance:** property test: for 10k random 173-byte structs, `decode(encode(x)) == x`.

### R-SEC-4 (cross-repo) — Signed hazard payload omits the frame image hash (H2)
**Files:** `hazard_uplink_node.cpp:113-182` (pack_event) — no `sha256(frame)` field; mirrors the archived GAP A.5.
**Failure:** a valid signer can attach any image to a validly-signed event; the signature covers telemetry but not the visual evidence. Undermines "verified condition evidence."
**V2 fix:** include `sha256(frame_jpeg)` in the ATECC-signed `E_t` hash input **and** in the MsgPack payload; the server (vigia-amazon `AttestationFn` / `ValidatorFn`) must recompute and compare. Coordinate with [vigia-amazon V2 §A-SEC](../../../vigia-amazon/docs/design/VIGIA_AMAZON_V2.md) and the Android telemetry signer.

### R-BUG-2 — `snapshot_all` silently returns stale/zero pixels when the output buffer is too small
**File:** `vigia_ws/src/vigia_edge_node/include/vigia_edge_node/shm_ring_buffer.hpp:136-140`
**Failure:** if `pixels_cap` is undersized, the per-slot memcpy is skipped with no error; the caller believes it captured frames it did not. In the emergency path this means a "successful" black-box with blank frames.
**V2 fix:** return a status/copied-count from `snapshot_all`; caller asserts full coverage or logs a FATAL degradation. Size the buffer from `ring_depth * frame_w * frame_h * 3` at construction and validate.

---

## 4. P2 — Quality / hardening

- **R-QUAL-1** — `CLAUDE.md` at repo root contains an unresolved git merge conflict (`<<<<<<< HEAD` … `>>>>>>>`). Resolve before any judge/mentor screen-share. (Also flagged in the roadmap.)
- **R-QUAL-2** — TLS pinned to 1.2 (`hazard_uplink_node.cpp:86`, `MQTT_SSL_VERSION_TLS_1_2`). Move to 1.3 where the broker supports it; keep 1.2 floor.
- **R-QUAL-3** — No file-permission/ownership check when loading `/etc/vigia/*.key` / `*.der` / `atecc_pubkey.bin` (`vigia_ecdh.hpp:95`, `sensor_bridge_node.cpp:141`). Assert `0600` + owner before use; refuse world-readable keys.
- **R-QUAL-4** — `to_hex()` in `hazard_uplink_node.cpp:14` appears unused → dead code; remove.
- **R-QUAL-5** — Firmware `main.c` should hard-refuse to transmit in a **production** build (`VIGIA_PHASE2_STUB=0`) if the ECDSA signature is all-zero (defense-in-depth against a mis-provisioned SE shipping stub output). Add a boot self-test: sign a known vector, verify non-zero.
- **R-QUAL-6** — Magic sizes (`static uint8_t decoded[256]`, `256`-byte chunk) are duplicated; centralise as named constants tied to `sizeof(SignedEtPacketPi)`.
- **R-QUAL-7** — Document the threading model per node (which thread owns which member) at the top of each `*_node.cpp`. Several safety arguments currently rest on implicit single-thread invariants.
- **R-QUAL-8** — `fusion_node.cpp:312` `TODO`: pass original width/height in `DepthMap` instead of using depth-map aspect as a proxy (accuracy of ROI depth sampling).

---

## 5. Azure / IC-2027 transition deltas (forward-looking)

These are not bugs; they are the V2 architecture moves that the [roadmap](../../../../Downloads/VIGIA_IC2027_Roadmap_v2_WinOdds.pdf) depends on. Kept here so the edge repo tracks its own half of the migration.

- **R-AZ-1 — IoT Hub + DPS swap.** `hazard_uplink_node` and `anti_death_node` MQTT targets become Azure IoT Hub; device enrolment via **DPS X.509 attestation** using the ATECC608 identity. Keep the publish interface identical so it is a config + connect-string change, not a rewrite. mTLS, QoS-1, LWT all carry over.
- **R-AZ-2 — On-device anonymization stage.** Insert a face/plate blur model between `camera_node` and any path that emits frames off-device (BLE, uplink frame-hash). Required for DPDP full-compliance (May 13 2027). Because telemetry already transmits latent vectors not raw video, state this explicitly: raw frames never leave the node; only the blurred frame is ever hashed/exported.
- **R-AZ-3 — `phi_edge_node` (Foundry Local).** New ROS 2 node wrapping Foundry Local (Linux ARM64, ONNX Runtime 1.26) for on-device intent + canned-warning selection. Gate on the August spike (architecture A/B/C decision). Pin to ≤2 cores; reuse the Dynamic-Dims degradation pattern for a low-FPS "conversation mode."
- **R-AZ-4 — Store-and-forward (R-CRIT-1) is a prerequisite** for the offline-copilot story; build it before the Phi node so offline voice has data to reason over.

---

## 6. Priority-ordered work plan

| Order | ID | Item | Effort |
|---|---|---|---|
| 1 | R-QUAL-1 | Resolve CLAUDE.md conflict; commit stranded M8 worktree | mins |
| 2 | R-CRIT-1 | On-disk store-and-forward spool for continuous uplink | ~3 d |
| 3 | R-CRIT-3 | Delete or sign the legacy HTTPS path | ~1 d |
| 4 | R-CRIT-2 | Double-buffer / bounded emergency snapshot | ~3 d |
| 5 | R-BUG-1 | Correct COBS + round-trip test (firmware + Pi together) | ~2 d |
| 6 | R-SEC-1/2 | RNG hardening + handshake expiry/anti-flood | ~2 d |
| 7 | R-SEC-4 | Frame-hash into signed payload (cross-repo) | ~2 d |
| 8 | R-SEC-3, R-BUG-2, R-QUAL-2..8 | Hardening batch | ~3 d |
| 9 | R-AZ-1..4 | Azure migration (Nov window) | see roadmap |

**Definition of done for V2:** every P0 closed and verified end-to-end (antenna-pull test, load test, forged-POST test); every P1 closed or explicitly risk-accepted in writing; the edge node passes a hostile technical-review walkthrough of the full signing chain with no unsigned or lossy path.

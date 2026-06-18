# VIGIA App ↔ Dashcam Integration — Detailed Design Spec

**Status:** Design — not yet implemented
**Owner:** Edge / Robotics
**Scope:** The local BLE channel between the Pi 5 edge node (`vigia-raspi`) and the Android companion app (`vigia2`). Covers the GATT server, pairing/provisioning, authentication, telemetry transport, and the ROS2 wiring that feeds it.
**Audience:** Hardware, firmware, Android, and backend engineers.

> This document is the contract that makes "open the app, sit in the car, it just connects" real. Every gap that currently breaks that experience is enumerated, root-caused, and given a concrete, industry-aligned design.

---

## 0. TL;DR — What's Broken and What We're Building

The AWS/DePIN uplink (Ed25519 → API Gateway) is solid. The **entire local link between the Pi and the phone does not exist on the Pi side**, and the auth model that the app assumes is cryptographically impossible as written. This spec fixes both.

| # | Gap | Severity | Fix in this spec |
|---|-----|----------|------------------|
| G1 | No BLE GATT server on the Pi | 🔴 Blocker | §3 BlueZ peripheral node |
| G2 | HMAC shared-secret handshake is **impossible** — Android key is non-exportable StrongBox | 🔴 Blocker | §4 ECDH-derived session key (re-architect) |
| G3 | No secret/identity provisioning channel | 🔴 Blocker | §5 QR out-of-band bootstrap |
| G4 | RRI is threshold-gated; app needs continuous ≥1 Hz | 🟠 Major | §6 dedicated streamer node |
| G5 | FusionNode is dead without Pico IMU | 🟠 Major | §6.4 degraded-mode path |
| G6 | 512-D vector (2054 B) ≫ default ATT MTU (23 B) | 🟠 Major | §7 MTU/DLE/2M PHY tuning |
| G7 | Placeholder GATT UUIDs | 🟡 Minor | §3.2 final UUID allocation |
| G8 | BlueZ defaults to legacy pairing, app demands LE SC | 🟡 Minor | §4.1 BlueZ SC-only config |
| G9 | Fake geohash, unsynced clocks, empty latent until model export | 🟡 Minor | §8 data-quality fixes |

**Competition-grade enhancements (beyond gap-closing):** AirPods-style CDM auto-connect (§5.3), hardware-attested anti-spoof beacon for the DePIN moat (§6.6), and dynamic dimensionality scaling for automotive link resilience (§7.1). These move the spec from "it works" to "it wins."

---

## 1. System Context

```
┌──────────────────────────── Pi 5 (vigia-raspi) ────────────────────────────┐
│  CameraNode ─▶ VisionNode(YOLO INT8) ─┬─▶ /vigia/detections                 │
│                                       └─▶ /vigia/spatial_latent             │
│  Pico USB-CDC ─▶ SensorBridgeNode ─▶ /vigia/imu /vigia/gps                  │
│  DepthNode(MiDaS) ─▶ /vigia/depth                                           │
│                                                                            │
│  FusionNode ─▶ /vigia/hazard_event ──▶ AntiDeathNode ──▶ AWS (Ed25519) ✓   │
│                                                                            │
│  ┌──────────────── NEW: BleGattNode (this spec) ─────────────────┐         │
│  │  subscribes /vigia/spatial_latent + /vigia/detections + /gps   │         │
│  │  BlueZ peripheral: advertise, pair (LE SC), authenticate,      │         │
│  │  stream telemetry frames as GATT notifications                 │         │
│  └───────────────────────────────────────────────────────────────┘         │
└─────────────────────────────────────┬───────────────────────────────────────┘
                                       │ BLE (LE SC, 2M PHY, MTU≥517)
                                       ▼
┌──────────────────────────── Android (vigia2) ──────────────────────────────┐
│  BleLinkManager ─▶ BleDataStreamerImpl ─▶ ContextAggregator ─▶ Copilot/STT  │
│  KeystoreManager (StrongBox TEE)   CdmPresenceService (auto-start)          │
└────────────────────────────────────────────────────────────────────────────┘
```

The Pi is the **GATT peripheral/server**; the phone is the **GATT central/client**. The phone owns GNSS; the Pi owns vision/depth/IMU and the DePIN uplink.

---

## 2. Industry Baseline — What the Leaders Do

Conclusions drawn from BLE throughput engineering literature (Punch Through, Nordic, Memfault/Interrupt) and connected-dashcam / DePIN architectures (Hivemapper, Nexar, comma.ai, ADAS dash-cam research):

**Parity with the leaders — where VIGIA already lands:**

- **Hivemapper (Solana DePIN dashcam, $100M+ network):** dashcams ship with **no hardcoded BLE/Wi-Fi password**. The device acts as the local server and the phone scans a QR code that carries the public-key payload to negotiate the secure tunnel. Our §4 ECDH + §5 QR bootstrap is the **same flow** — VIGIA reaches parity with a production DePIN network.
- **Apple AirPods (the UX gold standard):** "magic" connect bypasses the Bluetooth settings menu via a proprietary `0x004C` manufacturer advertising packet that the iPhone intercepts to silently negotiate ECDH against the iCloud Keychain. We can't use Apple's packet, but Android's **CompanionDeviceManager (CDM) Presence Service** — already in the app's architecture — gives the equivalent: bond the resolvable private address to CDM and the app wakes from Doze and auto-connects the instant the driver enters the vehicle. See §5.3.
- **Nexar:** BLE for low-power metadata (like our RRI), Wi-Fi Direct only when pulling raw video. Confirms our "stream the embedding, not pixels" choice.
- **comma.ai:** fully self-contained Linux node, uploads over LTE, bypasses the phone for critical telemetry. This is the model for our **AWS DePIN uplink** (already built) — the phone link is a *convenience/UX* channel, not the safety-critical path. Architectural rule: BLE link loss must **never** compromise the AWS hazard uplink.

Conclusions:

1. **Edge-filters, streams summaries — not raw frames.** Nexar's model is "a connected sensor at the edge … devices filter and send only critical events." Our spatial latent vector + RRI is exactly this pattern: we ship a compact embedding, not pixels. Correct instinct — keep it.
2. **Throughput ceiling of GATT-over-BLE is ~170–180 kB/s** on 2M PHY with a maxed MTU and 244- or 495-byte writes. A 256-D frame (1030 B) at 5 Hz = ~5 kB/s — comfortably within budget. A 512-D frame (2054 B) at 10 Hz = ~20 kB/s — still fine. **GATT notifications are sufficient; we do NOT need L2CAP CoC** (its benefit over MTU-tuned GATT is ~1%).
3. **MTU + connection interval + PHY must be tuned together.** Default 23-byte MTU wastes the entire link. Negotiate MTU 517, enable 2M PHY, request a 15 ms connection interval.
4. **Auth = device identity + session key, never a shipped shared secret.** Modern practice (Azure IoT, embedded secure provisioning) is per-device asymmetric identity with out-of-band bootstrap (QR/NFC). A symmetric secret baked into both ends is an anti-pattern and — in our case — literally unbuildable (§4).
5. **Pairing is LE Secure Connections only.** Legacy pairing (BLE 4.0/4.1) is broken; the app already enforces SC. The Pi must match.

---

## 3. BLE GATT Server on the Pi (G1)

### 3.1 Stack choice

| Option | Verdict |
|--------|---------|
| **BlueZ via D-Bus GATT API** (`org.bluez.GattManager1`, `LEAdvertisingManager1`) | ✅ **Chosen.** Native to the Pi, no extra kernel modules, LE SC support since 5.48, fine-grained per-characteristic security flags. |
| Bare HCI / custom L2CAP socket | ❌ Reinvents pairing/bonding; loses SC. |
| NimBLE / external MCU | ❌ Pi 5 has a capable controller (`hci0`); no need for extra silicon. |

**Implementation:** a standalone helper process (`vigia_ble_server`) owns the BlueZ D-Bus objects, exposed to ROS2 through a thin `BleGattNode`. Rationale: BlueZ's GATT API is async D-Bus and pairs poorly with the ROS2 executor on the same thread. Two clean options:

- **3.1a (recommended):** `BleGattNode` (C++, in `vigia_edge_node`) runs a GLib `GMainLoop` on a dedicated `std::thread` (SCHED_OTHER — this is best-effort, not RT), using `sdbus-c++` or `gdbus`. ROS2 callbacks push the latest frame into a lock-free single-slot mailbox; the GLib thread reads it on its notify timer.
- **3.1b:** a separate Python process using `bluezero`/`dbus-next`, fed over a local Unix socket from a ROS2 node. Faster to prototype, heavier at runtime. Use only for bring-up.

> **Decision needed:** 3.1a (C++/sdbus, production) vs 3.1b (Python, prototype). Default to 3.1a; allow 3.1b behind a build flag for the first hardware bring-up.

### 3.2 GATT profile — final UUID allocation (G7)

Replace the placeholders in `core/sensor/src/main/kotlin/com/vigia/core/sensor/ble/GattConstants.kt`. Generate real 128-bit UUIDs with `uuidgen` and pin them in **both** repos. Proposed base UUID `8e7a0001-…` (placeholders below — regenerate before commit):

| Role | UUID (regenerate) | Properties | Security |
|------|-------------------|-----------|----------|
| `VIGIA_SERVICE` | `8e7axxxx-…` | Primary Service | — |
| `HANDSHAKE_CHAR` | `8e7axxxx-…` | Read · Write · Notify | Encrypted, authenticated (SC) |
| `TELEMETRY_CHAR` | `8e7axxxx-…` | Notify | Encrypted, authenticated (SC) |
| `CONTROL_CHAR` (new) | `8e7axxxx-…` | Write | Encrypted — phone→Pi (e.g. request 256-D vs 512-D, pause stream) |
| CCCD | `0x2902` | — | standard |

> **Why not keep `0000CAFE-…`?** Those are 16-bit-aliased into the SIG base range and risk collision with assigned numbers. Use a private random 128-bit base.

### 3.3 Advertising

- Advertise the `VIGIA_SERVICE` UUID + a short local name (`VIGIA-<last4 of device id>`).
- **General discoverable**, connectable, 100 ms advertising interval (fast reconnect).
- Use a **resolvable private address** bonded to the phone after first pairing so CDM (`CompanionDeviceManager`) can re-detect it while preserving privacy.
- The phone's `BleLinkManager.scanForDevice()` filters on a fixed MAC (`BLACKBOX_MAC`). **Pin a static identity address** on the Pi adapter (`btmgmt public-addr` or `hci0` static random) so the MAC is stable across reboots — the app will not connect to anything else.

---

## 4. Authentication — Re-architecture (G2, the hard one)

### 4.1 The fatal flaw in the current design

The app's `KeystoreManager` generates a 256-bit HMAC key with:

```kotlin
KeyGenParameterSpec.Builder(KEY_ALIAS, KeyProperties.PURPOSE_SIGN)
    .setKeySize(256)
    .setIsStrongBoxBacked(true)   // hardware TEE
```

**This key is non-exportable by design.** StrongBox `PURPOSE_SIGN` keys never leave the secure element — only the HMAC *output tag* is returned. Therefore the hardware-expectations doc's requirement — *"the hardware must hold the same 256-bit secret"* — **cannot be satisfied.** There is no API to read the bytes out, and importing the same bytes into StrongBox on the phone side is equally blocked. The symmetric handshake is a dead end.

### 4.2 The fix — mutual ECDH with out-of-band identity pinning

Switch from "shared symmetric secret" to **per-device asymmetric identity + ephemeral session key**. This is what Azure IoT / modern secure-provisioning stacks do, and it's buildable on both platforms.

**Identities (long-term):**
- **Pi:** reuse the existing **Ed25519 device key** (`/etc/vigia/device_ed25519.key`). For ECDH we derive an X25519 key from the same seed (libsodium `crypto_sign_ed25519_sk_to_curve25519`). One identity, two uses (sign for AWS, ECDH for the phone).
- **Phone:** generate a **P-256 (secp256r1) key pair in StrongBox** with `PURPOSE_AGREE_KEY` (Android supports ECDH key agreement in the TEE since API 31). The private key stays in hardware; only the public key and the agreement output are exposed.

> Note: X25519 (Pi) vs P-256 (Android Keystore) — Keystore ECDH only supports NIST curves in-hardware. Two clean resolutions:
> - **4.2a (recommended):** Use **P-256 on both ends.** The Pi generates a P-256 identity (mbedTLS, already an optional dep) *in addition to* Ed25519. ECDH = `ECDH(P-256)`. Fully hardware-backed on the phone.
> - **4.2b:** Use X25519 on both ends; phone does X25519 in software (BouncyCastle), losing TEE protection of the agreement. Avoid.
>
> **Decision: 4.2a — P-256 ECDH, hardware-backed both sides.**

**Handshake (replaces the 4-step HMAC flow, same wire framing):**

| Step | Direction | Bytes |
|------|-----------|-------|
| 1 HELLO | Phone → Pi | `[0x01]` |
| 2 CHALLENGE | Pi → Phone | `[0x02 | nonce(32) | Pi_pub_P256(65) | Ed25519_sig_over(nonce‖Pi_pub)(64)]` |
| 3 RESPONSE | Phone → Pi | `[0x03 | nonce_phone(32) | Phone_pub_P256(65) | StrongBox_sig(...)]` |
| 4 CONFIRM | Pi → Phone | `[0x04 | HMAC(session_key, "VIGIA-CONFIRM"‖both_nonces)]` on success, `[0xFF]` on fail |

- **Session key:** `K = HKDF-SHA256( ECDH(Pi_priv, Phone_pub), salt = nonce‖nonce_phone, info = "vigia-ble-v1" )`.
- Both sides verify the peer's signature over the ephemeral material → **mutual authentication + forward secrecy** per session.
- The phone pins `Pi_pub` after the QR bootstrap (§5); the Pi pins `Phone_pub` on first bond. MITM is prevented because the QR carries the Pi's public key out-of-band.
- After step 4, derive per-frame keys if needed; for telemetry confidentiality we already have link-layer LE SC encryption, so the session key's role is **authentication + replay protection**, not a second encryption layer.

**Replay/freshness:** every CHALLENGE nonce is single-use; the Pi rejects a RESPONSE whose nonce it didn't just issue, with a 10 s validity window (matches the app's `timeoutMs = 10_000`).

### 4.3 BlueZ LE Secure Connections config (G8)

`/etc/bluetooth/main.conf`:

```ini
[General]
JustWorksRepairing = never
[GATT]
# enforce SC; reject legacy pairing
[Policy]
```

Set the adapter to **SC-only** and **bondable**, with IO capability `KeyboardDisplay` to drive Numeric Comparison (matches the app's `device.createBond()` LE SC flow). Pairing agent registered by `vigia_ble_server`. Reject any pairing that negotiates down to legacy.

---

## 5. Provisioning / Bootstrap (G3)

The phone must learn the Pi's static BLE MAC and its pinned P-256 public key **before** the first connection. Industry standard = out-of-band channel; we use a **QR code**.

**Bootstrap flow (one-time, at install / pairing):**

1. Pi generates (or already has) its identity keys and computes a bootstrap blob:
   `vigia://pair?mac=<static_addr>&pk=<base64url P-256 pub>&id=<device_id>&v=1`
2. The blob is rendered as a QR code — printed on the device label **and** available via a local setup endpoint (`vigia_ble_server --print-qr`).
3. The app scans the QR (existing camera permission), parses it, stores `mac → BlackboxConfig`, pins `pk` for handshake step 2 verification, and triggers the CDM association flow with that MAC.
4. First BLE connect → LE SC bond → ECDH handshake (peer keys already pinned) → `associationId` stored.

> **Decision needed:** QR on a printed label (factory) vs QR shown on a one-time web/setup page served by the Pi. Recommend **both**: factory label for production, `--print-qr` for dev units. NFC is a future option (the patent literature favors it) but adds hardware cost now.

**Key rotation:** support a `v=` version field and a `CONTROL_CHAR` re-key command so a compromised pairing can be revoked without re-flashing.

### 5.3 "AirPods-magic" auto-connect via CDM (UX innovation)

The QR scan happens **once**. Every drive after that must feel like AirPods: open the app (or don't), sit in the car, it's connected — zero menus.

- After the QR bootstrap, register the bonded **resolvable private address** with `CompanionDeviceManager` via `CdmPresenceService` (already in the app).
- CDM wakes the app from Doze and starts `VigiaForegroundService` (typed `connectedDevice`) the moment the Pi's advertisement comes in range — no polling, no Settings screen, no 6-hour background limit.
- The Pi advertises at a fast 100 ms interval (§3.3) so the cold-start reconnect from "driver opens door" to "Bound + streaming" completes in **< 2 s**.
- **Pitch framing (SFT):** "AirPods-level simplicity for commercial trucking — scan once, then it just works, every trip." This is the zero-config installation story (D3) made concrete.

---

## 6. Telemetry Streaming — Decouple from the Hazard Gate (G4, G5)

### 6.1 The problem

`FusionNode::fuse_and_maybe_publish()` only emits a `HazardEvent` when `rri >= 0.75`, and only when `latest_imu_` exists. The app's `ContextAggregator` expects **continuous ≥1 Hz** telemetry (RRI + latent), even when RRI is low, to keep `VigiaSearchContext` fresh. Gating on hazards starves the app.

### 6.2 The fix — `BleGattNode` streams independently

`BleGattNode` (the same node hosting the GATT server) subscribes **directly** to the inference topics, not to `/vigia/hazard_event`:

- `/vigia/spatial_latent` (`vigia::qos::inference_results()`) → latent vector
- `/vigia/detections` → derive a lightweight **continuous RRI proxy** (max detection confidence, or a cheap RRI computed without the IMU/depth terms)
- `/vigia/gps` → optional, for cross-check (the phone uses its own GNSS as primary)

It re-emits a telemetry frame at a fixed cadence (default **5 Hz**, configurable) regardless of hazard threshold. This is the Nexar "always-on sensor, filter downstream" pattern.

### 6.3 Continuous RRI vs hazard RRI

Two distinct RRI consumers now exist:
- **Hazard RRI** (FusionNode, gated ≥0.75) → AWS DePIN uplink. Unchanged.
- **Stream RRI** (BleGattNode, continuous) → phone context. New.

Factor the RRI math so both share one implementation. Extract `compute_rri()` into a header (`vigia_rri.hpp`) callable with optional IMU/depth — when those are absent it degrades to `w_yolo * yolo_conf` renormalized. This kills two birds: continuous streaming *and* the G5 "dead without IMU" problem.

### 6.4 Degraded-mode operation (G5)

When the Pico is in STUB mode (no IMU/GPS), the system must still be useful:

| Input available | RRI terms used | Behavior |
|-----------------|----------------|----------|
| Vision only | `w_yolo` (renormalized to 1.0) | Stream RRI works; no ISS; no hazard uplink |
| Vision + Depth | `w_yolo + w_geometry` | Adds geometry confidence |
| Vision + Depth + IMU + GPS | full fusion | Hazard uplink enabled |

`FusionNode` must **not** early-return on `!latest_imu_` — instead publish with the terms it has, and set a `degraded` flag on `HazardEvent`. The BLE stream never depends on the IMU at all.

### 6.5 Frame encoder (matches `BleDataStreamerImpl`)

Wire format is already pinned by the app; the Pi must emit it exactly (little-endian):

```
[0]      uint8    version = 0x01
[1..4]   float32  RRI in [0,1]          (LE)
[5]      uint8    dims code: 0x00=256-D, 0x01=512-D
[6..]    float32[dims]  latent vector   (LE)
```

- 256-D → 1030 bytes; 512-D → 2054 bytes.
- **RRI clamp:** the app silently drops frames outside `[0,1]` — encoder must `clamp(rri,0,1)`.
- **Dims selection:** default 256-D for BLE efficiency; phone can request 512-D via `CONTROL_CHAR`. The model's true latent dim must be projected/truncated to 256 or 512 (see §8).
- **Endianness:** Pi (aarch64) is little-endian and so is the app's decoder — direct `memcpy` is correct; assert at build time.

### 6.6 Anti-spoofing — hardware-attested telemetry (DePIN moat)

**Threat:** a DePIN reward network is attacked by **location/data spoofing** — a mocked Android emulator fabricates pothole frames to farm tokens. The cloud must be able to prove a frame came from real VIGIA silicon, not a software fake.

**Mechanism:** bind telemetry to the hardware secure element (ATECC608A on the Pico) by signing a compact attestation tuple `(device_id ‖ frame_seq ‖ RRI ‖ rolling_hash)`. The phone forwards the attestation to the Azure/AWS backend, which verifies it against the registered device public key — exactly the trust model the AWS Ed25519 uplink already uses, now extended to the phone path.

**Engineering reality — do NOT sign every frame.** The ATECC608A lives on the Pico, reachable only over USB-CDC, and an ECDSA-P256 sign is ~50–100 ms round-trip. At 5–10 Hz that would throttle the entire stream and add a 64-byte signature to every 1030-byte frame (~6% overhead, plus serialization stalls). Instead use a **signed rolling beacon**:

- Every **N frames** (e.g. 1 Hz, N=5 at 5 Hz stream) the Pi emits an attestation over `CONTROL_CHAR` / a dedicated `ATTEST_CHAR`: `[device_id, frame_seq, sha256(last_N_frames), ecdsa_sig(64)]`.
- The signature covers a **hash chain** of the intervening frames, so a spoofer can't splice fake frames between beacons without breaking the chain.
- `frame_seq` is monotonic → replay of an old beacon is detectable.
- Until the ATECC608A is wired (Ben's task), run with `VIGIA_PHASE2_STUB` — the beacon is emitted with a zero signature and the backend treats the stream as *unattested* (still usable for UX, just not reward-eligible). This makes the anti-spoof path **incremental**, not a hard dependency for first connection.

**Pitch framing (Imagine Cup):** "Every rewarded pothole is cryptographically attested by a hardware secure element — VIGIA's DePIN data is spoof-resistant by construction." This is the security moat.

---

## 7. Link Tuning — Make It Fast and Reliable (G6)

| Parameter | Default | Target | Where |
|-----------|---------|--------|-------|
| ATT MTU | 23 B | **517 B** | Phone calls `gatt.requestMtu(517)` **before** `discoverServices()`; Pi honors via BlueZ |
| PHY | 1M | **2M** | `gatt.setPreferredPhy(PHY_LE_2M)` (phone), BlueZ accepts |
| Connection interval | ~50 ms | **15 ms** | Pi sends a Connection Parameter Update Request |
| Data Length Ext | off | **251 B** | Enabled by controller; verify on `hci0` |
| Notification, not Indication | — | Notify | Already correct in profile |

**Fragmentation:** a 2054-byte 512-D frame spans multiple ATT packets even at MTU 517. Two options:
- **7a (recommended):** MTU 517 + GATT handles ATT-level fragmentation; one `notify` per logical frame. Simpler; app's `onCharacteristicChanged` already receives the reassembled value.
- **7b:** App-level chunking with a 2-byte `[seq|total]` header if any stack caps notify size. Keep as fallback.

**Throughput sanity:** 512-D @ 10 Hz = 20.5 kB/s; ceiling on 2M PHY ≈ 175 kB/s. **~12% utilization** — ample headroom. 256-D @ 5 Hz = 5 kB/s. No need for L2CAP CoC.

**Required app-side change:** `BleLinkManager` currently never negotiates MTU or PHY. Add an `requestMtu(517)` step between `connectGatt()` and `discoverServices()`, await `onMtuChanged`, then `setPreferredPhy`. Document in the app repo.

### 7.1 Dynamic Dimensionality Scaling (automotive resilience)

The math says 2M PHY has ample headroom, but **BLE in a car is lossy** — alternator EMI, infotainment radios, and the metal cabin all degrade the link. A fixed 512-D @ 10 Hz stream will stutter exactly when the car is running. The `BleGattNode` must adapt instead of dropping the link:

| Link health (from BlueZ stats) | Stream mode |
|--------------------------------|-------------|
| Healthy (low retransmit, RSSI strong) | 512-D @ 10 Hz |
| Degraded (rising retransmits / dropped notifies) | **downgrade to 256-D @ 5 Hz** |
| Poor (sustained loss, weak RSSI) | **RRI-only beacon** — drop the latent vector, keep the 1 Hz attestation + RRI so the app's context never goes fully stale |
| Recovered | step back up after a hysteresis window |

- Link health is read from BlueZ (`org.bluez.Device1` RSSI, connection stats) and/or an app→Pi NACK over `CONTROL_CHAR`.
- The phone advertises which `dims_code` it currently expects; the Pi confirms the active mode in the frame header so the decoder never desyncs.
- **Note on wire format:** "RRI-only" needs a sentinel. Reserve `dims_code = 0xFF = RRI-only (no vector)` and teach `BleDataStreamerImpl` to accept it (currently it rejects unknown codes). This is the one **forward-compatible app change** required for graceful degradation.

**Pitch framing:** "VIGIA degrades gracefully — it never drops the driver's safety context, it just sends less detail when the radio environment is hostile." Resilience as a feature.

---

## 8. Data-Quality Fixes (G9)

1. **Latent dimension projection.** YOLO's penultimate feature map is not 256/512-D flat — it's e.g. `[1, C, H, W]`. `VisionNode` must either: (a) global-average-pool to `C`, then a fixed linear projection to 256/512, or (b) export the model with a projection head. Until the model is exported and `latent_layer_name` is set via Netron, the frame's vector is **empty** — the app receives a zero vector and `VigiaSearch` context is meaningless. **Action: define the projection in the ONNX export, not on the Pi**, so the embedding is trained, not arbitrary.
2. **Geohash.** `anti_death_node.cpp` writes `"%.4f_%.4f"` truncated to 7 chars — not a valid geohash. Either link a real geohash encoder (`libgeohash`, ~100 LOC) or **drop the field** and let the server compute it from lat/lon (it already does). Recommend: drop it from the Pi payload to avoid a malformed field.
3. **Clock sync.** `originTimestampMs` is stamped with the phone's clock in `BleDataStreamerImpl`. The Pi has no NTP guarantee in a car. Add a Pi monotonic `frame_seq` (uint32) to the frame and let the phone map it to its own clock — staleness becomes measurable. (Optional v2 field; keep v1 wire format frozen.)
4. **GPS source duplication.** Phone GNSS is primary for the app's context; Pi GNSS is only for hazard coordinates in the AWS uplink. Document this explicitly so no one "fixes" the redundancy by mistake — they serve different consumers.

---

## 9. ROS2 Wiring Summary

New node: **`BleGattNode`** in `vigia_edge_node`.

| Aspect | Value |
|--------|-------|
| Subscribes | `/vigia/spatial_latent`, `/vigia/detections`, `/vigia/gps` (optional) |
| Publishes | none (terminal sink to BLE) |
| Threads | ROS2 callback (cache latest) + GLib `GMainLoop` (D-Bus/notify) on a separate `std::thread` |
| Scheduling | SCHED_OTHER — best-effort; **must not** contend with RT vision/fusion threads |
| Mailbox | single-slot lock-free `std::atomic` swap of the latest encoded frame |
| Params | `stream_hz` (5.0), `default_dims` (256), `ble_adapter` (`hci0`), `device_id` |
| Build deps | `sdbus-c++` (or `gdbus`/GLib), `libsodium` (already), `mbedTLS` (P-256, already optional) |

**CMake:** add `find_package(PkgConfig)` → `pkg_check_modules(SDBUS sdbus-c++)`; gate `BleGattNode` behind `HAVE_SDBUS` like the other optional deps. Falls back to "not built" with a warning, mirroring the existing ORT/mbedTLS pattern.

---

## 10. Failure Modes & Recovery (robotics-grade)

| Failure | Detection | Recovery |
|---------|-----------|----------|
| Phone out of range | BlueZ `Disconnected` D-Bus signal | Resume advertising; CDM re-detects on return |
| Pairing downgraded to legacy | SC flag absent on bond | Reject bond, log, re-advertise |
| MTU negotiation fails | `onMtuChanged` ≤ 23 | Fall back to 256-D + app-level chunking (7b) |
| Handshake nonce expired | >10 s since CHALLENGE | Return `[0xFF]`, phone retries from HELLO |
| Latent vector empty (model not exported) | `latent_vector.size()==0` | Stream RRI-only frames (dims=0 sentinel)? **No** — app rejects unknown dims. Instead: don't start streaming until a valid latent arrives; log clearly |
| BlueZ crash | D-Bus connection lost | `vigia_ble_server` supervised by systemd `Restart=on-failure`; ROS2 node tolerates absent server |
| Two phones connect | second central connects | Peripheral accepts one bonded central; reject others by bond identity |

---

## 11. Implementation Plan (ordered)

1. **Finalize UUIDs** (`uuidgen`) → update `GattConstants.kt` + Pi config. *(1 h)*
2. **`vigia_ble_server` skeleton** — BlueZ advertising + GATT service + LE SC bonding, no auth yet, echoes a static telemetry frame. Verify the app reaches `Bound` against a stub handshake. *(2–3 d)*
3. **ECDH handshake** (§4) on both ends — Pi P-256 identity, phone `PURPOSE_AGREE_KEY`, HKDF session key, mutual sig verify. *(3–4 d, cross-team)*
4. **QR bootstrap** (§5) — Pi `--print-qr`, app scanner + pinning. *(2 d)*
5. **`BleGattNode`** — ROS2 subscriptions + mailbox + 5 Hz encoder (§6). *(2 d)*
6. **`compute_rri()` refactor** into `vigia_rri.hpp`; FusionNode degraded mode (§6.4). *(1 d)*
7. **Link tuning** — app MTU 517 / 2M PHY / 15 ms (§7). *(1 d)*
8. **CDM auto-connect** — bond resolvable address to `CdmPresenceService`, verify Doze wake + <2 s reconnect (§5.3). *(2 d)*
9. **Dynamic dimensionality scaling** — link-health monitor + 512/256/RRI-only modes; `dims_code=0xFF` sentinel in app decoder (§7.1). *(2 d)*
10. **Anti-spoof beacon** — hash-chained attestation over `ATTEST_CHAR`; stub-signed until ATECC608A wired (§6.6). *(2 d, gated on SE)*
11. **Data-quality** — latent projection in ONNX export, drop fake geohash (§8). *(with model export)*
12. **Failure-mode tests** (§10) on the bench before vehicle trials.

**Competition-critical subset:** steps 1–5 + 8 give the live "scan once, auto-connect, streaming" demo. Steps 6, 9, 10 are the resilience + security narrative (degraded mode, dynamic scaling, hardware attestation) that differentiate the pitch.

**Critical path / cross-team:** steps 1 → 2 → 3 unblock everything. Step 3 (auth re-architecture) is the long pole and needs Android + edge engineers in the same room. The auth decision (§4.2a) and provisioning decision (§5) should be ratified before step 3 starts.

---

## 12. Decisions — RATIFIED

All six decisions signed off (design review, June 2026). Rationale tied to SFT / Imagine Cup enterprise-grade criteria.

| ID | Decision | **Ratified** | Why |
|----|----------|--------------|-----|
| D1 | GATT server impl | **C++/sdbus** | Python/bluezero is too brittle for automotive + Pi 5 thermal-throttle limits. Python only behind a bring-up flag. |
| D2 | Auth curve | **P-256 (4.2a)** | Hardware-backed TEE execution on Android — a security selling point for Imagine Cup. |
| D3 | Provisioning | **QR label** + `--print-qr` | NFC too costly for v1 BOM. Enables zero-config "scan once" UX (§5.3). |
| D4 | Default latent dims | **256-D** | BLE in-car is lossy; prioritize bandwidth headroom. 512-D on request when link is healthy (§7.1). |
| D5 | Geohash | **Drop the field** | Cloud computes it instantly; don't spend edge compute. |
| D6 | UUIDs | **New random 128-bit base** | Colliding with SIG 16-bit-aliased bases can crash the Android BLE stack in a live demo. |

---

## 13. References

- [BLE ATT MTU, DLE, and Message Sizing — Punch Through](https://punchthrough.com/ble-att-mtu-throughput/) — 244/495-byte optimal payloads, ~175 kB/s ceiling
- [Maximizing BLE Throughput Part 4 — Punch Through](https://punchthrough.com/ble-throughput-part-4/) — combined MTU/interval/PHY tuning
- [1M vs 2M vs Coded PHY — Punch Through](https://punchthrough.com/ble-phy-throughput-2/) — 2M ≈ 77% faster, sensitivity tradeoff
- [A Practical Guide to BLE Throughput — Memfault/Interrupt](https://interrupt.memfault.com/blog/ble-throughput-primer)
- [L2CAP CoC over GATT — Bluetooth Demystified](https://medium.com/bluetooth-demystified/performance-boost-using-l2cap-socket-over-gatt-for-bluetooth-data-traffic-2ef42cd6dfcf) — ~1% gain, not worth complexity
- [Creating a BLE Peripheral with BlueZ — Punch Through](https://punchthrough.com/creating-a-ble-peripheral-with-bluez/)
- [moovel/gatt-server — standalone C/C++ BlueZ GATT server](https://github.com/moovel/gatt-server)
- [Why Secure Provisioning Is Critical for Embedded Systems — Embedded Computing Design](https://embeddedcomputing.com/technology/security/why-secure-provisioning-is-critical-for-embedded-systems)
- [Securing IoT Devices — Manufacturing and Provisioning — Mutually Human](https://www.mutuallyhuman.com/securing-iot-devices-manufacturing-and-provisioning/)
- [Nexar — Edge-to-Edge OS for Autonomous AI](https://www.nexar-ai.com/) — edge-filter-and-stream model
- Hivemapper — Solana DePIN dashcam network; QR-bootstrapped device-as-server pairing (parity reference for §4/§5)
- Apple AirPods proximity pairing (`0x004C` manufacturer data) — UX reference for CDM auto-connect (§5.3)
- [AI-CDA4All: Cooperative Autonomous Driving via Dash-cam Hardware](https://arxiv.org/pdf/2505.06749)

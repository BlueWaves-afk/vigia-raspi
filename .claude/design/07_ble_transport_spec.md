# VIGIA BLE GATT Transport Specification

**Revision:** 1.0  
**Status:** Draft  
**Covers:** `ble_gatt_node.cpp`, `vigia_ecdh.hpp`, `vigia_msgs/SpatialLatent`

---

## 1. Overview

The VIGIA edge node exposes a GATT server over Bluetooth LE (BlueZ 5 via sdbus-c++ v2). A companion iOS/Android dashcam app connects, performs an ECDH key exchange, and receives encrypted real-time telemetry.

**Design goals:**
- Zero-copy path from SHM ring → BLE characteristic notification
- End-to-end encryption (ECDH + HKDF + HMAC-MAC) — no plaintext on the radio
- Graceful degradation: if ATECC608A is not wired, `attest` characteristic sends all-zero signature (app displays "unverified")

---

## 2. GATT Service Layout

**Service UUID:** `6e400001-b5a3-f393-e0a9-e50e24dcca9e` (Nordic UART–style namespace, VIGIA-assigned)

| Characteristic | UUID (last 12 hex digits) | Properties | Description |
|----------------|--------------------------|------------|-------------|
| `telemetry`    | `…e0a9-e50e24dcca9e`     | Notify     | Encrypted `DimsFrame` (~200 B per frame) |
| `handshake`    | `…e0a9-e50e24dcca9f`     | Write      | App writes ECDH public key (65 B) |
| `control`      | `…e0a9-e50e24dccaa0`     | Write      | App writes control commands (JSON, max 512 B) |
| `attest`       | `…e0a9-e50e24dccaa1`     | Read/Notify | `et_hash[32] ‖ ecdsa_sig[64]` (96 B raw) |

Full UUIDs use the base `6e400001-b5a3-f393-e0a9-` prefix with the last segment substituted per row above.

---

## 3. ECDH Handshake Protocol

### 3.1 Sequence

```
App                                Pi (BleGattNode)
 |                                      |
 |--- Write(handshake, app_pub_65B) --->|  app sends uncompressed P-256 public key
 |                                      |  Pi calls VigiaIdentityKey::ecdh_shared_secret(app_pub)
 |                                      |  shared = ECDH(pi_priv, app_pub)  [32 B raw]
 |                                      |  session_key = HKDF-SHA256(shared, salt="vigia-ble-v1")
 |<-- Notify(telemetry, ack_frame) -----|  Pi begins encrypted telemetry
```

### 3.2 Key Derivation

```
IKM  = raw ECDH shared secret (32 bytes)
Salt = "vigia-ble-v1" (ASCII, 12 bytes)
Info = "" (empty)
L    = 32 bytes

session_key = HKDF-SHA256(IKM, Salt, Info, L=32)
```

Implemented in `vigia::hkdf_sha256()` (`vigia_ecdh.hpp`).

### 3.3 Frame Encryption

Each telemetry notification is:

```
[4 B seq_counter] [N B HMAC-SHA256(session_key, seq_counter ‖ plaintext)[16B trunc]] [N B plaintext]
```

Where `plaintext` is the `DimsFrame` struct defined in §4.

HMAC truncation: first 16 bytes of HMAC-SHA256. Replay protection: seq_counter must be strictly monotonic; app rejects frames with seq ≤ last_seen.

---

## 4. DimsFrame Wire Format

Sent as binary (msgpack-like packing, but custom, defined in `vigia::ble::encode_frame()`).

| Offset | Size | Type | Field |
|--------|------|------|-------|
| 0      | 1    | u8   | `magic` = `0xD1` |
| 1      | 1    | u8   | `version` = `0x01` |
| 2      | 4    | u32  | `frame_id` |
| 6      | 8    | u64  | `timestamp_us` |
| 14     | 4    | f32  | `rri_score` |
| 18     | 4    | f32  | `iss_score` |
| 22     | 1    | u8   | `degraded` flag |
| 23     | 1    | u8   | `rri_tier` |
| 24     | 2    | u16  | `latent_dims` (number of float32 elements that follow) |
| 26     | 4×N  | f32[] | `latent_vector[N]` (N = latent_dims, typically 256) |

Total for 256-D latent: 26 + 1024 = **1050 bytes** (before encryption overhead).

BLE MTU default: 23 bytes. App must negotiate MTU ≥ 1100 (BLE 5 allows up to 512 B per ATT PDU; fragmented by BlueZ GATT layer automatically for larger payloads).

---

## 5. Attest Characteristic

**Format:** 96 bytes raw

```
[et_hash:32][ecdsa_sig:64]
```

- `et_hash`: SHA-256 hash of the 96-byte EtHashInput struct (firmware-computed, doc 03 §6.3.2)
- `ecdsa_sig`: 64-byte raw R‖S ECDSA signature (ATECC608A secp256r1, P1363 format)

When `VIGIA_PHASE2_STUB=1` (SE not wired), both fields are all-zeros. The app should display "Unverified sensor" in this state.

App verification:
1. Fetch device certificate from `GET /v1/device/{device_id}/cert` (future endpoint)
2. Reconstruct EtHashInput locally (using field values from the telemetry frame)
3. Verify SHA-256(EtHashInput) == et_hash
4. Verify ECDSA(cert.pubkey, et_hash, ecdsa_sig) is valid

---

## 6. Control Characteristic

App writes JSON commands (UTF-8, max 512 bytes). Supported commands:

```json
{"cmd": "set_rri_threshold", "value": 0.6}
{"cmd": "ping"}
{"cmd": "reset_seq"}
```

BleGattNode responds via a telemetry frame with `rri_score = -1.0` as ACK signal (TODO: define proper response characteristic).

---

## 7. Pi Identity Key

- Path: `/etc/vigia/pi_p256.der` (PKCS#8 DER, unencrypted)
- Generated once at provisioning: `tools/vigia-pair-qr.py --provision`
- Pi signs handshake challenges with this key (future: challenge-response flow)
- Public key exported as 65-byte uncompressed P-256 point via QR code during pairing

---

## 8. Open Items

| # | Item | Status |
|---|------|--------|
| A | App ECDH implementation | Not started |
| B | Challenge-response app authentication | Not designed |
| C | Response characteristic for control ACKs | Not designed |
| D | BLE advertisement payload (device_id, firmware version) | Stubbed |
| E | Connection security mode (BLE pairing vs. application-layer ECDH) | Application-layer chosen; BLE pairing disabled |

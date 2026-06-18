# Telemetry Signing

Two independent cryptographic layers protect event integrity:

1. **ECDSA packet signatures** — embedded in each signed-et binary frame by the
   Pico's ATECC608A hardware security element (HSE).
2. **HMAC-SHA256 event envelopes** — added by the Pi before HTTP upload to the
   server ingest endpoint.

---

## Layer 1 — ECDSA Packet Signatures (ATECC608A / secp256r1)

### On-device signing (Pico firmware)

- `atecc608a_driver.c` uses the CryptoAuthLib I²C HAL (`atca_hal_pico_i2c.c`)
  to call `atcab_sign()` on a 32-byte `et_hash`.
- The resulting raw `R ∥ S` signature (64 bytes) is stored in
  `SignedEtPacketView.ecdsa_sig[64]`.
- Stub signature (all-zero sig) is emitted in development when the HSE is absent.

### On-host verification (Pi / `EcdsaVerifier`)

```
EcdsaVerifier::Config
    pubkey_file       path to 128-hex-char (64-byte X∥Y secp256r1 key)
    allow_stub_sig    true → accept all-zero sig as valid (dev mode only)
```

**`loadPublicKey()`**
- Opens the hex file, strips whitespace / colons, asserts exactly 128 hex chars.
- Parses into `pubkey_[64]` (X ∥ Y uncompressed point).

**`verify(hash[32], sig[64])`**
- Requires `VIGIA_HAVE_OPENSSL`. Without it, always returns `false`.
- Zero-checks `hash` and `sig` pointers before touching them.
- Constructs `EC_KEY` from raw X∥Y via `EC_KEY_set_public_key_affine_coordinates`.
- Calls `ECDSA_do_verify()` with the R∥S big-endian integers.
- Returns `true` only when OpenSSL returns 1.

**Stub signature path**: `isStubSignature()` checks all-zero — accepted when
`allow_stub_sig = true`, otherwise rejected. This prevents a zero-filled
firmware build from passing verification in production.

---

## Layer 2 — HMAC-SHA256 Event Envelopes (`EventSigner`)

### Key loading

```
EventSigner::Config
    hmac_key_file   path to a UTF-8 file containing the raw HMAC secret
```

`loadKey()` reads the file in binary mode, strips trailing CR/LF.
An empty key disables signing — `hasKey()` returns false.

### Canonical payload

Keys are sorted alphabetically (matching Python `json.dumps(sort_keys=True)`):

```json
{
  "device_id": "<str>",
  "device_seq": <u64>,
  "event_id": "<uuid>",
  "hazard": { "bbox": [...], "frame_index": ..., "geometry_conf": ...,
              "iss": ..., "rri": ..., "temporal_conf": ..., "yolo_conf": ... },
  "hazard_class": <u8>,
  "location": { "lat": <f64, prec=10>, "lon": <f64, prec=10> },
  "motion": { "fix_type": ..., "hdop": ..., "speed_mps": ... },
  "observed_at": "<ISO-8601>"
}
```

> **Precision fix June 2026**: lat/lon use `std::setprecision(10)` in
> `canonicalPayload()` and `signEnvelope()`. Previous default (6 sig figs)
> rounded coordinates to ~10 m resolution, causing HMAC mismatches when the
> server re-computed signatures with full double precision.

> **Buffer safety fix June 2026**: `device_id` (a `char[32]` that may not be
> null-terminated) is now copied to a `char[33]{}` with explicit
> `memcpy` before being written to `ostringstream`.

### Envelope JSON (transmitted)

```json
{
  "event_id": "...",
  "device_id": "...",
  "device_seq": ...,
  "observed_at": "...",
  "hazard_class": ...,
  "location": { ... },
  "hazard": { ... },
  "motion": { ... },
  "payload_hash": "<sha256 of canonical>",
  "signature": "<base64 HMAC-SHA256 of canonical>",
  "signed_et": { "valid": true, "sequence": ..., "hash": "<hex>" } | null
}
```

Server verification: strip `payload_hash` and `signature`, sort keys, re-hash →
compare HMAC.

---

## Telemetry Validator Gate

The SKILL.md at `.claude/skills/telemetry-validator/SKILL.md` requires
`make validate-buffers` to pass before any telemetry change is declared
complete. This target compiles and runs `sensor_bridge_test` and
`cobs_roundtrip_test` against the live source tree — both exercise all
encoding/decoding/signing code paths.

Backlinks: [[02_Data_Ingestion_Layer]] | [[05_Event_Pipeline]] | [[00_Index]]

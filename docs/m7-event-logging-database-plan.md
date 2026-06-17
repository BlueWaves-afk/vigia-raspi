# VIGIA M7: Event Logging + Database + Hazard Map

**Document version:** 1.1  
**Date:** 2026-06-18  
**Project:** VIGIA ADAS Edge Node (vigia-raspi)  
**Status:** Implemented — see `docs/m7-event-logging-implementation.md`  
**Prerequisite:** Milestone M6 multimodal sensor fusion (complete)

---

## Executive summary

M7 turns fused pothole detections into a **durable, queryable hazard platform** — not a flat log of every frame. Each promoted detection becomes a versioned **observation**, syncs to a **PostGIS server**, clusters into **hazard entities**, and feeds a **hazard map UI**.

**Hardware baseline (current BOM):** No NVMe SSD. Pi buffers events in RAM; the **server database is the source of truth**. SD card is boot media only — not for event logging.

---

## 1. Current state

| Area | Status |
|------|--------|
| Multimodal fusion (RRI + ISS + GPS) | Done — `src/fusion.cpp`, `src/coordinator.cpp` |
| Structured hazard output in production | Done — `EventPromoter` + `EventStore` in `vigia_app` |
| Event persistence / database | Done — PostGIS + FastAPI ingest |
| Hazard map API / UI | Done — `/v1/hazards`, `/map` |
| Secure cloud ingest | Done — HMAC + anti-replay (mTLS deferred) |

**Related design references in repo:**

- `.claude/design/02_ros2_node_contracts.md` — `HazardEvent.msg` schema
- `docs/hardware-architecture.md` — JSON uplink, mTLS, long-term NVMe target
- `.claude/design/05_anti_death_and_depin_contracts.md` — server attestation pipeline
- `docs/sensor-fusion-plan.md` — milestone roadmap (M0–M7)

---

## 2. Why not a simple event log?

Industry road-hazard systems separate three layers:

| Layer | Purpose | Without it |
|-------|---------|------------|
| **Observation** | Per-detection signal | 30 Hz YOLO → thousands of duplicate rows |
| **Hazard entity** | Deduplicated geo defect | Map becomes unusable noise |
| **Projection** | Map, heatmaps, dashboards | DB rows never become user value |

**Pipeline:** observation → promotion → persistence → projection

---

## 3. Architecture overview

```
┌─────────────────────────────────────────────────────────────┐
│  Raspberry Pi 5 (edge) — vigia_app                          │
│  midasLoop (full fusion) → EventPromoter → in-process ring  │
│                      │                                      │
│                      └── Sync thread (HTTPS signed batches) │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               v
┌─────────────────────────────────────────────────────────────┐
│  Server (canonical store)                                   │
│  Ingest API → observations (append-only)                    │
│            → hazard_entities (aggregated, PostGIS)          │
│                      │                                      │
│                      └── Hazard map API + MapLibre UI       │
└─────────────────────────────────────────────────────────────┘
```

**Critical wiring rule:** EventPromoter runs **only** from `Coordinator::midasLoop` after full geometry + temporal fusion — **never** from the YOLO-only sync path in `processFrame()`.

---

## 4. Storage strategy (no NVMe in BOM)

### 4.1 Key concepts

| Component | What it is | Survives power loss? | Cost |
|-----------|------------|----------------------|------|
| In-process event ring | Pre-allocated SPSC queue in `vigia_app` | **No** | $0 |
| `/dev/shm` | Optional tmpfs for cross-process sync (not M7.0 default) | **No** | $0 |
| SD card | Pi boot media | Yes | Already in BOM |
| NVMe HAT + M.2 SSD | PCIe flash storage | Yes | ~$25–40 (not in BOM) |
| PostGIS server | Cloud VPS or bench PC | Yes | $0–20/mo |
| 18650 UPS | Battery + POWER_FAIL GPIO | N/A (runtime, not storage) | ~$15–40 |

**Important:** RAM buffers are **not** crash-safe. Phase 5 Anti-Death (UPS + emergency uplink) handles controlled power loss.

### 4.2 Approved approach (M7.0 — no extra hardware)

1. **Hot buffer:** Pre-allocated in-process SPSC ring (~500 slots × ~512 B ≈ 250 KB)
2. **Continuous sync:** Dedicated C++ sync thread inside `vigia_app` POSTs batches every ~5 s or N events
3. **Canonical store:** Postgres + PostGIS on server
4. **Graceful shutdown:** `systemd` stop hook drains queue before reboot
5. **Power loss (Phase 5):** UPS flush of unsynced queue in ~10–15 s window

**Do not log hazard events to the SD card.**

**M7.0 does not use a separate Python sync sidecar** — one binary, one sync thread. Defer `vigia_sync_agent.py` unless sync is split into a separate process later.

### 4.3 Edge dedup vs server dedup

| Layer | Scope | Behavior |
|-------|-------|----------|
| **Edge EventPromoter** | Same drive session, ~5 m / ~30 s | Suppress repeat pass-by observations |
| **Server `observations`** | Fleet-wide | **Append-only** — every accepted ingest is a row |
| **Server `hazard_entities`** | Fleet-wide, persistent | Merge nearby observations; update severity/count |

Reinforcement = new observation row + updated hazard entity aggregate — never UPDATE-in-place on observations.

### 4.4 When to add local SSD (optional M7.1)

Add **USB SSD** (~$20–35) only if field testing shows long offline routes or crash-safe local WAL is required before sync.

---

## 5. Edge implementation (Pi C++)

### 5.1 EventPromoter

Runs **only** after full async fusion in `midasLoop` (YOLO + MiDaS geometry + temporal + sensors):

1. **Path gate:** `geometry_confidence > 0` (proves MiDaS path ran) — blocks YOLO-only false promotions
2. **RRI gate:** `finalConfidence >= 0.75`
3. **GPS gate:** valid fix (`fix_type >= 2`, `hdop <= 2.5`) or `gps.require_valid: false` for bench
4. **Spatial-temporal dedup:** ~5 m + ~30 s — reinforce locally, don't spam queue
5. **Async handoff:** SPSC ring → sync thread (zero heap alloc in `midasLoop`)

**Map pin placement:** Use GPS lat/lon only. Bounding box is evidence metadata — do not geoproject pixel boxes without camera calibration.

### 5.2 HazardObservation struct (v1)

Hot-path safe — no `std::string`, no heap:

```cpp
struct HazardObservation {
    uint8_t  event_id[16];      // UUIDv7 raw bytes
    uint64_t device_seq;        // monotonic anti-replay counter
    uint64_t frame_index;
    uint64_t timestamp_us;      // monotonic boot time (Pi)
    char     device_id[32];     // loaded once from config at init
    uint8_t  hazard_class;      // 0 = pothole (reserve for future classes)
    float    rri, iss, yolo_conf, geometry_conf, temporal_conf;
    int32_t  bbox_x, bbox_y, bbox_w, bbox_h;
    double   lat, lon;
    float    speed_ms, hdop;
    uint8_t  gps_fix_type;
    bool     gps_valid;
};
```

JSON serialization and signing happen **only in the sync thread**.

### 5.3 New modules

| File | Role |
|------|------|
| `include/hazard_event.hpp` | Struct + JSON serializer (sync thread only) |
| `include/event_promoter.hpp` | Threshold, dedup, SPSC enqueue |
| `include/event_store.hpp` | Ring buffer + sync thread |
| `include/event_signer.hpp` | Canonical JSON hash + HMAC sign |
| `src/coordinator.cpp` | Call promoter from `midasLoop` only |
| `config/device.yaml.example` | Device ID, thresholds, sync endpoint |

---

## 6. Server implementation (PostGIS + FastAPI)

### 6.1 Database tables

**observations** — append-only ingest (idempotent on `event_id`)

```sql
observations (
  event_id UUID PRIMARY KEY,
  device_id TEXT NOT NULL,
  device_seq BIGINT NOT NULL,
  hazard_id UUID REFERENCES hazard_entities(hazard_id),  -- set after merge
  hazard_class SMALLINT NOT NULL DEFAULT 0,
  observed_at TIMESTAMPTZ NOT NULL,
  location GEOGRAPHY(POINT, 4326),
  rri, iss, yolo_conf, geometry_conf, temporal_conf REAL,
  bbox JSONB,
  speed_mps, hdop REAL,
  trust_level TEXT NOT NULL,          -- assigned by server on ingest
  raw_payload JSONB,
  ingested_at TIMESTAMPTZ DEFAULT now()
);
```

**hazard_entities** — deduplicated map features

**device_registry** — fleet identity, last `device_seq` (anti-replay)

### 6.2 API endpoints

| Endpoint | Purpose |
|----------|---------|
| `POST /v1/events` | Signed ingest (single or batch) |
| `GET /v1/hazards?bbox=...&min_severity=...` | Map viewport |
| `GET /v1/hazards/{id}` | Detail + linked observations |
| `GET /v1/hazards/heatmap?bbox=...` | Density buckets |
| `GET /v1/devices/{id}/events` | Fleet view |

### 6.3 Hazard map UI

Minimal **MapLibre GL JS** in `web/hazard-map/` — filter by `trust_level`; public view excludes unverified pins.

---

## 7. Security (build in M7 — do not defer)

### 7.1 M7 requirements (software trust)

| Control | Implementation |
|---------|----------------|
| Transport | TLS 1.3 (HTTPS only in production) |
| Device auth | mTLS client cert; dev: HMAC + device key on localhost |
| Anti-replay | Monotonic `device_seq` per device (DB transaction) |
| Integrity | HMAC/Ed25519 over canonical JSON |
| Rate limits | Per device (e.g. 60 events/min) |

**Client must not set `trust_level`.** Server assigns on ingest:

| After verify | trust_level |
|--------------|-------------|
| mTLS + signature + replay OK | `software_signed` |
| + `signed_et` ECDSA OK (M6) | `hardware_attested` |
| Multiple devices corroborate | `corroborated` |

### 7.2 Deferred to M3/M4/M6 (hardware trust)

- ATECC608A ECDSA over 96-byte `EtHashInput`
- Pico→Pi AEAD binary packets
- Full server ECDSA verify pipeline

### 7.3 Dev vs production security ramp

| Environment | Acceptable |
|-------------|------------|
| Local Docker | HMAC + device key, HTTP localhost |
| Field demo | mTLS self-signed fleet CA |
| Internet-facing | mTLS + proper PKI, no anonymous ingest |

### 7.4 Example event envelope (client payload)

```json
{
  "event_id": "0192a1b2-...",
  "device_id": "vigia-dev-001",
  "device_seq": 99102,
  "observed_at": "2026-06-17T14:32:01.042Z",
  "hazard_class": 0,
  "location": { "lat": 12.9716, "lon": 77.5946 },
  "hazard": {
    "rri": 0.82, "iss": 0.45,
    "yolo_conf": 0.91, "geometry_conf": 0.78, "temporal_conf": 0.85,
    "bbox": [120, 200, 80, 60],
    "frame_index": 104892
  },
  "motion": { "speed_mps": 8.3, "hdop": 1.2, "fix_type": 3 },
  "payload_hash": "sha256-hex",
  "signature": "base64-hmac",
  "signed_et": null
}
```

---

## 8. Configuration (proposed)

```yaml
device_id: "vigia-dev-001"
rri_threshold: 0.75
dedup:
  radius_m: 5.0
  window_s: 30.0
storage:
  mode: ram_sync
  ring_capacity: 512
sync:
  endpoint: "http://127.0.0.1:8080/v1/events"
  batch_size: 50
  interval_s: 5
  hmac_key_file: "/etc/vigia/device.key"   # gitignored; dev provisioning
gps:
  require_valid: true
  max_hdop: 2.5
```

---

## 9. Delivery phases (parallel-friendly)

| Track | Scope | Directory | Can run in parallel? |
|-------|-------|-----------|---------------------|
| **M7a** | Edge promoter + ring + sync thread | `include/`, `src/`, `config/` | Yes |
| **M7b** | PostGIS + FastAPI ingest + auth | `server/`, `docker-compose.yml` | Yes |
| **M7c** | MapLibre UI | `web/hazard-map/` | After M7b API stable |
| **M7d** | UPS flush, USB WAL, ATECC | firmware + edge | Later |

**Recommended build order:**

1. M7b server skeleton + Docker (mock ingest → map stub)
2. M7a edge wired to real fusion → POST to server
3. M7c hazard map UI
4. M7d hardening

---

## 10. Testing

| Test | Validates |
|------|-----------|
| `event_promoter_test` | Dedup, RRI gate, GPS gate, rejects YOLO-only path |
| `event_store_test` | Ring overflow, sync cursor, idempotent re-upload |
| `ingest_api_test` | Idempotency, replay/tamper rejection |
| `security_ingest_test` | HMAC/mTLS + seq window |
| `hazard_merge_test` | Two devices same coords → one entity, two observations |
| Field run | Pothole pin on map after sync |

---

## 11. Pre-build checklist

- [x] EventPromoter only on MiDaS-complete fusion path
- [x] `device_seq` + `event_id` on edge struct
- [x] Fixed-size types in hot path (no heap in promoter)
- [x] Server assigns `trust_level`
- [x] Append-only observations + aggregated hazard_entities
- [x] Single C++ sync thread (no Python sidecar for M7.0)
- [x] `observations.hazard_id` FK for map detail
- [x] Bench mode via `gps.require_valid: false`

---

## 12. Success criteria

- RRI >= 0.75 + valid GPS + full fusion → one deduplicated observation per 5 m / 30 s pass
- Synced events durable on server; hazard map shows geo pins at GPS coordinates
- Ingest rejects unauthenticated, replayed, tampered requests
- Public map respects `trust_level` policy
- Fusion hot path: zero new heap allocations in promoter enqueue path

---

## 13. Non-goals for M7

- ROS 2 migration (design for it; plain C++ first)
- NVIDIA Cosmos 3 integration
- Public anonymous reporting
- Full ATECC on every event (schema ready; hardware is M3/M4)
- Python sync sidecar (M7.0)
- Custom vector tile server

---

## 14. Research pointers for teammates

| Topic | Starting points |
|-------|-----------------|
| PostGIS spatial queries | `ST_DWithin`, geography types, bbox fetch |
| Event dedup / clustering | H3 geohash, DBSCAN on geo points |
| mTLS device provisioning | Smallstep, OpenSSL, AWS IoT cert patterns |
| Hardware attestation | ATECC608A, `EtHashInput` in `03_pico2_firmware_contracts.md` |
| Fleet telematics trust | Waze sybil problems; corroboration models |
| Edge buffer patterns | WAL, idempotent ingest, at-least-once delivery |

---

*Repository: vigia-raspi — Plan v1.1 approved for implementation.*

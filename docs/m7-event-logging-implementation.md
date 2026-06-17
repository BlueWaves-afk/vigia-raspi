# M7 Event Logging Database — Implementation Spec

**Milestone:** M7 — Event Logging, Database & Hazard Map  
**Status:** Implemented  
**Date:** 2026-06-18  
**Plan reference:** `docs/m7-event-logging-database-plan.md`

---

## 1. What M7 delivers

M7 turns fused pothole detections from the Raspberry Pi into a **durable, queryable, map-visible hazard platform**. Before M7, detections were debug stdout only — no persistence, no deduplication, no geographic record.

```
Raspberry Pi (vigia_app)
  └─ Coordinator::midasLoop (full YOLO + MiDaS + ISS fusion)
       └─ EventPromoter  (gate + dedup + enqueue)
            └─ EventStore sync thread (HMAC-signed batch upload every 5 s)
                  │
                  ▼  POST /v1/events
         Server (FastAPI + PostGIS)
           ├─ authenticate_event (HMAC-SHA256 + anti-replay)
           ├─ observations table  (append-only audit log)
           ├─ hazard_entities table  (deduplicated geo pins)
           └─ GET /v1/hazards → MapLibre map UI
```

---

## 2. Repository layout

### Edge (C++ — Raspberry Pi)

| File | Role |
|------|------|
| `include/hazard_event.hpp` | `HazardObservation` struct — fixed-size, no heap on hot path |
| `src/hazard_event.cpp` | UUID + JSON serialization |
| `include/event_promoter.hpp` | Promotion gate + SPSC ring buffer |
| `src/event_promoter.cpp` | RRI/GPS/geometry gates, haversine dedup |
| `include/event_signer.hpp` | HMAC-SHA256 signing |
| `src/event_signer.cpp` | Canonical JSON + OpenSSL HMAC |
| `include/event_store.hpp` | Ring → sync thread → HTTP upload |
| `src/event_store.cpp` | Sync loop, batch JSON, libcurl POST |
| `src/coordinator.cpp` | Wires `EventStore` into `midasLoop` only |
| `src/main.cpp` | Env vars, starts `EventStore` |
| `config/device.yaml.example` | Device ID, sync endpoint, key file |

### Server (Python + Docker)

| File | Role |
|------|------|
| `docker-compose.yml` | PostGIS 16 on port 5432 |
| `server/db/init.sql` | Schema: 3 tables + indexes |
| `server/main.py` | FastAPI — ingest, hazards, stats, map |
| `server/ingest/` | HMAC verify, anti-replay, auth |
| `scripts/provision_device.sh` | Device + HMAC key provisioning |
| `tools/m7_ingest_demo.py` | Signed demo ingest (local + `--global`) |

### Map UI

| File | Role |
|------|------|
| `web/hazard-map/index.html` | MapLibre map, OpenFreeMap tiles, GeoJSON pins |

### Tests

| File | Coverage |
|------|----------|
| `tests/event_promoter_test.cpp` | RRI/GPS/dedup gates (17 checks) |
| `tests/security_ingest_test.py` | HMAC, replay, tamper, API (15 tests) |

---

## 3. Database schema

PostgreSQL + PostGIS (`server/db/init.sql`).

### `device_registry`
Device identity and HMAC secret. `last_device_seq` enforces anti-replay.

### `observations`
One row per accepted hazard report (append-only audit log).

Key fields: `event_id`, `device_id`, `device_seq`, `location` (GEOGRAPHY POINT), `rri`, fusion scores, `bbox`, `trust_level`, `raw_payload`, `hazard_id`.

### `hazard_entities`
Deduplicated map pins. Nearby observations (within 5 m, configurable via `HAZARD_MERGE_RADIUS_M`) merge into one entity.

Key fields: `hazard_id`, `centroid`, `severity_score` (max RRI), `observation_count`, `first_seen`, `last_seen`, `status`.

---

## 4. API

Base URL: `http://<server>:8080`

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/health` | Liveness + DB check |
| `GET` | `/v1/stats` | Active hazards, observations, devices, 24 h count |
| `POST` | `/v1/events` | Signed ingest (single or `{"events":[...]}`) |
| `GET` | `/v1/hazards?bbox=&min_severity=` | GeoJSON in bounding box |
| `GET` | `/v1/hazards?all=1&min_severity=` | All active hazards (world view) |
| `GET` | `/v1/hazards/{id}?obs_limit=&obs_offset=` | Hazard detail + observations |
| `GET` | `/map` | Hazard map UI |

### Ingest security

1. **HMAC-SHA256** over canonical JSON (sorted keys, no `signature` field)
2. **Anti-replay:** `device_seq` must exceed `device_registry.last_device_seq`
3. **Idempotency:** duplicate `event_id` → HTTP 200, `status: duplicate`
4. **Trust level:** `software_signed` on accept

Errors: `401` bad signature, `403` unknown device, `409` replay.

### Hazard query notes

- Bbox filter uses numeric lat/lon bounds (not geography polygon intersection — world-spanning polygons return zero rows in PostGIS).
- At map zoom ≤ 3, the UI calls `?all=1` to fetch every active hazard.

---

## 5. Edge promotion rules

`EventPromoter::submit()` gates (all must pass):

| Gate | Default |
|------|---------|
| RRI | ≥ 0.75 |
| Geometry | `geometry_conf > 0` (MiDaS path only) |
| GPS valid | required (`VIGIA_GPS_REQUIRE_VALID=0` to disable) |
| Fix type | ≥ 2D |
| HDOP | ≤ 2.5 |
| Dedup | 5 m / 30 s window |

**Wiring rule:** promoter runs only from `Coordinator::midasLoop`, never YOLO-only `processFrame()`.

Promoted events go to an SPSC ring (512). Sync thread batches up to 50 events every 5 s, signs with HMAC, POSTs to `/v1/events`.

---

## 6. HMAC signing (edge ↔ server)

Canonical JSON must match byte-for-byte on C++ and Python sides.

- C++: alphabetical keys, default stream float formatting (no trailing zeros)
- Python: `json.dumps(..., sort_keys=True, separators=(",", ":"))`
- `observed_at` generated once and shared between canonical payload and envelope

---

## 7. Environment variables

**Edge (Pi):**

| Variable | Default | Purpose |
|----------|---------|---------|
| `VIGIA_DEVICE_ID` | `vigia-dev-001` | Must match `device_registry` |
| `VIGIA_HMAC_KEY_FILE` | — | Path to provisioned key |
| `VIGIA_SYNC_ENDPOINT` | `http://127.0.0.1:8080/v1/events` | Ingest URL |
| `VIGIA_GPS_REQUIRE_VALID` | `true` | Set `0` for bench without GPS |

**Server:**

| Variable | Default | Purpose |
|----------|---------|---------|
| `DATABASE_URL` | `postgresql://vigia:vigia_dev@127.0.0.1:5432/vigia` | Postgres |
| `HAZARD_MERGE_RADIUS_M` | `5` | Merge radius (m) |
| `DB_POOL_MIN` / `DB_POOL_MAX` | `2` / `10` | Connection pool |

---

## 8. Local dev runbook

```bash
docker compose up -d
# provision device (save HMAC key)
cd server && python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 8080

# demo ingest — local cluster
python3 tools/m7_ingest_demo.py --hmac-key "$HMAC_KEY" --from-seq 0

# demo ingest — 10 cities worldwide
python3 tools/m7_ingest_demo.py --global --from-seq N --hmac-key "$HMAC_KEY"

open http://127.0.0.1:8080/map
```

Tests:

```bash
pytest tests/security_ingest_test.py -v
g++ -std=c++17 -I include tests/event_promoter_test.cpp src/event_promoter.cpp -o /tmp/promoter_test && /tmp/promoter_test
```

---

## 9. Raspberry Pi wiring

```bash
export VIGIA_DEVICE_ID=vigia-pi-001
export VIGIA_HMAC_KEY_FILE=/etc/vigia/device.key
export VIGIA_SYNC_ENDPOINT=http://<server-ip>:8080/v1/events
export VIGIA_GPS_REQUIRE_VALID=0   # bench only

./build/vigia_app models/yolo26/yolo26_model.xml \
  models/midasv21/openvino_midas_v21_small_256.xml 30 0
```

Requires build with libcurl + OpenSSL. Server must be reachable from the Pi (not `127.0.0.1` on the Pi unless the server runs there).

**Video file:** `vigia_app` uses camera index only. `system_visual_test --video` does not wire `EventStore` — demo video does not log to the DB without further integration.

---

## 10. Known gaps (post-M7)

| Gap | Notes |
|-----|-------|
| ROS2 → PostGIS | `FusionNode` publishes `HazardEvent`; no node POSTs to `/v1/events` yet |
| New YOLO classes | `hazard_class` enum needs alignment with retrained model |
| HTTPS / mTLS | Dev uses plain HTTP |
| Video → DB | `system_visual_test` has no `EventStore` |
| Hazard resolution | No API to mark hazards resolved |
| Field E2E | Pi camera + GPS → server → map not yet validated in field |

---

## 11. Design decisions

**Observation vs hazard entity:** Raw reports go to `observations`; map shows deduplicated `hazard_entities` within 5 m.

**HMAC vs Ed25519:** M7 uses symmetric HMAC (server-provisioned keys). ROS2 AntiDeath AWS path uses Ed25519 — separate until unified.

**PostGIS on server:** Pi buffers in RAM; server DB is source of truth (no SD-card event WAL in M7).

**MiDaS-only promotion:** Prevents YOLO-only ghost events from entering the database.

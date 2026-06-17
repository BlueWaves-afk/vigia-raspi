# VIGIA M7b Server

PostGIS-backed ingest API and hazard map endpoints for the VIGIA edge node.

## Prerequisites

- Docker and Docker Compose
- Python 3.11+

## Quick start

### 1. Start PostgreSQL + PostGIS

From the repository root:

```bash
docker compose up -d
```

Wait for the database to become healthy:

```bash
docker compose ps
```

The schema in `server/db/init.sql` is applied automatically on first boot.

### 2. Install Python dependencies

```bash
cd server
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 3. Provision a device

From the repository root:

```bash
./scripts/provision_device.sh vigia-dev-001
```

This prints the generated HMAC key. Store it on the edge device (e.g. `/etc/vigia/device.key`) and reference it from `config/device.yaml`.

### 4. Run the API

```bash
cd server
source .venv/bin/activate
uvicorn main:app --host 0.0.0.0 --port 8080 --reload
```

Health check:

```bash
curl http://127.0.0.1:8080/health
```

## Environment

| Variable | Default | Purpose |
|----------|---------|---------|
| `DATABASE_URL` | `postgresql://vigia:vigia_dev@127.0.0.1:5432/vigia` | Postgres connection |
| `HAZARD_MERGE_RADIUS_M` | `5` | Hazard entity merge radius (meters) |

## API

| Method | Path | Description |
|--------|------|-------------|
| `POST` | `/v1/events` | Signed event ingest (single object or `{"events": [...]}`) |
| `GET` | `/v1/hazards?bbox=minLon,minLat,maxLon,maxLat` | GeoJSON FeatureCollection |
| `GET` | `/v1/hazards/{id}` | Hazard detail with observations |
| `GET` | `/health` | Liveness + DB connectivity |

### Ingest security

- **HMAC-SHA256** over canonical JSON (sorted keys, compact, `signature` field excluded)
- Signature via `X-Vigia-Signature` header or body `signature` field
- **Anti-replay:** `device_seq` must be strictly greater than `device_registry.last_device_seq`
- **Idempotency:** duplicate `event_id` returns HTTP 200 with `status: duplicate`
- Server assigns `trust_level=software_signed` on successful verification

### Example signed ingest

```python
import base64, hashlib, hmac, json, requests

key = "your-provisioned-key"
event = {
    "event_id": "0192a1b2-c3d4-7890-abcd-ef1234567890",
    "device_id": "vigia-dev-001",
    "device_seq": 1,
    "observed_at": "2026-06-17T14:32:01.042Z",
    "hazard_class": 0,
    "location": {"lat": 12.9716, "lon": 77.5946},
    "hazard": {
        "rri": 0.82, "iss": 0.45,
        "yolo_conf": 0.91, "geometry_conf": 0.78, "temporal_conf": 0.85,
        "bbox": [120, 200, 80, 60],
        "frame_index": 104892,
    },
    "motion": {"speed_mps": 8.3, "hdop": 1.2, "fix_type": 3},
}
canonical = json.dumps(event, sort_keys=True, separators=(",", ":"))
sig = base64.b64encode(hmac.new(key.encode(), canonical.encode(), hashlib.sha256).digest()).decode()
event["signature"] = sig
requests.post("http://127.0.0.1:8080/v1/events", json=event)
```

## Tests

From the repository root (install `pytest` and `httpx` if needed):

```bash
pip install pytest httpx
pytest tests/security_ingest_test.py -v
```

Tests mock the database layer and validate replay rejection, tamper detection, and successful ingest flow.

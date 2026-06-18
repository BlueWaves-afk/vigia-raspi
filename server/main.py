"""VIGIA M7b ingest and hazard map API."""

from __future__ import annotations

import json
import os
import uuid
from contextlib import asynccontextmanager, contextmanager
from datetime import datetime
from typing import Any, Generator

import logging
import threading

import psycopg2
import psycopg2.extras
from psycopg2.pool import ThreadedConnectionPool
from fastapi import FastAPI, Header, HTTPException, Query, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from ingest.attestation import AttestationError, process_attestation_event
from ingest.cosmos3_stub import get_cosmos3_client
from ingest.db_adapter import VigiaDb
from ingest.auth import (
    AuthError,
    InvalidSignatureError,
    MissingDeviceIdError,
    UnknownDeviceError,
    authenticate_event,
    lookup_device,
)
from ingest.replay import ReplayError, advance_device_seq

DATABASE_URL = os.environ.get(
    "DATABASE_URL",
    "postgresql://vigia:vigia_dev@127.0.0.1:5432/vigia",
)
MERGE_RADIUS_M = float(os.environ.get("HAZARD_MERGE_RADIUS_M", "5"))
MQTT_BROKER_HOST = os.environ.get("MQTT_BROKER_HOST", "")
MQTT_BROKER_PORT = int(os.environ.get("MQTT_BROKER_PORT", "1883"))
MQTT_ATTEST_TOPIC = "vigia/attest/+/hazard"

log = logging.getLogger(__name__)
POOL_MIN = int(os.environ.get("DB_POOL_MIN", "2"))
POOL_MAX = int(os.environ.get("DB_POOL_MAX", "10"))
TRUST_LEVEL_SIGNED = "software_signed"
TRUST_LEVEL_HARDWARE = "hardware_attested"


def _resolve_trust_level(event: dict[str, Any]) -> str:
    signed_et = event.get("signed_et")
    if isinstance(signed_et, dict) and signed_et.get("valid") is True:
        return TRUST_LEVEL_HARDWARE
    if event.get("signed_et_valid") is True:
        return TRUST_LEVEL_HARDWARE
    return TRUST_LEVEL_SIGNED

# ---------------------------------------------------------------------------
# Connection pool — created once at startup, shared across all requests.
# ---------------------------------------------------------------------------
_pool: ThreadedConnectionPool | None = None


_cosmos3_client = None  # initialised in lifespan after pool is ready


def _mqtt_on_message(client, userdata, msg) -> None:
    """Handle incoming MQTT attestation event from an edge node."""
    try:
        with get_conn() as conn:
            db = VigiaDb(conn)
            verified = process_attestation_event(msg.payload, db, _cosmos3_client)
            log.info("Attestation verified: device=%s seq=%s",
                     verified.get("device_id"), verified.get("sequence"))
    except AttestationError as exc:
        log.warning("Attestation failed: %s", exc)
    except Exception as exc:  # noqa: BLE001
        log.error("MQTT handler error: %s", exc)


def _start_mqtt_subscriber() -> None:
    """Launch MQTT subscriber in a daemon thread (no-op if broker not configured)."""
    if not MQTT_BROKER_HOST:
        log.info("MQTT_BROKER_HOST not set — attestation MQTT subscriber disabled")
        return
    try:
        import paho.mqtt.client as mqtt  # optional dep — only needed when broker is up
    except ImportError:
        log.warning("paho-mqtt not installed — MQTT subscriber disabled")
        return

    client = mqtt.Client(client_id="vigia-server-attest")
    client.on_message = _mqtt_on_message
    client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, keepalive=60)
    client.subscribe(MQTT_ATTEST_TOPIC)
    log.info("MQTT subscriber connected to %s:%d topic=%s",
             MQTT_BROKER_HOST, MQTT_BROKER_PORT, MQTT_ATTEST_TOPIC)

    t = threading.Thread(target=client.loop_forever, daemon=True, name="mqtt-attest")
    t.start()


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _pool, _cosmos3_client
    _pool = ThreadedConnectionPool(POOL_MIN, POOL_MAX, DATABASE_URL)
    _cosmos3_client = get_cosmos3_client()
    _start_mqtt_subscriber()
    yield
    if _pool:
        _pool.closeall()


app = FastAPI(title="VIGIA Hazard Server", version="1.0.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

WEB_DIR = os.path.join(os.path.dirname(__file__), "..", "web", "hazard-map")
if os.path.isdir(WEB_DIR):
    app.mount("/map-static", StaticFiles(directory=WEB_DIR), name="map-static")


class IngestResult(BaseModel):
    event_id: str
    hazard_id: str | None = None
    status: str = "accepted"


class BatchIngestResponse(BaseModel):
    results: list[IngestResult]


@contextmanager
def get_conn() -> Generator[Any, None, None]:
    """Borrow a connection from the pool; roll back and return on any error."""
    if _pool is None:
        raise RuntimeError("Connection pool not initialised")
    conn = _pool.getconn()
    try:
        yield conn
    except Exception:
        conn.rollback()
        raise
    finally:
        _pool.putconn(conn)


def _parse_events(body: Any) -> list[dict[str, Any]]:
    if isinstance(body, dict) and "events" in body:
        events = body["events"]
        if not isinstance(events, list) or not events:
            raise HTTPException(status_code=400, detail="events must be a non-empty list")
        return events
    if isinstance(body, dict):
        return [body]
    raise HTTPException(status_code=400, detail="Expected event object or {events: [...]}")


def _parse_observed_at(value: str) -> datetime:
    normalized = value.replace("Z", "+00:00")
    return datetime.fromisoformat(normalized)


def _location_wkt(lat: float, lon: float) -> str:
    return f"SRID=4326;POINT({lon} {lat})"


def _existing_event(cur, event_id: str) -> dict[str, Any] | None:
    cur.execute(
        "SELECT event_id::text, hazard_id::text FROM observations WHERE event_id = %s",
        (event_id,),
    )
    row = cur.fetchone()
    if row is None:
        return None
    return {"event_id": row[0], "hazard_id": row[1], "status": "duplicate"}


def _merge_hazard(cur, lat: float, lon: float, observed_at: datetime, rri: float) -> str:
    wkt = _location_wkt(lat, lon)
    cur.execute(
        """
        SELECT hazard_id::text
        FROM hazard_entities
        WHERE status = 'active'
          AND ST_DWithin(centroid, ST_GeogFromText(%s), %s)
        ORDER BY ST_Distance(centroid, ST_GeogFromText(%s))
        LIMIT 1
        """,
        (wkt, MERGE_RADIUS_M, wkt),
    )
    row = cur.fetchone()
    if row:
        hazard_id = row[0]
        cur.execute(
            """
            UPDATE hazard_entities
            SET observation_count = observation_count + 1,
                last_seen = GREATEST(last_seen, %s),
                severity_score = GREATEST(severity_score, %s)
            WHERE hazard_id = %s
            RETURNING hazard_id::text
            """,
            (observed_at, rri, hazard_id),
        )
        return cur.fetchone()[0]

    hazard_id = str(uuid.uuid4())
    cur.execute(
        """
        INSERT INTO hazard_entities (
            hazard_id, centroid, severity_score, observation_count,
            first_seen, last_seen, status
        ) VALUES (
            %s, ST_GeogFromText(%s), %s, 1, %s, %s, 'active'
        )
        RETURNING hazard_id::text
        """,
        (hazard_id, wkt, rri, observed_at, observed_at),
    )
    return cur.fetchone()[0]


def _insert_observation(cur, event: dict[str, Any], hazard_id: str) -> str:
    location = event.get("location") or {}
    hazard = event.get("hazard") or {}
    motion = event.get("motion") or {}

    lat = location.get("lat")
    lon = location.get("lon")
    wkt = _location_wkt(lat, lon) if lat is not None and lon is not None else None

    event_id = event["event_id"]
    observed_at = _parse_observed_at(event["observed_at"])
    bbox = hazard.get("bbox")

    cur.execute(
        """
        INSERT INTO observations (
            event_id, device_id, device_seq, hazard_id, hazard_class,
            observed_at, location, rri, iss, yolo_conf, geometry_conf,
            temporal_conf, bbox, speed_mps, hdop, trust_level, raw_payload
        ) VALUES (
            %s, %s, %s, %s, %s,
            %s,
            CASE WHEN %s IS NULL THEN NULL ELSE ST_GeogFromText(%s) END,
            %s, %s, %s, %s, %s,
            %s, %s, %s, %s, %s
        )
        RETURNING event_id::text
        """,
        (
            event_id,
            event["device_id"],
            event["device_seq"],
            hazard_id,
            event.get("hazard_class", 0),
            observed_at,
            wkt,
            wkt,
            hazard.get("rri"),
            hazard.get("iss"),
            hazard.get("yolo_conf"),
            hazard.get("geometry_conf"),
            hazard.get("temporal_conf"),
            json.dumps(bbox) if bbox is not None else None,
            motion.get("speed_mps"),
            motion.get("hdop"),
            _resolve_trust_level(event),
            json.dumps(event),
        ),
    )
    return cur.fetchone()[0]


def _validate_event_shape(event: dict[str, Any]) -> None:
    required = ("event_id", "device_id", "device_seq", "observed_at")
    missing = [field for field in required if field not in event]
    if missing:
        raise HTTPException(status_code=400, detail=f"Missing fields: {', '.join(missing)}")


def ingest_single_event(
    conn,
    event: dict[str, Any],
    header_signature: str | None,
) -> IngestResult:
    _validate_event_shape(event)

    event_id = event["event_id"]
    device_id = event["device_id"]

    with conn.cursor() as cur:
        existing = _existing_event(cur, event_id)
        if existing:
            conn.commit()
            return IngestResult(
                event_id=existing["event_id"],
                hazard_id=existing["hazard_id"],
                status="duplicate",
            )

        hmac_key, last_seq, cert_pem = lookup_device(cur, device_id)
        device_seq = authenticate_event(
            event,
            header_signature,
            hmac_key=hmac_key,
            last_device_seq=last_seq,
            cert_pem=cert_pem,
        )

        location = event.get("location") or {}
        lat = location.get("lat")
        lon = location.get("lon")
        if lat is None or lon is None:
            raise HTTPException(status_code=400, detail="location.lat and location.lon required")

        hazard = event.get("hazard") or {}
        rri = float(hazard.get("rri") or 0.0)
        observed_at = _parse_observed_at(event["observed_at"])

        hazard_id = _merge_hazard(cur, float(lat), float(lon), observed_at, rri)
        inserted_id = _insert_observation(cur, event, hazard_id)
        advance_device_seq(cur, device_id, device_seq)
        conn.commit()

        return IngestResult(event_id=inserted_id, hazard_id=hazard_id, status="accepted")


@app.get("/health")
def health():
    try:
        with get_conn() as conn, conn.cursor() as cur:
            cur.execute("SELECT 1")
        return {"status": "ok"}
    except Exception as exc:  # noqa: BLE001 — surface DB connectivity in health
        return JSONResponse(status_code=503, content={"status": "error", "detail": str(exc)})


@app.get("/v1/stats")
def stats() -> dict[str, Any]:
    """Aggregate stats for the map sidebar."""
    with get_conn() as conn, conn.cursor() as cur:
        cur.execute(
            """
            SELECT
                (SELECT COUNT(*)
                    FROM hazard_entities WHERE status = 'active') AS active_hazards,
                (SELECT COUNT(*)
                    FROM observations)                             AS total_observations,
                (SELECT COUNT(DISTINCT device_id)
                    FROM observations)                            AS total_devices,
                (SELECT COUNT(*)
                    FROM observations
                    WHERE ingested_at > now() - INTERVAL '24 hours') AS obs_last_24h
            """
        )
        row = cur.fetchone()
    return {
        "active_hazards":    int(row[0]),
        "total_observations": int(row[1]),
        "total_devices":     int(row[2]),
        "obs_last_24h":      int(row[3]),
    }


@app.post("/v1/events", response_model=IngestResult | BatchIngestResponse)
async def ingest_events(
    request: Request,
    x_vigia_signature: str | None = Header(default=None, alias="X-Vigia-Signature"),
) -> IngestResult | BatchIngestResponse:
    try:
        body = await request.json()
    except json.JSONDecodeError as exc:
        raise HTTPException(status_code=400, detail="Invalid JSON") from exc

    events = _parse_events(body)
    is_batch = isinstance(body, dict) and "events" in body

    try:
        with get_conn() as conn:
            results: list[IngestResult] = []
            for event in events:
                if not isinstance(event, dict):
                    raise HTTPException(status_code=400, detail="Each event must be an object")
                results.append(ingest_single_event(conn, event, x_vigia_signature))
    except UnknownDeviceError as exc:
        raise HTTPException(status_code=403, detail=str(exc)) from exc
    except InvalidSignatureError as exc:
        raise HTTPException(status_code=401, detail=str(exc)) from exc
    except ReplayError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except AuthError as exc:
        raise HTTPException(status_code=401, detail=str(exc)) from exc
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc

    if is_batch:
        return BatchIngestResponse(results=results)
    return results[0]


@app.get("/v1/hazards")
def list_hazards(
    bbox: str | None = Query(default=None, description="minLon,minLat,maxLon,maxLat"),
    min_severity: float = Query(default=0.0, ge=0.0, le=1.0),
    all_hazards: bool = Query(default=False, alias="all"),
    limit: int = Query(default=500, ge=1, le=2000),
) -> dict[str, Any]:
    """Return active hazard entities as GeoJSON.

    Use ``?all=1`` (or omit bbox on low-zoom map) to fetch every active hazard.
    Bbox filtering uses numeric lat/lon bounds — PostGIS ST_Intersects on a
    geography polygon fails for world-spanning envelopes.
    """
    geo_clause = ""
    params: list[Any] = [min_severity]

    if not all_hazards and bbox:
        try:
            min_lon, min_lat, max_lon, max_lat = (float(v) for v in bbox.split(","))
        except ValueError as exc:
            raise HTTPException(status_code=400, detail="bbox must be minLon,minLat,maxLon,maxLat") from exc

        if min_lon >= max_lon or min_lat >= max_lat:
            raise HTTPException(status_code=400, detail="Invalid bbox bounds")

        lon_span = max_lon - min_lon
        lat_span = max_lat - min_lat
        # Near-global viewport — skip geo filter (polygon intersection is unreliable).
        if lon_span < 350 and lat_span < 170:
            geo_clause = """
              AND ST_X(centroid::geometry) >= %s
              AND ST_X(centroid::geometry) <= %s
              AND ST_Y(centroid::geometry) >= %s
              AND ST_Y(centroid::geometry) <= %s
            """
            params.extend([min_lon, max_lon, min_lat, max_lat])

    params.append(limit)

    with get_conn() as conn, conn.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cur:
        cur.execute(
            f"""
            SELECT
                hazard_id::text AS id,
                ST_Y(centroid::geometry) AS lat,
                ST_X(centroid::geometry) AS lon,
                severity_score,
                observation_count,
                first_seen,
                last_seen,
                status,
                h3_index
            FROM hazard_entities
            WHERE status = 'active'
              AND severity_score >= %s
              {geo_clause}
            ORDER BY last_seen DESC
            LIMIT %s
            """,
            params,
        )
        rows = cur.fetchall()

    features = [
        {
            "type": "Feature",
            "id": row["id"],
            "geometry": {"type": "Point", "coordinates": [row["lon"], row["lat"]]},
            "properties": {
                "hazard_id": row["id"],
                "severity_score": row["severity_score"],
                "observation_count": row["observation_count"],
                "first_seen": row["first_seen"].isoformat(),
                "last_seen": row["last_seen"].isoformat(),
                "status": row["status"],
                "h3_index": row["h3_index"],
            },
        }
        for row in rows
    ]
    return {"type": "FeatureCollection", "features": features}


@app.get("/v1/hazards/{hazard_id}")
def get_hazard(
    hazard_id: str,
    obs_limit: int = Query(default=50, ge=1, le=500),
    obs_offset: int = Query(default=0, ge=0),
) -> dict[str, Any]:
    with get_conn() as conn, conn.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cur:
        cur.execute(
            """
            SELECT
                hazard_id::text AS id,
                ST_Y(centroid::geometry) AS lat,
                ST_X(centroid::geometry) AS lon,
                severity_score,
                observation_count,
                first_seen,
                last_seen,
                status,
                h3_index
            FROM hazard_entities
            WHERE hazard_id = %s
            """,
            (hazard_id,),
        )
        hazard = cur.fetchone()
        if hazard is None:
            raise HTTPException(status_code=404, detail="Hazard not found")

        cur.execute(
            """
            SELECT
                event_id::text,
                device_id,
                device_seq,
                hazard_class,
                observed_at,
                ST_Y(location::geometry) AS lat,
                ST_X(location::geometry) AS lon,
                rri, iss, yolo_conf, geometry_conf, temporal_conf,
                bbox, speed_mps, hdop, trust_level, ingested_at
            FROM observations
            WHERE hazard_id = %s
            ORDER BY observed_at DESC
            LIMIT %s OFFSET %s
            """,
            (hazard_id, obs_limit, obs_offset),
        )
        observations = cur.fetchall()

    obs_list = []
    for row in observations:
        obs_list.append(
            {
                "event_id": row["event_id"],
                "device_id": row["device_id"],
                "device_seq": row["device_seq"],
                "hazard_class": row["hazard_class"],
                "observed_at": row["observed_at"].isoformat(),
                "location": (
                    {"lat": row["lat"], "lon": row["lon"]}
                    if row["lat"] is not None and row["lon"] is not None
                    else None
                ),
                "rri": row["rri"],
                "iss": row["iss"],
                "yolo_conf": row["yolo_conf"],
                "geometry_conf": row["geometry_conf"],
                "temporal_conf": row["temporal_conf"],
                "bbox": row["bbox"],
                "speed_mps": row["speed_mps"],
                "hdop": row["hdop"],
                "trust_level": row["trust_level"],
                "ingested_at": row["ingested_at"].isoformat(),
            }
        )

    return {
        "hazard_id": hazard["id"],
        "centroid": {"lat": hazard["lat"], "lon": hazard["lon"]},
        "severity_score": hazard["severity_score"],
        "observation_count": hazard["observation_count"],
        "first_seen": hazard["first_seen"].isoformat(),
        "last_seen": hazard["last_seen"].isoformat(),
        "status": hazard["status"],
        "h3_index": hazard["h3_index"],
        "observations": obs_list,
        "obs_limit": obs_limit,
        "obs_offset": obs_offset,
    }


@app.get("/map")
def hazard_map() -> FileResponse:
    index = os.path.join(WEB_DIR, "index.html")
    if not os.path.isfile(index):
        raise HTTPException(status_code=404, detail="Hazard map UI not found")
    return FileResponse(index)

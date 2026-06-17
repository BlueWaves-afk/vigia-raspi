#!/usr/bin/env python3
"""Post sample signed hazard events to the M7b ingest API for map testing."""

from __future__ import annotations

import argparse
import json
import sys
import uuid
from datetime import datetime, timezone

import httpx

sys.path.insert(0, "server")
from ingest.signature import compute_hmac  # noqa: E402

# Named demo sites — spaced >5 m apart so each becomes its own map pin.
GLOBAL_DEMO_SITES = [
    ("Bangalore, India",       12.9716,  77.5946, 0.88),
    ("London, UK",             51.5074,  -0.1278, 0.91),
    ("New York, USA",          40.7128, -74.0060, 0.79),
    ("Tokyo, Japan",           35.6762, 139.6503, 0.85),
    ("Sydney, Australia",     -33.8688, 151.2093, 0.77),
    ("São Paulo, Brazil",     -23.5505, -46.6333, 0.82),
    ("Cape Town, South Africa",-33.9249,  18.4241, 0.74),
    ("Dubai, UAE",             25.2048,  55.2708, 0.86),
    ("Paris, France",          48.8566,   2.3522, 0.83),
    ("Mexico City, Mexico",    19.4326, -99.1332, 0.80),
]


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def make_event(
    device_id: str,
    device_seq: int,
    lat: float,
    lon: float,
    rri: float,
    hmac_key: str,
    *,
    hazard_class: int = 0,
) -> dict:
    event = {
        "event_id": str(uuid.uuid4()),
        "device_id": device_id,
        "device_seq": device_seq,
        "observed_at": utc_now_iso(),
        "hazard_class": hazard_class,
        "location": {"lat": lat, "lon": lon},
        "hazard": {
            "rri": rri,
            "iss": 0.45,
            "yolo_conf": 0.91,
            "geometry_conf": 0.78,
            "temporal_conf": 0.85,
            "bbox": [120, 200, 80, 60],
            "frame_index": 1000 + device_seq,
        },
        "motion": {"speed_mps": 8.3, "hdop": 1.2, "fix_type": 3},
    }
    event["signature"] = compute_hmac(event, hmac_key)
    return event


def build_events(args: argparse.Namespace) -> list[dict]:
    events: list[dict] = []
    seq = args.from_seq

    if args.global_demo:
        sites = GLOBAL_DEMO_SITES[: args.count] if args.count else GLOBAL_DEMO_SITES
        for name, lat, lon, rri in sites:
            seq += 1
            events.append(make_event(args.device_id, seq, lat, lon, rri, args.hmac_key))
            print(f"  + {name} ({lat:.4f}, {lon:.4f}) RRI={rri:.2f} seq={seq}")
        return events

    base_lat, base_lon = 12.9716, 77.5946
    for i in range(args.count):
        seq += 1
        events.append(
            make_event(
                args.device_id,
                seq,
                base_lat + i * 0.0001,
                base_lon + i * 0.0001,
                0.78 + i * 0.02,
                args.hmac_key,
            )
        )
    return events


def main() -> int:
    parser = argparse.ArgumentParser(description="Ingest demo hazard events")
    parser.add_argument("--api", default="http://127.0.0.1:8080")
    parser.add_argument("--device-id", default="vigia-dev-001")
    parser.add_argument("--hmac-key", required=True)
    parser.add_argument("--count", type=int, default=3,
                        help="Number of events (local cluster) or cities (with --global)")
    parser.add_argument("--from-seq", type=int, default=0,
                        help="Start device_seq after this value (check device_registry.last_device_seq)")
    parser.add_argument("--global", dest="global_demo", action="store_true",
                        help="Post hazards at cities around the world")
    args = parser.parse_args()

    if args.global_demo and args.count == 3:
        args.count = len(GLOBAL_DEMO_SITES)

    print(f"Ingesting {args.count if not args.global_demo else min(args.count, len(GLOBAL_DEMO_SITES))} demo hazard(s)…")
    events = build_events(args)
    if not events:
        print("No events to send.", file=sys.stderr)
        return 1

    url = f"{args.api.rstrip('/')}/v1/events"
    resp = httpx.post(url, json={"events": events}, timeout=30.0)
    print(f"\nPOST {url} -> {resp.status_code}")
    print(json.dumps(resp.json(), indent=2))

    if resp.is_success:
        print(f"\nMap: {args.api.rstrip('/')}/map")
        print("Pan/zoom the map — pins appear worldwide. Lower min severity slider if needed.")

    return 0 if resp.is_success else 1


if __name__ == "__main__":
    raise SystemExit(main())

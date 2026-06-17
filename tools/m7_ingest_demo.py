#!/usr/bin/env python3
"""Post sample signed hazard events to the M7b ingest API for map testing."""

from __future__ import annotations

import argparse
import json
import sys
import uuid

import httpx

sys.path.insert(0, "server")
from ingest.signature import compute_hmac  # noqa: E402


def make_event(device_id: str, device_seq: int, lat: float, lon: float, rri: float, hmac_key: str) -> dict:
    event = {
        "event_id": str(uuid.uuid4()),
        "device_id": device_id,
        "device_seq": device_seq,
        "observed_at": "2026-06-18T12:00:00.000Z",
        "hazard_class": 0,
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


def main() -> int:
    parser = argparse.ArgumentParser(description="Ingest demo hazard events")
    parser.add_argument("--api", default="http://127.0.0.1:8080")
    parser.add_argument("--device-id", default="vigia-dev-001")
    parser.add_argument("--hmac-key", required=True)
    parser.add_argument("--count", type=int, default=3)
    args = parser.parse_args()

    events = []
    base_lat, base_lon = 12.9716, 77.5946
    for i in range(args.count):
        events.append(
            make_event(
                args.device_id,
                i + 1,
                base_lat + i * 0.0001,
                base_lon + i * 0.0001,
                0.78 + i * 0.02,
                args.hmac_key,
            )
        )

    url = f"{args.api.rstrip('/')}/v1/events"
    resp = httpx.post(url, json={"events": events}, timeout=10.0)
    print(f"POST {url} -> {resp.status_code}")
    print(json.dumps(resp.json(), indent=2))
    return 0 if resp.is_success else 1


if __name__ == "__main__":
    raise SystemExit(main())

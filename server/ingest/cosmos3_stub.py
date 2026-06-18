"""
cosmos3_stub.py — Strategic stub for NVIDIA Cosmos 3 world model integration.

Architecture position:
  VIGIA edge node → MQTT attestation pipeline → [here] → Cosmos 3

The VIGIA pipeline already produces the exact inputs Cosmos 3 requires:
  - spatial_latent: 256-D embedding from the YOLO C2PSA neck (S_t)
  - gps_location:   georeferenced hazard position
  - et_hash:        hardware-attested integrity proof of the sensor reading

This stub logs the would-be submission and is a drop-in replacement for
a real Cosmos 3 API client. When NVIDIA grants API partnership access,
replace this module with the live client — no other code changes needed.

Activation: set COSMOS3_API_KEY in environment to switch to live mode.
"""

from __future__ import annotations

import json
import logging
import os
from typing import Any

log = logging.getLogger(__name__)

_LIVE_MODE = bool(os.environ.get("COSMOS3_API_KEY"))


class Cosmos3Client:
    """
    Client for NVIDIA Cosmos 3 world model update API.

    In stub mode (no COSMOS3_API_KEY): logs the payload at INFO level and returns
    a simulated response. In live mode: submits to the Cosmos 3 API endpoint.

    The spatial_latent field carries the 256-D scene embedding from the YOLO
    penultimate feature map — this is the compressed perceptual representation
    that Cosmos 3 uses to update its physical world state.
    """

    def __init__(self, api_key: str | None = None, endpoint: str | None = None) -> None:
        self._api_key = api_key or os.environ.get("COSMOS3_API_KEY", "")
        self._endpoint = endpoint or os.environ.get(
            "COSMOS3_ENDPOINT",
            "https://api.cosmos.nvidia.com/v1/world-model/update",
        )
        self._live = bool(self._api_key)

        if self._live:
            log.info("Cosmos3Client: LIVE MODE — submitting to %s", self._endpoint)
        else:
            log.info(
                "Cosmos3Client: STUB MODE — set COSMOS3_API_KEY to enable live submission. "
                "Pipeline is architecturally ready for Phase 6 foundation model integration."
            )

    def submit_world_model_update(self, event: dict[str, Any]) -> dict[str, Any]:
        """
        Submit a verified VIGIA hazard event as a Cosmos 3 world model update.

        event fields consumed:
          device_id      — which sensor reported
          gps_location   — {lat, lon} for spatial anchoring
          spatial_latent — 256-D float32 scene embedding (S_t)
          et_hash        — hardware attestation proof (hex string)
          rri_score      — road roughness index
          iss_score      — impact severity score
          timestamp_us   — microsecond epoch

        Returns the API response dict (or simulated response in stub mode).
        """
        if not self._live:
            return self._stub_submit(event)
        return self._live_submit(event)

    def _stub_submit(self, event: dict[str, Any]) -> dict[str, Any]:
        latent = event.get("spatial_latent")
        latent_summary = (
            f"{len(latent)}-D embedding, norm={sum(x**2 for x in latent)**0.5:.3f}"
            if latent else "none"
        )
        log.info(
            "COSMOS3 [STUB] Would submit world model update: "
            "device=%s gps=(%s, %s) rri=%.3f iss=%.3f latent=%s et_hash=%s...",
            event.get("device_id"),
            event.get("gps_location", {}).get("lat"),
            event.get("gps_location", {}).get("lon"),
            event.get("rri_score") or 0.0,
            event.get("iss_score") or 0.0,
            latent_summary,
            (event.get("et_hash") or "")[:16],
        )
        return {
            "status": "stub",
            "message": "Cosmos 3 integration pending API key — VIGIA pipeline ready",
            "would_submit": {
                "device_id": event.get("device_id"),
                "gps_location": event.get("gps_location"),
                "rri_score": event.get("rri_score"),
                "latent_dims": len(latent) if latent else 0,
                "attested": bool(event.get("et_hash")),
            },
        }

    def _live_submit(self, event: dict[str, Any]) -> dict[str, Any]:
        import urllib.request  # stdlib only — avoids adding httpx as hard dep here
        body = json.dumps({
            "source": "vigia_edge",
            "event_type": event.get("event_type", "road_hazard_attested"),
            "timestamp_us": event.get("timestamp_us"),
            "device_id": event.get("device_id"),
            "attestation": {
                "et_hash": event.get("et_hash"),
                "status": event.get("attestation_status"),
            },
            "location": event.get("gps_location"),
            "scores": {
                "rri": event.get("rri_score"),
                "iss": event.get("iss_score"),
                "yolo_confidence": event.get("yolo_confidence"),
            },
            "spatial_embedding": event.get("spatial_latent"),
        }).encode()

        req = urllib.request.Request(
            self._endpoint,
            data=body,
            headers={
                "Authorization": f"Bearer {self._api_key}",
                "Content-Type": "application/json",
                "X-Vigia-Version": "1.0",
            },
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=10) as resp:
            return json.loads(resp.read())


def get_cosmos3_client() -> Cosmos3Client | None:
    """Return a Cosmos3Client instance, or None if explicitly disabled."""
    if os.environ.get("COSMOS3_DISABLED", "").lower() in ("1", "true", "yes"):
        return None
    return Cosmos3Client()

"""Security tests for M7b ingest: HMAC, replay, tamper rejection."""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from ingest.auth import InvalidSignatureError, authenticate_event
from ingest.replay import ReplayError, check_device_seq
from ingest.signature import canonical_json, compute_hmac, verify_hmac


DEVICE_ID = "vigia-test-001"
HMAC_KEY = "test-secret-key-for-hmac-signing"


def _sample_event(device_seq: int = 1, **overrides: Any) -> dict[str, Any]:
    event = {
        "event_id": "0192a1b2-c3d4-7890-abcd-ef1234567890",
        "device_id": DEVICE_ID,
        "device_seq": device_seq,
        "observed_at": "2026-06-17T14:32:01.042Z",
        "hazard_class": 0,
        "location": {"lat": 12.9716, "lon": 77.5946},
        "hazard": {
            "rri": 0.82,
            "iss": 0.45,
            "yolo_conf": 0.91,
            "geometry_conf": 0.78,
            "temporal_conf": 0.85,
            "bbox": [120, 200, 80, 60],
            "frame_index": 104892,
        },
        "motion": {"speed_mps": 8.3, "hdop": 1.2, "fix_type": 3},
    }
    event.update(overrides)
    event["signature"] = compute_hmac(event, HMAC_KEY)
    return event


class TestSignature:
    def test_canonical_json_excludes_signature(self) -> None:
        payload = {"b": 2, "a": 1, "signature": "ignored"}
        assert canonical_json(payload) == b'{"a":1,"b":2}'

    def test_verify_valid_hmac(self) -> None:
        event = _sample_event()
        assert verify_hmac(event, event["signature"], HMAC_KEY)

    def test_verify_rejects_tampered_body(self) -> None:
        event = _sample_event()
        tampered = dict(event)
        tampered["location"] = {"lat": 99.0, "lon": 77.5946}
        assert not verify_hmac(tampered, event["signature"], HMAC_KEY)

    def test_header_signature_preferred(self) -> None:
        event = _sample_event()
        from ingest.signature import extract_signature

        assert extract_signature(event, "header-sig") == "header-sig"


class TestReplay:
    def test_accepts_strictly_increasing_seq(self) -> None:
        check_device_seq(100, 101, DEVICE_ID)

    def test_rejects_equal_seq(self) -> None:
        with pytest.raises(ReplayError):
            check_device_seq(100, 100, DEVICE_ID)

    def test_rejects_decreasing_seq(self) -> None:
        with pytest.raises(ReplayError):
            check_device_seq(100, 99, DEVICE_ID)


class TestAuth:
    def test_authenticate_valid_event(self) -> None:
        event = _sample_event(device_seq=5)
        seq = authenticate_event(event, None, hmac_key=HMAC_KEY, last_device_seq=4)
        assert seq == 5

    def test_authenticate_rejects_replay(self) -> None:
        event = _sample_event(device_seq=4)
        with pytest.raises(ReplayError):
            authenticate_event(event, None, hmac_key=HMAC_KEY, last_device_seq=4)

    def test_authenticate_rejects_bad_signature(self) -> None:
        event = _sample_event(device_seq=5)
        with pytest.raises(InvalidSignatureError):
            authenticate_event(event, "bad-signature", hmac_key=HMAC_KEY, last_device_seq=0)


class TestIngestAPI:
    @pytest.fixture()
    def client(self) -> TestClient:
        import sys
        from pathlib import Path

        server_dir = Path(__file__).resolve().parents[1] / "server"
        if str(server_dir) not in sys.path:
            sys.path.insert(0, str(server_dir))

        from main import app

        return TestClient(app)

    def _mock_conn(self) -> MagicMock:
        mock_cur = MagicMock()
        mock_cur.fetchone.return_value = None
        mock_conn = MagicMock()
        mock_conn.cursor.return_value.__enter__ = MagicMock(return_value=mock_cur)
        mock_conn.cursor.return_value.__exit__ = MagicMock(return_value=False)
        mock_cm = MagicMock()
        mock_cm.__enter__ = MagicMock(return_value=mock_conn)
        mock_cm.__exit__ = MagicMock(return_value=False)
        return mock_cm

    def test_replay_rejected_via_api(self, client: TestClient) -> None:
        event = _sample_event(device_seq=10)

        with patch("main.get_conn", return_value=self._mock_conn()):
            with patch("main.lookup_device", return_value=(HMAC_KEY, 10)):
                response = client.post("/v1/events", json=event)

        assert response.status_code == 409
        assert "Replay rejected" in response.json()["detail"]

    def test_tamper_rejected_via_api(self, client: TestClient) -> None:
        event = _sample_event(device_seq=11)
        event["hazard"]["rri"] = 0.01  # tamper after signing

        with patch("main.get_conn", return_value=self._mock_conn()):
            with patch("main.lookup_device", return_value=(HMAC_KEY, 0)):
                response = client.post("/v1/events", json=event)

        assert response.status_code == 401

    def test_valid_ingest_accepted(self, client: TestClient) -> None:
        event = _sample_event(device_seq=42)
        from main import IngestResult

        mock_conn = MagicMock()
        mock_cm = MagicMock()
        mock_cm.__enter__ = MagicMock(return_value=mock_conn)
        mock_cm.__exit__ = MagicMock(return_value=False)

        with patch("main.get_conn", return_value=mock_cm):
            with patch(
                "main.ingest_single_event",
                return_value=IngestResult(
                    event_id=event["event_id"],
                    hazard_id="hazard-uuid",
                    status="accepted",
                ),
            ):
                response = client.post("/v1/events", json=event)

        assert response.status_code == 200
        body = response.json()
        assert body["event_id"] == event["event_id"]
        assert body["status"] == "accepted"
        assert body["hazard_id"] == "hazard-uuid"

    def test_duplicate_returns_200(self, client: TestClient) -> None:
        event = _sample_event(device_seq=43)
        from main import IngestResult

        mock_conn = MagicMock()
        mock_cm = MagicMock()
        mock_cm.__enter__ = MagicMock(return_value=mock_conn)
        mock_cm.__exit__ = MagicMock(return_value=False)

        with patch("main.get_conn", return_value=mock_cm):
            with patch(
                "main.ingest_single_event",
                return_value=IngestResult(
                    event_id=event["event_id"],
                    hazard_id="existing-hazard",
                    status="duplicate",
                ),
            ):
                response = client.post("/v1/events", json=event)

        assert response.status_code == 200
        assert response.json()["status"] == "duplicate"

    def test_health_ok_without_db(self, client: TestClient) -> None:
        mock_conn = MagicMock()
        mock_cur = MagicMock()
        mock_conn.cursor.return_value.__enter__ = MagicMock(return_value=mock_cur)
        mock_conn.cursor.return_value.__exit__ = MagicMock(return_value=False)
        mock_cm = MagicMock()
        mock_cm.__enter__ = MagicMock(return_value=mock_conn)
        mock_cm.__exit__ = MagicMock(return_value=False)

        with patch("main.get_conn", return_value=mock_cm):
            response = client.get("/health")

        assert response.status_code == 200
        assert response.json()["status"] == "ok"

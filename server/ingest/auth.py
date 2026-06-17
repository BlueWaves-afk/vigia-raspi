"""Device lookup and ingest authentication orchestration."""

from __future__ import annotations

from typing import Any

from ingest.replay import ReplayError, check_device_seq
from ingest.signature import extract_signature, verify_hmac


class AuthError(Exception):
    """Base class for ingest authentication failures."""

    status_code = 401


class UnknownDeviceError(AuthError):
    status_code = 403

    def __init__(self, device_id: str) -> None:
        self.device_id = device_id
        super().__init__(f"Unknown device: {device_id}")


class InvalidSignatureError(AuthError):
    def __init__(self, device_id: str) -> None:
        self.device_id = device_id
        super().__init__(f"Invalid signature for device: {device_id}")


class MissingDeviceIdError(AuthError):
    def __init__(self) -> None:
        super().__init__("Missing device_id in payload")


def lookup_device(cur, device_id: str) -> tuple[str, int]:
    """Return (hmac_key, last_device_seq) for a registered device."""
    cur.execute(
        "SELECT hmac_key, last_device_seq FROM device_registry WHERE device_id = %s",
        (device_id,),
    )
    row = cur.fetchone()
    if row is None:
        raise UnknownDeviceError(device_id)
    return row[0], int(row[1])


def authenticate_event(
    payload: dict[str, Any],
    header_signature: str | None,
    *,
    hmac_key: str,
    last_device_seq: int,
) -> int:
    """
    Verify HMAC signature and anti-replay seq for a single event payload.

    Returns validated device_seq on success.
    """
    device_id = payload.get("device_id")
    if not isinstance(device_id, str) or not device_id:
        raise MissingDeviceIdError()

    device_seq = payload.get("device_seq")
    if not isinstance(device_seq, int):
        raise AuthError("device_seq must be an integer")

    signature = extract_signature(payload, header_signature)
    if not signature or not verify_hmac(payload, signature, hmac_key):
        raise InvalidSignatureError(device_id)

    check_device_seq(last_device_seq, device_seq, device_id)
    return device_seq

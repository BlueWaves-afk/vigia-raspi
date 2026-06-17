"""Monotonic device_seq anti-replay enforcement."""

from __future__ import annotations


class ReplayError(Exception):
    """Raised when device_seq is not strictly greater than last accepted seq."""

    def __init__(self, device_id: str, device_seq: int, last_seq: int) -> None:
        self.device_id = device_id
        self.device_seq = device_seq
        self.last_seq = last_seq
        super().__init__(
            f"Replay rejected for {device_id}: device_seq {device_seq} <= last {last_seq}"
        )


def check_device_seq(last_seq: int, device_seq: int, device_id: str) -> None:
    """Validate that device_seq advances strictly past the registry watermark."""
    if device_seq <= last_seq:
        raise ReplayError(device_id, device_seq, last_seq)


def advance_device_seq(cur, device_id: str, device_seq: int) -> None:
    """Update registry watermark after a successful ingest (same transaction)."""
    cur.execute(
        """
        UPDATE device_registry
        SET last_device_seq = %s
        WHERE device_id = %s AND last_device_seq < %s
        """,
        (device_seq, device_id, device_seq),
    )
    if cur.rowcount != 1:
        raise ReplayError(device_id, device_seq, device_seq - 1)

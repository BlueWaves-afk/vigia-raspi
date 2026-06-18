"""
Thin psycopg2 adapter satisfying the interface expected by attestation.py.

attestation.py calls:
  db.transaction()            — context manager that commits or rolls back
  db.get_last_sequence(dev)   — returns int or None
  db.update_last_sequence(dev, seq)
  db.get_device_cert(dev)     — returns cert PEM string or None
  db.get_root_ca()            — returns root CA PEM string or None
  db.log_verified_event(dev, event)
"""

from __future__ import annotations

import json
from contextlib import contextmanager
from typing import Any


class VigiaDb:
    def __init__(self, conn) -> None:
        self._conn = conn

    @contextmanager
    def transaction(self):
        try:
            yield
            self._conn.commit()
        except Exception:
            self._conn.rollback()
            raise

    def get_last_sequence(self, device_id: str) -> int | None:
        with self._conn.cursor() as cur:
            cur.execute(
                "SELECT last_device_seq FROM device_registry WHERE device_id = %s",
                (device_id,),
            )
            row = cur.fetchone()
        return int(row[0]) if row else None

    def update_last_sequence(self, device_id: str, sequence: int) -> None:
        with self._conn.cursor() as cur:
            cur.execute(
                "UPDATE device_registry SET last_device_seq = %s WHERE device_id = %s",
                (sequence, device_id),
            )

    def get_device_cert(self, device_id: str) -> str | None:
        with self._conn.cursor() as cur:
            cur.execute(
                "SELECT cert_pem FROM device_registry WHERE device_id = %s",
                (device_id,),
            )
            row = cur.fetchone()
        return row[0] if row else None

    def get_root_ca(self) -> str | None:
        with self._conn.cursor() as cur:
            cur.execute(
                "SELECT root_ca_pem FROM fleet_ca ORDER BY id DESC LIMIT 1"
            )
            row = cur.fetchone()
        return row[0] if row else None

    def log_verified_event(self, device_id: str, event: dict[str, Any]) -> None:
        with self._conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO attestation_log (device_id, verified_event)
                VALUES (%s, %s)
                ON CONFLICT DO NOTHING
                """,
                (device_id, json.dumps(event)),
            )

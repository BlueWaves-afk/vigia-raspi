"""HMAC-SHA256 signature verification over canonical JSON."""

from __future__ import annotations

import base64
import hashlib
import hmac
import json
from typing import Any


def canonical_json(payload: dict[str, Any]) -> bytes:
    """Serialize payload to canonical JSON (sorted keys, compact, no signature)."""
    cleaned = {k: v for k, v in payload.items() if k != "signature"}
    return json.dumps(cleaned, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode(
        "utf-8"
    )


def compute_hmac(payload: dict[str, Any], key: str) -> str:
    """Return base64-encoded HMAC-SHA256 over canonical JSON."""
    digest = hmac.new(key.encode("utf-8"), canonical_json(payload), hashlib.sha256).digest()
    return base64.b64encode(digest).decode("ascii")


def verify_hmac(payload: dict[str, Any], signature: str, key: str) -> bool:
    """Constant-time compare of provided signature against expected HMAC."""
    if not signature:
        return False
    expected = compute_hmac(payload, key)
    return hmac.compare_digest(signature, expected)


def extract_signature(payload: dict[str, Any], header_signature: str | None) -> str | None:
    """Prefer X-Vigia-Signature header, fall back to body signature field."""
    if header_signature:
        return header_signature.strip()
    body_sig = payload.get("signature")
    if isinstance(body_sig, str) and body_sig:
        return body_sig
    return None

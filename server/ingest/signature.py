"""HMAC-SHA256 and ECDSA-SHA256 signature verification over canonical JSON."""

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


def verify_ecdsa_header(payload: dict[str, Any], signature: str, cert_pem: str) -> bool:
    """
    Verify base64-encoded DER ECDSA-SHA256 signature over canonical JSON.
    The device signs canonical_json(payload) with its ATECC608A private key.
    """
    try:
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import ec
        from cryptography.exceptions import InvalidSignature
        from cryptography.x509 import load_pem_x509_certificate
        from cryptography.hazmat.backends import default_backend

        der_sig = base64.b64decode(signature)
        cert = load_pem_x509_certificate(cert_pem.encode(), default_backend())
        msg = canonical_json(payload)
        cert.public_key().verify(der_sig, msg, ec.ECDSA(hashes.SHA256()))
        return True
    except (InvalidSignature, Exception):
        return False

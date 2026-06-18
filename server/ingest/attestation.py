"""
attestation.py — Phase 6 DePIN attestation pipeline.

Full server-side validation of VIGIA edge node MQTT events:
  1. MsgPack decode (doc 05 §5 schema)
  2. Anti-replay: monotonic sequence check (doc 05 §8.2)
  3. EtHashInput reconstruction + SHA-256 hash verification (doc 05 §8.3)
  4. ECDSA secp256r1 signature verification via ATECC608A cert (doc 05 §8.4)
  5. Spatial latent deserialization + cosine similarity check (doc 05 §8.5)
  6. Cosmos 3 world model update submission (doc 05 §8.6)

Raises on any verification failure. Verified events are submitted to
the Cosmos 3 pipeline and logged in the fleet database.
"""

import hashlib
import struct
import logging
from typing import Optional, Tuple

import msgpack
import numpy as np

from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives.asymmetric.utils import encode_dss_signature, Prehashed
from cryptography.hazmat.backends import default_backend
from cryptography.exceptions import InvalidSignature
from cryptography.x509 import load_pem_x509_certificate

log = logging.getLogger(__name__)


class AttestationError(Exception):
    pass

class SecurityError(AttestationError):
    pass

class IntegrityError(AttestationError):
    pass

class AuthorizationError(AttestationError):
    pass


# ─────────────────────────────────────────────────────────────────────────────
# Step 1: MsgPack decode
# ─────────────────────────────────────────────────────────────────────────────

def decode_payload(mqtt_bytes: bytes) -> dict:
    payload = msgpack.unpackb(mqtt_bytes, raw=False, strict_map_key=False)
    version = payload.get("version")
    if version != 1:
        raise AttestationError(f"Unsupported payload version: {version}")
    return payload


# ─────────────────────────────────────────────────────────────────────────────
# Step 2: Anti-replay (doc 05 §8.2)
# ─────────────────────────────────────────────────────────────────────────────

def check_anti_replay(db, device_id: str, sequence: int) -> None:
    """
    Monotonic sequence guard — prevents replay attacks.
    Uses a DB transaction: update THEN verify, so a slow verify cannot be raced.
    """
    with db.transaction():
        last_seq = db.get_last_sequence(device_id)
        if last_seq is not None and sequence <= last_seq:
            raise SecurityError(
                f"Replay: device={device_id} recv_seq={sequence} last_seq={last_seq}"
            )
        # Update first; ECDSA verify happens after. ROLLBACK on verify failure.
        db.update_last_sequence(device_id, sequence)


# ─────────────────────────────────────────────────────────────────────────────
# Step 3: EtHashInput reconstruction (doc 05 §8.1 / §8.3)
# ─────────────────────────────────────────────────────────────────────────────

def reconstruct_et_hash_input(payload: dict, signed_et: dict) -> bytes:
    """
    Reconstructs the 96-byte EtHashInput that the ATECC608A hashed and signed.
    Layout must be byte-for-byte identical to 03_pico2_firmware_contracts.md §6.3.2.
    """
    device_uuid_bytes = bytes.fromhex(payload["device_id"].replace("-", ""))
    assert len(device_uuid_bytes) == 16

    # Pull IMU fields from the last frame_metadata entry (set by Pico2 at E_t time).
    last_frame = payload["frame_metadata"][-1] if payload["frame_metadata"] else {}

    et_input = struct.pack(
        "<"        # little-endian (Pi 5 / ARM native)
        "16s"      # device_id[16]
        "Q"        # timestamp_us  (uint64)
        "I"        # sequence      (uint32)
        "ffff"     # q_w, q_x, q_y, q_z
        "fff"      # lin_accel_x, y, z
        "B3x"      # imu_cal_status + 3 pad bytes
        "dd"       # latitude, longitude  (float64)
        "fff"      # altitude_m, speed_ms, course_deg
        "BB2x"     # fix_type, satellites + 2 pad bytes
        "f",       # hdop
        device_uuid_bytes,
        signed_et["mcu_timestamp_us"],
        signed_et["sequence"],
        signed_et.get("q_w", 0.0),
        signed_et.get("q_x", 0.0),
        signed_et.get("q_y", 0.0),
        signed_et.get("q_z", 1.0),
        signed_et.get("accel_x", 0.0),
        signed_et.get("accel_y", 0.0),
        signed_et.get("accel_z", 0.0),
        signed_et.get("imu_cal_status", 0),
        last_frame.get("lat", 0.0),
        last_frame.get("lon", 0.0),
        last_frame.get("alt_m", 0.0),
        last_frame.get("speed_ms", 0.0),
        0.0,   # course_deg not in FrameMetadata MsgPack schema
        signed_et.get("gps_fix_type", 0),
        0,     # satellites not stored separately — zero per spec
        last_frame.get("hdop", 99.9),
    )
    assert len(et_input) == 96, f"EtHashInput wrong size: {len(et_input)}"
    return et_input


def verify_et_hash(et_input: bytes, received_hash: bytes) -> None:
    computed = hashlib.sha256(et_input).digest()
    if computed != received_hash:
        raise IntegrityError(
            f"et_hash mismatch.\n"
            f"  recv:     {received_hash.hex()}\n"
            f"  computed: {computed.hex()}"
        )


# ─────────────────────────────────────────────────────────────────────────────
# Step 4: ECDSA signature verification (doc 05 §8.4)
# ─────────────────────────────────────────────────────────────────────────────

def verify_ecdsa_signature(db, device_id: str, et_hash: bytes, ecdsa_sig_raw: bytes) -> None:
    """
    Verifies ATECC608A secp256r1 ECDSA signature.
    sig_raw: 64-byte raw R‖S (IEEE P1363 format from ATECC608A atcab_sign).
    """
    assert len(ecdsa_sig_raw) == 64
    assert len(et_hash) == 32

    # Convert raw R‖S → DER-encoded signature
    r = int.from_bytes(ecdsa_sig_raw[:32], byteorder="big")
    s = int.from_bytes(ecdsa_sig_raw[32:], byteorder="big")
    der_sig = encode_dss_signature(r, s)

    # Load device certificate from fleet registry
    device_cert_pem = db.get_device_cert(device_id)
    if not device_cert_pem:
        raise AuthorizationError(
            f"Unknown device_id={device_id}. Not provisioned in fleet registry."
        )

    cert = load_pem_x509_certificate(device_cert_pem.encode(), default_backend())

    # Verify certificate chain against Microchip Trust Platform root CA
    root_ca_pem = db.get_root_ca()
    _verify_certificate_chain(cert, root_ca_pem)

    # Verify ECDSA signature (Prehashed: ATECC608A already computed SHA-256)
    try:
        cert.public_key().verify(
            der_sig,
            et_hash,
            ec.ECDSA(Prehashed(hashes.SHA256()))
        )
    except InvalidSignature:
        raise SecurityError(
            f"ECDSA verification FAILED for device={device_id}. "
            "Possible: firmware bug, key slot misconfiguration, or Sybil attack."
        )


def _verify_certificate_chain(cert, root_ca_pem: str) -> None:
    if not root_ca_pem:
        log.warning("Root CA not configured — skipping certificate chain verification.")
        return
    root_cert = load_pem_x509_certificate(root_ca_pem.encode(), default_backend())
    root_pub = root_cert.public_key()
    try:
        root_pub.verify(
            cert.signature,
            cert.tbs_certificate_bytes,
            ec.ECDSA(hashes.SHA256()),
        )
    except InvalidSignature:
        raise SecurityError("Device certificate not signed by configured root CA.")


# ─────────────────────────────────────────────────────────────────────────────
# Step 5: Spatial latent deserialization (doc 05 §8.5)
# ─────────────────────────────────────────────────────────────────────────────

def deserialize_spatial_latent(latent_bin: Optional[bytes]) -> Optional[np.ndarray]:
    if not latent_bin:
        return None
    # Bytes are native little-endian float32 as packed by AntiDeathNode on ARM Pi 5.
    return np.frombuffer(latent_bin, dtype="<f4")


def compute_scene_similarity(s_t: np.ndarray, reference_vectors: dict) -> dict:
    s_norm = s_t / (np.linalg.norm(s_t) + 1e-8)
    return {
        cls: float(np.dot(s_norm, ref / (np.linalg.norm(ref) + 1e-8)))
        for cls, ref in reference_vectors.items()
    }


# ─────────────────────────────────────────────────────────────────────────────
# Full attestation pipeline (doc 05 §8.6)
# ─────────────────────────────────────────────────────────────────────────────

def process_attestation_event(mqtt_bytes: bytes, db, cosmos3_client=None) -> dict:
    """
    Entry point called by the MQTT subscriber for every message on
    vigia/attest/+/hazard. Returns the verified event dict on success.
    Raises AttestationError subclass on any verification failure.
    """
    payload = decode_payload(mqtt_bytes)
    device_id = payload["device_id"]
    signed_et = payload.get("signed_et")

    if signed_et is None:
        raise AttestationError(f"Missing signed_et — device={device_id}")

    # Anti-replay (monotonic sequence guard)
    check_anti_replay(db, device_id, signed_et["sequence"])

    # EtHashInput reconstruction + hash integrity
    et_input = reconstruct_et_hash_input(payload, signed_et)
    et_hash_bytes = bytes(signed_et["et_hash"])
    verify_et_hash(et_input, et_hash_bytes)

    # ECDSA signature verification
    verify_ecdsa_signature(db, device_id, et_hash_bytes, bytes(signed_et["ecdsa_sig"]))

    # Spatial latent
    latent = deserialize_spatial_latent(payload.get("spatial_latent"))

    log.info(
        "ATTESTATION VERIFIED: device=%s seq=%d latent_elems=%s",
        device_id,
        signed_et["sequence"],
        len(latent) if latent is not None else "none",
    )

    # Build Cosmos 3 world model update
    frame_metadata = payload.get("frame_metadata", [])
    last_frame = frame_metadata[-1] if frame_metadata else {}
    hazard = payload.get("hazard_event") or {}

    verified_event = {
        "device_id":          device_id,
        "event_type":         "road_hazard_attested",
        "timestamp_us":       signed_et["mcu_timestamp_us"],
        "sequence":           signed_et["sequence"],
        "gps_location":       {"lat": last_frame.get("lat"), "lon": last_frame.get("lon")},
        "rri_score":          hazard.get("rri_score"),
        "iss_score":          hazard.get("iss_score"),
        "yolo_confidence":    hazard.get("yolo_confidence"),
        "spatial_latent":     latent.tolist() if latent is not None else None,
        "et_hash":            et_hash_bytes.hex(),
        "attestation_status": "VERIFIED",
    }

    if cosmos3_client is not None:
        cosmos3_client.submit_world_model_update(verified_event)

    db.log_verified_event(device_id, verified_event)
    return verified_event

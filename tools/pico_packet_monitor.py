#!/usr/bin/env python3
"""Decode COBS SignedEtPacket frames from Pico Phase 2 firmware."""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

SIGNED_ET_SIZE = 173
SIGNED_ET_MAGIC = 0xE7
SIGNED_ET_VERSION = 0x02


@dataclass
class SignedEtPacket:
    magic: int
    version: int
    timestamp_us: int
    sequence: int
    qw: float
    qx: float
    qy: float
    qz: float
    ax: float
    ay: float
    az: float
    cal_status: int
    latitude: float
    longitude: float
    speed_ms: float
    fix_type: int
    satellites: int
    et_hash: bytes
    ecdsa_sig: bytes


def cobs_decode(src: bytes) -> bytes:
    """COBS decode — matches sensor_bridge_node.cpp decode_cobs()."""
    if len(src) < 2:
        return b""
    dst = bytearray()
    read = 0
    while read < len(src):
        code = src[read]
        read += 1
        if code == 0:
            break
        for _ in range(1, code):
            if read >= len(src):
                return b""
            dst.append(src[read])
            read += 1
        if code < 0xFF:
            dst.append(0x00)
    # Strip trailing structural zero (same as Pi sensor_bridge)
    if dst and dst[-1] == 0x00:
        dst.pop()
    return bytes(dst)


def parse_signed_et(data: bytes) -> SignedEtPacket | None:
    """Parse 173-byte SignedEtPacket (must match firmware atecc608a_driver.h)."""
    # USB framing sometimes yields one extra trailing 0x00 after decode
    if len(data) == SIGNED_ET_SIZE + 1 and data[-1] == 0x00:
        data = data[:SIGNED_ET_SIZE]
    if len(data) != SIGNED_ET_SIZE:
        return None
    if data[0] != SIGNED_ET_MAGIC or data[1] != SIGNED_ET_VERSION:
        return None
    try:
        (
            magic,
            version,
            timestamp_us,
            sequence,
            qw,
            qx,
            qy,
            qz,
            ax,
            ay,
            az,
            cal_status,
        ) = struct.unpack_from("<BBQ I 7f B", data, 0)
        latitude, longitude, speed_ms, fix_type, satellites = struct.unpack_from(
            "<ddfBB", data, 46
        )
        et_hash = data[69:101]
        ecdsa_sig = data[101:165]
    except struct.error:
        return None
    return SignedEtPacket(
        magic=magic,
        version=version,
        timestamp_us=timestamp_us,
        sequence=sequence,
        qw=qw,
        qx=qx,
        qy=qy,
        qz=qz,
        ax=ax,
        ay=ay,
        az=az,
        cal_status=cal_status,
        latitude=latitude,
        longitude=longitude,
        speed_ms=speed_ms,
        fix_type=fix_type,
        satellites=satellites,
        et_hash=et_hash,
        ecdsa_sig=ecdsa_sig,
    )


def load_pubkey_hex(path: Path) -> bytes | None:
    text = path.read_text().strip().replace(":", "").replace(" ", "").replace("\n", "")
    if len(text) != 128:
        return None
    return bytes.fromhex(text)


def raw_rs_to_der(sig: bytes) -> bytes:
    """ATECC608A / IEEE P1363 raw R||S → ASN.1 DER for cryptography.verify()."""
    from cryptography.hazmat.primitives.asymmetric.utils import encode_dss_signature

    if len(sig) != 64:
        raise ValueError(f"expected 64-byte raw signature, got {len(sig)}")
    r = int.from_bytes(sig[:32], "big")
    s = int.from_bytes(sig[32:], "big")
    return encode_dss_signature(r, s)


def verify_ecdsa_simple(pubkey: bytes, digest: bytes, sig: bytes) -> bool:
    if len(sig) != 64 or all(b == 0 for b in sig):
        return False
    if len(digest) != 32 or all(b == 0 for b in digest):
        return False
    try:
        from cryptography.hazmat.primitives.asymmetric import ec
        from cryptography.hazmat.primitives.asymmetric.utils import Prehashed
        from cryptography.hazmat.primitives.hashes import SHA256
        from cryptography.exceptions import InvalidSignature
    except ImportError:
        return False

    public_key = ec.EllipticCurvePublicKey.from_encoded_point(
        ec.SECP256R1(), b"\x04" + pubkey
    )
    try:
        public_key.verify(
            raw_rs_to_der(sig), digest, ec.ECDSA(Prehashed(SHA256()))
        )
        return True
    except (InvalidSignature, ValueError):
        return False


def open_serial(port: str, baud: int):
    import serial

    return serial.Serial(port, baudrate=baud, timeout=0.2)


def monitor(port: str, baud: int, duration: float, pubkey_path: Path | None) -> int:
    pubkey = load_pubkey_hex(pubkey_path) if pubkey_path else None
    if pubkey_path and pubkey is None:
        print(f"ERROR: invalid pubkey file {pubkey_path} (need 128 hex chars)", file=sys.stderr)
        return 1

    ser = open_serial(port, baud)

    acc = bytearray()
    in_frame = False
    count = 0
    valid_sigs = 0
    warned_stub = False
    start = time.monotonic()

    print(f"Listening on {port} @ {baud} for {duration:.0f}s (Ctrl+C to stop early)")

    try:
        while time.monotonic() - start < duration:
            chunk = ser.read(256)
            if not chunk:
                continue
            for byte in chunk:
                if byte == 0x00:
                    if acc:
                        decoded = cobs_decode(bytes(acc))
                        pkt = parse_signed_et(decoded)
                        if pkt:
                            count += 1
                            sig_ok = False
                            if pubkey:
                                sig_ok = verify_ecdsa_simple(
                                    pubkey, pkt.et_hash, pkt.ecdsa_sig
                                )
                                if sig_ok:
                                    valid_sigs += 1
                                elif not warned_stub and all(
                                    b == 0 for b in pkt.ecdsa_sig
                                ):
                                    print(
                                        "[warn] ecdsa_sig is all zeros — "
                                        "flash vigia_pico_phase2_live.uf2 (not stub/hello)",
                                        file=sys.stderr,
                                    )
                                    warned_stub = True
                            print(
                                f"seq={pkt.sequence} ts={pkt.timestamp_us} "
                                f"lat={pkt.latitude:.7f} lon={pkt.longitude:.7f} "
                                f"sig_valid={sig_ok if pubkey else 'n/a'}"
                            )
                        else:
                            print(f"[warn] bad packet ({len(decoded)} bytes decoded)")
                    acc.clear()
                    in_frame = False
                    continue
                if not in_frame:
                    in_frame = True
                    acc.clear()
                acc.append(byte)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    elapsed = max(time.monotonic() - start, 0.001)
    print(f"\nPackets: {count} ({count / elapsed:.2f}/s)")
    if pubkey:
        print(f"Valid signatures: {valid_sigs}/{count}")
    return 0 if count > 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--pubkey", type=Path, help="64-byte secp256r1 pubkey hex file")
    args = parser.parse_args()
    return monitor(args.port, args.baud, args.duration, args.pubkey)


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""One-shot patch for Pi: fix pico_packet_monitor.py parse + COBS + ECDSA verify.

Run on the Pi:
  python3 patch_packet_monitor_on_pi.py ~/vigia-raspi/tools/pico_packet_monitor.py
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "struct.unpack_from(\"<BBQ I 7f B\", data, 0)"
OLD_MARKER = 'struct.unpack("<BBQ I 7f B 3x d d f B B x 32s 64s 8x", data)'

PATCHED_PARSE = '''def parse_signed_et(data: bytes) -> SignedEtPacket | None:
    """Parse 173-byte SignedEtPacket (must match firmware atecc608a_driver.h)."""
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
'''

RAW_RS_FN = '''def raw_rs_to_der(sig: bytes) -> bytes:
    """ATECC608A / IEEE P1363 raw R||S → ASN.1 DER for cryptography.verify()."""
    from cryptography.hazmat.primitives.asymmetric.utils import encode_dss_signature

    if len(sig) != 64:
        raise ValueError(f"expected 64-byte raw signature, got {len(sig)}")
    r = int.from_bytes(sig[:32], "big")
    s = int.from_bytes(sig[32:], "big")
    return encode_dss_signature(r, s)

'''


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} path/to/pico_packet_monitor.py", file=sys.stderr)
        return 1

    path = Path(sys.argv[1])
    text = path.read_text()

    if MARKER in text and "raw_rs_to_der" in text:
        print(f"already patched: {path}")
        return 0

    if OLD_MARKER not in text and "def parse_signed_et" not in text:
        print(f"unrecognized file layout: {path}", file=sys.stderr)
        return 1

    # Replace entire parse_signed_et through its return SignedEtPacket block
    start = text.index("def parse_signed_et")
    end = text.index("\n\n", text.index("ecdsa_sig=ecdsa_sig,\n    )", start) + 1)
    text = text[:start] + PATCHED_PARSE + text[end:]

    if "raw_rs_to_der" not in text:
        anchor = "def load_pubkey_hex"
        text = text.replace(anchor, RAW_RS_FN + anchor)
        text = text.replace(
            "public_key.verify(sig, digest, ec.ECDSA(Prehashed(SHA256())))",
            "public_key.verify(\n            raw_rs_to_der(sig), digest, ec.ECDSA(Prehashed(SHA256()))\n        )",
        )

    path.write_text(text)
    print(f"patched: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

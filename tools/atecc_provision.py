#!/usr/bin/env python3
"""
ATECC608P provisioning helper for Vigia Phase 2.

Exports device public keys and documents one-time slot-0 provisioning.
When cryptoauthlib Python bindings are available, can probe the secure element
over an I2C adapter; on the Vigia bench, provisioning is typically done via
Pico live firmware + Microchip Trust Platform tools.

Usage:
  python3 tools/atecc_provision.py --device-id vigia-pico-001 --pubkey-hex <128-hex-chars>
  python3 tools/atecc_provision.py --device-id vigia-pico-001 --read-pubkey config/device_keys/existing.pub
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
KEYS_DIR = ROOT / "config" / "device_keys"
REGISTRY_FILE = KEYS_DIR / "registry.json"


def normalize_hex(text: str) -> str:
    cleaned = "".join(ch for ch in text if ch in "0123456789abcdefABCDEF")
    return cleaned.lower()


def save_pubkey(device_id: str, pubkey_hex: str, serial: str | None = None) -> Path:
    KEYS_DIR.mkdir(parents=True, exist_ok=True)
    pubkey_hex = normalize_hex(pubkey_hex)
    if len(pubkey_hex) != 128:
        raise ValueError(f"pubkey must be 128 hex chars (64 bytes), got {len(pubkey_hex)}")

    out_path = KEYS_DIR / f"{device_id}.pub"
    out_path.write_text(pubkey_hex + "\n")

    registry: dict = {}
    if REGISTRY_FILE.exists():
        registry = json.loads(REGISTRY_FILE.read_text())

    registry[device_id] = {
        "device_id": device_id,
        "pubkey_file": str(out_path.relative_to(ROOT)),
        "pubkey_hex": pubkey_hex,
        "serial": serial,
        "slot": 0,
        "provisioned_at": datetime.now(timezone.utc).isoformat(),
    }
    REGISTRY_FILE.write_text(json.dumps(registry, indent=2) + "\n")
    return out_path


def try_cryptoauthlib_probe() -> None:
    try:
        import cryptoauthlib  # type: ignore
    except ImportError:
        print(
            "cryptoauthlib Python module not installed.\n"
            "For host-side I2C provisioning use Microchip Trust Platform or:\n"
            "  pip install cryptoauthlib\n"
            "On the Vigia bench, provision slot 0 via Pico live firmware + atcab_genkey(0)."
        )
        return
    print(f"cryptoauthlib version: {getattr(cryptoauthlib, '__version__', 'unknown')}")
    print("Host I2C provisioning requires a USB-I2C adapter wired to the SE.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device-id", required=True, help="Fleet device identifier")
    parser.add_argument("--pubkey-hex", help="128-char hex pubkey (X||Y)")
    parser.add_argument("--read-pubkey", type=Path, help="Import pubkey from existing file")
    parser.add_argument("--serial", help="ATECC608P serial number (hex)")
    parser.add_argument("--probe", action="store_true", help="Check cryptoauthlib availability")
    args = parser.parse_args()

    if args.probe:
        try_cryptoauthlib_probe()
        return 0

    pubkey_hex = args.pubkey_hex
    if args.read_pubkey:
        pubkey_hex = args.read_pubkey.read_text()

    if not pubkey_hex:
        print("Provide --pubkey-hex or --read-pubkey", file=sys.stderr)
        return 1

    out = save_pubkey(args.device_id, pubkey_hex, args.serial)
    print(f"Saved pubkey to {out}")
    print(f"Registry updated: {REGISTRY_FILE}")
    print(
        "\nNext steps:\n"
        "  1. Wire ATECC608P (SDA→GP2, SCL→GP3, VCC/GND to sensor rail)\n"
        "  2. Flash vigia_pico_phase2_live.uf2\n"
        "  3. python3 tools/pico_packet_monitor.py --pubkey config/device_keys/"
        f"{args.device_id}.pub\n"
        "  4. Set VIGIA_PUBKEY_FILE on Pi for vigia_app SensorBridge verify"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

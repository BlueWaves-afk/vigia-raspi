#!/usr/bin/env python3
"""
vigia-pair-qr.py — Provision the Pi identity key and generate the pairing QR code.

Usage:
    sudo python3 tools/vigia-pair-qr.py [--provision] [--id vigia-001]

Flags:
    --provision   Generate /etc/vigia/pi_p256.der if it does not already exist.
                  Must run as root (or with sudo) so it can write to /etc/vigia/.
    --id ID       Device ID embedded in the QR code (default: vigia-001).
    --key PATH    Path to the PKCS#8 DER private key (default: /etc/vigia/pi_p256.der).

The QR code encodes:
    vigia://pair?mac=AA:BB:CC:DD:EE:FF&pk=<base64url P-256 pub>&id=vigia-001&v=1

Android side reads this with CameraX + ML Kit and calls CompanionDeviceManager.associate().
"""

import argparse
import base64
import os
import stat
import struct
import subprocess
import sys

try:
    from cryptography.hazmat.primitives.asymmetric.ec import (
        SECP256R1,
        generate_private_key,
        EllipticCurvePublicKey,
    )
    from cryptography.hazmat.primitives.serialization import (
        Encoding,
        PublicFormat,
        PrivateFormat,
        NoEncryption,
        load_der_private_key,
    )
    from cryptography.hazmat.backends import default_backend
except ImportError:
    sys.exit("ERROR: python3-cryptography is required. Install: sudo apt install python3-cryptography")


KEY_PATH_DEFAULT = "/etc/vigia/pi_p256.der"


def get_ble_mac(adapter: str = "hci0") -> str:
    """Read the BLE MAC address from hciconfig."""
    try:
        out = subprocess.check_output(["hciconfig", adapter], text=True)
        for line in out.splitlines():
            line = line.strip()
            if line.startswith("BD Address:"):
                return line.split()[2]
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    # Fallback: read from sysfs
    try:
        with open(f"/sys/class/bluetooth/{adapter}/address") as f:
            return f.read().strip().upper()
    except OSError:
        return "00:00:00:00:00:00"


def provision_key(key_path: str) -> None:
    """Generate a P-256 key and write it to key_path as PKCS#8 DER if not present."""
    if os.path.exists(key_path):
        print(f"[ok] Key already exists at {key_path} — skipping generation.")
        return

    key_dir = os.path.dirname(key_path)
    os.makedirs(key_dir, mode=0o700, exist_ok=True)

    key = generate_private_key(SECP256R1(), default_backend())
    der_bytes = key.private_bytes(Encoding.DER, PrivateFormat.PKCS8, NoEncryption())

    with open(key_path, "wb") as f:
        f.write(der_bytes)
    os.chmod(key_path, stat.S_IRUSR | stat.S_IWUSR)  # 0600

    print(f"[ok] Generated P-256 identity key → {key_path}")


def load_public_key_uncompressed(key_path: str) -> bytes:
    """Load PKCS#8 DER and return the uncompressed public key point (65 bytes: 0x04 || X || Y)."""
    with open(key_path, "rb") as f:
        der = f.read()
    key = load_der_private_key(der, password=None, backend=default_backend())
    pub: EllipticCurvePublicKey = key.public_key()
    # Uncompressed point: X9.62 format — first byte is 0x04
    raw = pub.public_bytes(Encoding.X962, PublicFormat.UncompressedPoint)
    assert len(raw) == 65 and raw[0] == 0x04, f"Unexpected public key format: {raw[:2].hex()}"
    return raw


def generate_qr_uri(mac: str, pub65: bytes, device_id: str) -> str:
    pk_b64 = base64.urlsafe_b64encode(pub65).rstrip(b"=").decode()
    return f"vigia://pair?mac={mac}&pk={pk_b64}&id={device_id}&v=1"


def print_qr(uri: str) -> None:
    """Print QR code to terminal using qrencode or fallback to raw URI."""
    try:
        subprocess.run(["qrencode", "-t", "UTF8", "-o", "-", uri], check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("[!] qrencode not found — install with: sudo apt install qrencode")
        print()
        print("Pairing URI (copy this to a QR generator if needed):")
        print(uri)


def main() -> None:
    parser = argparse.ArgumentParser(description="VIGIA BLE pairing QR provisioner")
    parser.add_argument("--provision", action="store_true",
                        help="Generate identity key if not present (requires root)")
    parser.add_argument("--id", default="vigia-001", metavar="DEVICE_ID",
                        help="Device ID embedded in QR (default: vigia-001)")
    parser.add_argument("--key", default=KEY_PATH_DEFAULT, metavar="PATH",
                        help=f"Path to PKCS#8 DER private key (default: {KEY_PATH_DEFAULT})")
    parser.add_argument("--adapter", default="hci0", metavar="ADAPTER",
                        help="Bluetooth adapter name (default: hci0)")
    args = parser.parse_args()

    if args.provision:
        if os.geteuid() != 0:
            sys.exit("ERROR: --provision requires root. Run with sudo.")
        provision_key(args.key)

    if not os.path.exists(args.key):
        sys.exit(
            f"ERROR: Identity key not found at {args.key}.\n"
            f"Run with --provision to generate it: sudo python3 {sys.argv[0]} --provision"
        )

    pub65 = load_public_key_uncompressed(args.key)
    mac   = get_ble_mac(args.adapter)
    uri   = generate_qr_uri(mac, pub65, args.id)

    print(f"Device ID  : {args.id}")
    print(f"BLE MAC    : {mac}")
    print(f"Public key : {pub65.hex()}")
    print(f"Pairing URI: {uri}")
    print()
    print_qr(uri)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
provision_device.py — One-time provisioning for a VIGIA edge node.

Automates the steps in docs/device_provisioning.md:
  1. Generate Ed25519 device keypair → /etc/vigia/device_ed25519.key (seed, 32 bytes)
                                     → /etc/vigia/device_pubkey.b58  (base58 public key)
  2. Register device with AWS backend (POST /register-device)
  3. Provision P-256 BLE identity key  → /etc/vigia/pi_p256.der
  4. Run signing smoke test

Usage (on the Pi, as root):
    sudo python3 tools/provision_device.py [--api-url URL] [--id DEVICE_ID] [--skip-ble] [--skip-register]

Flags:
    --api-url URL      AWS API Gateway base URL (without trailing slash)
    --id DEVICE_ID     Human-readable device identifier (default: vigia-001)
    --skip-ble         Skip P-256 BLE key generation (if already done with vigia-pair-qr.py)
    --skip-register    Skip AWS device registration (offline provisioning)
    --force            Re-generate keys even if they already exist (DANGER: changes device identity)
"""

import argparse
import json
import os
import stat
import sys
import urllib.error
import urllib.request

API_URL_DEFAULT = "https://sq2ri2n51g.execute-api.us-east-1.amazonaws.com/prod"
KEY_DIR = "/etc/vigia"
ED25519_KEY_PATH = f"{KEY_DIR}/device_ed25519.key"
ED25519_PUB_PATH = f"{KEY_DIR}/device_pubkey.b58"
P256_DER_PATH    = f"{KEY_DIR}/pi_p256.der"
DEVICE_ID_PATH   = f"{KEY_DIR}/device_id"


def require_root() -> None:
    if os.geteuid() != 0:
        sys.exit("ERROR: This script must be run as root (sudo).")


def ensure_key_dir() -> None:
    os.makedirs(KEY_DIR, mode=0o700, exist_ok=True)


# ── Step 1: Ed25519 keypair ────────────────────────────────────────────────────

def provision_ed25519(force: bool) -> str:
    """Generate Ed25519 seed + write public key in base58. Returns public key string."""
    try:
        import nacl.signing
        import nacl.encoding
    except ImportError:
        sys.exit("ERROR: pynacl required. Run: pip3 install pynacl")

    if os.path.exists(ED25519_KEY_PATH) and not force:
        print(f"[ok] Ed25519 key already exists at {ED25519_KEY_PATH} — skipping.")
        # Read existing public key
        key = nacl.signing.SigningKey(open(ED25519_KEY_PATH, "rb").read())
        pubkey_b58 = key.verify_key.encode(encoder=nacl.encoding.Base58Encoder).decode()
        print(f"      Public key (base58): {pubkey_b58}")
        return pubkey_b58

    key = nacl.signing.SigningKey.generate()

    # Write 32-byte seed (private key)
    with open(ED25519_KEY_PATH, "wb") as f:
        f.write(bytes(key))
    os.chmod(ED25519_KEY_PATH, stat.S_IRUSR | stat.S_IWUSR)  # 0600

    # Write base58 public key
    pubkey_b58 = key.verify_key.encode(encoder=nacl.encoding.Base58Encoder).decode()
    with open(ED25519_PUB_PATH, "w") as f:
        f.write(pubkey_b58 + "\n")
    os.chmod(ED25519_PUB_PATH, 0o644)

    print(f"[ok] Ed25519 keypair generated.")
    print(f"      Seed (private) → {ED25519_KEY_PATH} (0600)")
    print(f"      Public key     → {ED25519_PUB_PATH}")
    print(f"      Public key (base58): {pubkey_b58}")
    return pubkey_b58


# ── Step 2: AWS device registration ───────────────────────────────────────────

def register_device(api_url: str, pubkey_b58: str) -> None:
    url = f"{api_url.rstrip('/')}/register-device"
    payload = json.dumps({"device_address": pubkey_b58}).encode()
    req = urllib.request.Request(
        url,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            body = resp.read().decode()
            data = json.loads(body)
            status = data.get("status", "unknown")
            print(f"[ok] Device registered → status={status}")
            print(f"      Response: {body}")
    except urllib.error.HTTPError as e:
        body = e.read().decode()
        print(f"[!] Registration HTTP {e.code}: {body}")
        # 409 = already registered — treat as success
        if e.code == 409:
            print("[ok] Device already registered (HTTP 409) — continuing.")
        else:
            sys.exit(f"ERROR: Registration failed (HTTP {e.code})")
    except urllib.error.URLError as e:
        sys.exit(f"ERROR: Cannot reach {url}: {e.reason}")


# ── Step 3: P-256 BLE identity key ────────────────────────────────────────────

def provision_p256(force: bool) -> None:
    try:
        from cryptography.hazmat.primitives.asymmetric.ec import (
            SECP256R1, generate_private_key,
        )
        from cryptography.hazmat.primitives.serialization import (
            Encoding, PrivateFormat, NoEncryption,
        )
        from cryptography.hazmat.backends import default_backend
    except ImportError:
        sys.exit("ERROR: python3-cryptography required. Run: sudo apt install python3-cryptography")

    if os.path.exists(P256_DER_PATH) and not force:
        print(f"[ok] P-256 BLE key already exists at {P256_DER_PATH} — skipping.")
        print("      To regenerate: re-run with --force (also re-generate QR code).")
        return

    key = generate_private_key(SECP256R1(), default_backend())
    der = key.private_bytes(Encoding.DER, PrivateFormat.PKCS8, NoEncryption())

    with open(P256_DER_PATH, "wb") as f:
        f.write(der)
    os.chmod(P256_DER_PATH, stat.S_IRUSR | stat.S_IWUSR)  # 0600

    print(f"[ok] P-256 BLE identity key generated → {P256_DER_PATH} (0600)")
    print("      Run vigia-pair-qr.py to generate the pairing QR code for the Android app.")


# ── Step 4: Write device_id ────────────────────────────────────────────────────

def write_device_id(device_id: str) -> None:
    with open(DEVICE_ID_PATH, "w") as f:
        f.write(device_id + "\n")
    os.chmod(DEVICE_ID_PATH, 0o644)
    print(f"[ok] Device ID written → {DEVICE_ID_PATH}: {device_id}")


# ── Step 5: Smoke test ────────────────────────────────────────────────────────

def smoke_test() -> None:
    try:
        import nacl.signing
    except ImportError:
        print("[!] Skipping smoke test — pynacl not available.")
        return

    key = nacl.signing.SigningKey(open(ED25519_KEY_PATH, "rb").read())
    msg = b"VIGIA:pothole:12.971600:77.594600:2026-06-17T10:00:00.000Z:0.8200"
    signed = key.sign(msg)
    sig = signed.signature
    assert len(sig) == 64, f"Unexpected sig length: {len(sig)}"
    print(f"[ok] Smoke test passed — Ed25519 signature is 64 bytes.")


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="VIGIA edge node one-time provisioning")
    parser.add_argument("--api-url", default=API_URL_DEFAULT,
                        help=f"AWS API Gateway base URL (default: {API_URL_DEFAULT})")
    parser.add_argument("--id", default="vigia-001", metavar="DEVICE_ID",
                        help="Device ID (default: vigia-001)")
    parser.add_argument("--skip-ble", action="store_true",
                        help="Skip P-256 BLE key generation")
    parser.add_argument("--skip-register", action="store_true",
                        help="Skip AWS device registration (offline)")
    parser.add_argument("--force", action="store_true",
                        help="Regenerate keys even if they already exist (changes device identity)")
    args = parser.parse_args()

    require_root()
    ensure_key_dir()

    print("=== VIGIA Device Provisioning ===")
    print(f"Device ID : {args.id}")
    print(f"API URL   : {args.api_url}")
    print()

    # 1. Ed25519
    print("Step 1/4: Ed25519 device keypair")
    pubkey_b58 = provision_ed25519(args.force)
    print()

    # 2. AWS registration
    if not args.skip_register:
        print("Step 2/4: AWS device registration")
        register_device(args.api_url, pubkey_b58)
    else:
        print("Step 2/4: AWS registration skipped (--skip-register)")
    print()

    # 3. P-256 BLE key
    if not args.skip_ble:
        print("Step 3/4: P-256 BLE identity key")
        provision_p256(args.force)
    else:
        print("Step 3/4: P-256 BLE key skipped (--skip-ble)")
    print()

    # 4. Device ID file
    print("Step 4/4: Device ID + smoke test")
    write_device_id(args.id)
    smoke_test()
    print()

    print("=== Provisioning complete ===")
    print()
    print("Next steps:")
    print(f"  1. Generate pairing QR: sudo python3 tools/vigia-pair-qr.py --id {args.id}")
    print("  2. Install systemd service: sudo cp config/vigia-edge.service /etc/systemd/system/")
    print("                              sudo systemctl daemon-reload")
    print("                              sudo systemctl enable --now vigia-edge.service")
    print("  3. Rebuild ROS2 workspace: cd ~/vigia_ws && colcon build --packages-select vigia_edge_node vigia_msgs")


if __name__ == "__main__":
    main()

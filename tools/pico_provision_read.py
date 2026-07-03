#!/usr/bin/env python3
"""Read VIGIA_PROVISION output from Pico and save the ATECC608 public key."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from atecc_provision import save_pubkey  # noqa: E402


def open_serial(port: str, baud: int):
    import serial

    # Plain open — same as the working monitoring tools.
    # macOS Pico CDC requires DTR asserted (the default) to flow data.
    ser = serial.Serial(port, baudrate=baud, timeout=0.2)
    ser.reset_input_buffer()
    return ser


def read_provision(port: str, baud: int, timeout_s: float) -> dict[str, str]:
    ser = open_serial(port, baud)
    lines: dict[str, str] = {}
    raw_lines: list[str] = []
    start = time.monotonic()

    print(f"Listening on {port} for VIGIA_PROVISION output ({timeout_s:.0f}s)...")
    print("The firmware repeats its output every ~5 s — no RESET needed.\n")

    try:
        while time.monotonic() - start < timeout_s:
            chunk = ser.read(512)
            if not chunk:
                continue
            for line in chunk.decode("utf-8", errors="replace").splitlines():
                line = line.strip()
                if not line:
                    continue
                raw_lines.append(line)
                print(line)
                if not line.startswith("VIGIA_PROVISION "):
                    continue
                body = line[len("VIGIA_PROVISION "):]
                if "=" in body:
                    key, value = body.split("=", 1)
                    lines[key.strip()] = value.strip()
                if lines.get("status") in ("ok", "error"):
                    break
            if lines.get("status") in ("ok", "error"):
                break
    finally:
        ser.close()

    if not raw_lines:
        print("\n(no bytes received — wrong port, wrong UF2, or port held by another app)")
        print("Try: lsof 2>/dev/null | grep usbmodem")
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Pico USB serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--device-id", default="vigia-pico-001")
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    info = read_provision(args.port, args.baud, args.timeout)

    if info.get("status") == "error":
        step = info.get("step", "unknown")
        print(f"\nERROR: provisioning failed at step={step}", file=sys.stderr)
        for key in ("rc", "get_pubkey_rc", "genkey_rc", "config_write_rc"):
            if key in info:
                print(f"  {key}={info[key]}", file=sys.stderr)
        return 1

    if info.get("status") != "ok":
        print("ERROR: did not see VIGIA_PROVISION status=ok", file=sys.stderr)
        print("Tip: press RESET on the Pico immediately after running this script.",
              file=sys.stderr)
        return 1

    pubkey_hex = info.get("pubkey")
    if not pubkey_hex or len(pubkey_hex) != 128:
        print("ERROR: missing or invalid pubkey line", file=sys.stderr)
        return 1

    out = save_pubkey(args.device_id, pubkey_hex, serial=info.get("serial"))
    print(f"\nSaved public key to {out}")
    print(f"generated_new={info.get('generated', '?')}")
    print(
        "\nNext steps:\n"
        "  1. Reflash firmware/build-phase2-live/vigia_pico_phase2_live.uf2\n"
        "  2. python3 tools/pico_packet_monitor.py "
        f"--port {args.port} --duration 30 --pubkey {out}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

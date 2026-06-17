#!/usr/bin/env python3
"""
pico_bridge_monitor.py — verify Pi ↔ Pico 2 USB CDC link.

Reads VIGIA_PING lines from /dev/ttyACM0 (or --port) and reports rate / gaps.

Usage (on the Pi):
    python3 tools/pico_bridge_monitor.py
    python3 tools/pico_bridge_monitor.py --port /dev/ttyACM0 --duration 30
"""

from __future__ import annotations

import argparse
import glob
import re
import sys
import time

try:
    import serial
except ImportError:
    print("Install pyserial: sudo apt install -y python3-serial", file=sys.stderr)
    sys.exit(1)

PING_RE = re.compile(
    r"VIGIA_PING seq=(?P<seq>\d+) uptime_ms=(?P<uptime>\d+) boot_ms=(?P<boot>\d+)"
)


def find_acm_port() -> str | None:
    matches = sorted(glob.glob("/dev/ttyACM*"))
    return matches[0] if matches else None


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor Pico 2 USB CDC heartbeat")
    parser.add_argument("--port", help="Serial device (default: first /dev/ttyACM*)")
    parser.add_argument("--baud", type=int, default=115200, help="Line speed (stdio USB default)")
    parser.add_argument("--duration", type=float, default=0.0, help="Stop after N seconds (0 = forever)")
    args = parser.parse_args()

    port = args.port or find_acm_port()
    if not port:
        print("No /dev/ttyACM* device found. Is Pico 2 plugged in and flashed?", file=sys.stderr)
        print("  ls -l /dev/ttyACM*   lsusb | grep -i raspberry", file=sys.stderr)
        return 1

    print(f"[monitor] Opening {port} @ {args.baud} baud")
    print("[monitor] Waiting for VIGIA_PING lines (Ctrl+C to stop)...")

    received = 0
    gaps = 0
    last_seq: int | None = None
    start = time.monotonic()

    try:
        with serial.Serial(port, args.baud, timeout=1.0) as ser:
            while True:
                if args.duration > 0 and (time.monotonic() - start) >= args.duration:
                    break

                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                m = PING_RE.match(line)
                if not m:
                    print(f"[raw] {line}")
                    continue

                seq = int(m.group("seq"))
                uptime_ms = int(m.group("uptime"))
                boot_ms = int(m.group("boot"))

                if last_seq is not None and seq != last_seq + 1:
                    gap = seq - last_seq - 1
                    gaps += gap
                    print(f"[WARN] seq gap: expected {last_seq + 1}, got {seq} (missed {gap})")

                last_seq = seq
                received += 1
                elapsed = time.monotonic() - start
                rate = received / elapsed if elapsed > 0 else 0.0
                print(f"[ok] seq={seq} uptime_ms={uptime_ms} boot_ms={boot_ms}  "
                      f"({received} msgs, {rate:.2f} Hz, {gaps} missed)")

    except serial.SerialException as exc:
        print(f"[error] Serial: {exc}", file=sys.stderr)
        print("Try: sudo usermod -aG dialout $USER  (then log out/in)", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print()

    elapsed = time.monotonic() - start
    print(f"\n[summary] {received} messages in {elapsed:.1f}s "
          f"({(received / elapsed) if elapsed else 0:.2f} Hz avg), {gaps} seq gaps")
    if received >= 5 and gaps == 0:
        print("[PASS] Pi ↔ Pico 2 link looks healthy.")
        return 0
    if received == 0:
        print("[FAIL] No VIGIA_PING received. Reflash vigia_pico_hello.uf2?")
        return 1
    print("[WARN] Link up but check for gaps or low rate.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
pico_gps_monitor.py — verify Pico 2 NEO-M8N GPS output over USB CDC.

Reads VIGIA_GPS lines from /dev/ttyACM0 and reports fix quality.

Usage (on the Pi):
    python3 tools/pico_gps_monitor.py
    python3 tools/pico_gps_monitor.py --port /dev/ttyACM0 --duration 60
"""

from __future__ import annotations

import argparse
import glob
import json
import re
import sys
import time

try:
    import serial
except ImportError:
    print("Install pyserial: sudo apt install -y python3-serial", file=sys.stderr)
    sys.exit(1)

GPS_RE = re.compile(
    r"VIGIA_GPS seq=(?P<seq>\d+) "
    r"(?:timestamp_us=(?P<ts>\d+) )?"
    r"lat=(?P<lat>[-\d.]+) lon=(?P<lon>[-\d.]+) "
    r"speed_ms=(?P<speed>[-\d.]+) fix_type=(?P<fix>\d+) "
    r"satellites=(?P<sats>\d+) hdop=(?P<hdop>[-\d.]+) valid=(?P<valid>[01])"
    r"(?: src=\w+)?"
)

PING_RE = re.compile(
    r"VIGIA_PING seq=(?P<seq>\d+) uptime_ms=(?P<uptime>\d+) boot_ms=(?P<boot>\d+)"
    r"(?: fw=[^\s]+)? uart_rx=(?P<uart_rx>\d+) baud=(?P<baud>\d+)"
)


def find_acm_port() -> str | None:
    matches = sorted(glob.glob("/dev/ttyACM*"))
    return matches[0] if matches else None


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor Pico 2 GPS over USB CDC")
    parser.add_argument("--port", help="Serial device (default: first /dev/ttyACM*)")
    parser.add_argument("--baud", type=int, default=115200, help="USB CDC line speed")
    parser.add_argument("--duration", type=float, default=0.0, help="Stop after N seconds (0 = forever)")
    parser.add_argument("--json", action="store_true", help="Print parsed fixes as JSON lines")
    args = parser.parse_args()

    port = args.port or find_acm_port()
    if not port:
        print("No /dev/ttyACM* device found. Is Pico 2 plugged in and flashed?", file=sys.stderr)
        return 1

    print(f"[gps] Opening {port} @ {args.baud} baud")
    print("[gps] Waiting for VIGIA_GPS (or VIGIA_PING if GPS not wired yet)...")

    gps_count = 0
    valid_count = 0
    ping_count = 0
    last_uart_rx = 0
    last_baud = 0
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

                m = GPS_RE.match(line)
                if m:
                    gps_count += 1
                    valid = m.group("valid") == "1"
                    if valid:
                        valid_count += 1

                    fix = {
                        "latitude": float(m.group("lat")),
                        "longitude": float(m.group("lon")),
                        "speed_ms": float(m.group("speed")),
                        "fix_type": int(m.group("fix")),
                        "satellites": int(m.group("sats")),
                        "hdop": float(m.group("hdop")),
                        "valid": valid,
                    }

                    if args.json:
                        print(json.dumps(fix))
                    else:
                        status = "FIX" if valid else "no-fix"
                        print(
                            f"[{status}] lat={fix['latitude']:.7f} lon={fix['longitude']:.7f} "
                            f"speed={fix['speed_ms']:.2f} m/s sats={fix['satellites']} "
                            f"hdop={fix['hdop']:.2f}"
                        )
                    continue

                m_ping = PING_RE.match(line)
                if m_ping:
                    ping_count += 1
                    last_uart_rx = int(m_ping.group("uart_rx"))
                    last_baud = int(m_ping.group("baud"))
                    if last_uart_rx > 0:
                        print(
                            f"[ping] uart_rx={last_uart_rx} baud={last_baud} "
                            f"(bytes flowing — waiting for VIGIA_GPS parse)"
                        )
                    else:
                        print(f"[ping] uart_rx=0 — wire M8N TX→GP9, RX←GP8, 3.3V, GND")
                    continue

                print(f"[raw] {line}")

    except serial.SerialException as exc:
        print(f"[error] Serial: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print()

    elapsed = time.monotonic() - start
    print(f"\n[summary] {gps_count} GPS lines ({valid_count} valid), "
          f"{ping_count} pings in {elapsed:.1f}s")

    if gps_count >= 3 and valid_count > 0:
        print("[PASS] GPS link up with at least one valid fix.")
        return 0
    if gps_count >= 3:
        print("[WARN] GPS UART active but no valid fix yet (move antenna outdoors?).")
        return 0
    if ping_count > 0 and gps_count == 0:
        if last_uart_rx > 1000:
            print(
                "[WARN] Row 2: UART bytes flowing but no VIGIA_GPS parsed — "
                "likely baud mismatch. Reflash latest firmware (parse-based autobaud) "
                "or power-cycle Pico with GPS wired before USB plug-in."
            )
        else:
            print("[WARN] Pi ↔ Pico OK; no GPS UART bytes — check M8N wiring on GP8/GP9.")
        return 1
    if gps_count == 0 and ping_count == 0:
        print("[FAIL] No output. Reflash vigia_pico_hello.uf2?")
        return 1
    print("[WARN] Insufficient samples.")
    return 1


if __name__ == "__main__":
    sys.exit(main())

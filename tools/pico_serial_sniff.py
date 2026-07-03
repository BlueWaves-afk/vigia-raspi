#!/usr/bin/env python3
"""Raw USB serial sniffer — shows any bytes from the Pico (text or binary)."""

from __future__ import annotations

import argparse
import sys
import time


def sniff(port: str, baud: int, duration: float) -> int:
    import serial

    # Plain open — same as the working pico_packet_monitor / pico_bridge_monitor.
    # Do NOT set dtr=False; macOS Pico CDC needs DTR asserted to send data.
    ser = serial.Serial(port, baudrate=baud, timeout=0.2)
    ser.reset_input_buffer()

    print(f"Sniffing {port} @ {baud} for {duration:.0f}s")
    print("Press RESET on the Pico now, or just wait for output...\n")

    total = 0
    text_lines: list[str] = []
    start = time.monotonic()

    try:
        while time.monotonic() - start < duration:
            chunk = ser.read(512)
            if not chunk:
                continue
            total += len(chunk)
            try:
                decoded = chunk.decode("utf-8", errors="replace")
                for line in decoded.splitlines():
                    if line.strip():
                        text_lines.append(line.strip())
                        print(line.rstrip())
            except Exception:
                print(f"[binary {len(chunk)} bytes] {chunk[:32].hex()}...")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    print(f"\nTotal bytes: {total}")
    if any("VIGIA_PROVISION" in l for l in text_lines):
        print("→ Provision firmware detected")
    elif any("VIGIA_IMU" in l or "VIGIA_PING" in l or "VIGIA_GPS" in l for l in text_lines):
        print("→ Hello (Phase 1) firmware detected")
    elif total > 50 and not text_lines:
        print("→ Binary COBS (Phase 2 live firmware)")
    elif total == 0:
        print("→ No data — wrong port, Pico not running, or port held by another app")
        print("  Try: lsof 2>/dev/null | grep usbmodem")
    return 0 if total > 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=15.0)
    args = parser.parse_args()
    return sniff(args.port, args.baud, args.duration)


if __name__ == "__main__":
    sys.exit(main())

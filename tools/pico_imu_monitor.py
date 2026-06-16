#!/usr/bin/env python3
"""
pico_imu_monitor.py — verify BNO085 VIGIA_IMU output over USB CDC.

Reads VIGIA_IMU lines and reports rate, sequence gaps, and quaternion norm.

Usage (Pi or Mac):
    python3 tools/pico_imu_monitor.py
    python3 tools/pico_imu_monitor.py --port /dev/cu.usbmodem11201 --duration 10
"""

from __future__ import annotations

import argparse
import glob
import re
import statistics
import sys
import time

try:
    import serial
except ImportError:
    print("Install pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)

IMU_RE = re.compile(
    r"VIGIA_IMU seq=(?P<seq>\d+) "
    r"(?:timestamp_us=(?P<ts>\d+) )?"
    r"qw=(?P<qw>[-\d.]+) qx=(?P<qx>[-\d.]+) qy=(?P<qy>[-\d.]+) qz=(?P<qz>[-\d.]+) "
    r"ax=(?P<ax>[-\d.]+) ay=(?P<ay>[-\d.]+) az=(?P<az>[-\d.]+) "
    r"cal=(?P<cal>\d+) valid=(?P<valid>[01]) "
    r"(?:qnorm=(?P<qnorm>[-\d.]+))?"
)

PING_SAMPLES_RE = re.compile(r"imu_samples=(?P<samples>\d+)")


def find_serial_port() -> str | None:
    for pattern in ("/dev/ttyACM*", "/dev/cu.usbmodem*", "/dev/tty.usbmodem*"):
        matches = sorted(glob.glob(pattern))
        if matches:
            # Prefer cu.* on macOS for non-blocking read/write.
            for m in matches:
                if m.startswith("/dev/cu."):
                    return m
            return matches[0]
    return None


def quat_norm(qw: float, qx: float, qy: float, qz: float) -> float:
    return (qw * qw + qx * qx + qy * qy + qz * qz) ** 0.5


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor Pico 2 IMU over USB CDC")
    parser.add_argument("--port", help="Serial device (auto-detect if omitted)")
    parser.add_argument("--baud", type=int, default=115200, help="Line speed (USB CDC)")
    parser.add_argument("--duration", type=float, default=10.0, help="Measure window in seconds")
    parser.add_argument(
        "--target-hz",
        type=float,
        default=95.0,
        help="Pass threshold for mean IMU rate (default 95)",
    )
    args = parser.parse_args()

    port = args.port or find_serial_port()
    if not port:
        print("No serial device found. Is Pico 2 plugged in and flashed?", file=sys.stderr)
        return 1

    print(f"[imu] Opening {port} @ {args.baud} baud")
    print(f"[imu] Collecting VIGIA_IMU for {args.duration:.0f}s...")

    imu_count = 0
    gaps = 0
    last_seq: int | None = None
    norms: list[float] = []
    chip_delta = 0
    last_chip_samples: int | None = None
    start = time.monotonic()

    try:
        with serial.Serial(port, args.baud, timeout=0.5) as ser:
            ser.reset_input_buffer()
            while True:
                elapsed = time.monotonic() - start
                if elapsed >= args.duration:
                    break

                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                m_ping = PING_SAMPLES_RE.search(line)
                if m_ping:
                    samples = int(m_ping.group("samples"))
                    if last_chip_samples is not None:
                        chip_delta += samples - last_chip_samples
                    last_chip_samples = samples

                m = IMU_RE.match(line)
                if not m:
                    continue

                imu_count += 1
                seq = int(m.group("seq"))
                if last_seq is not None and seq != last_seq + 1:
                    gaps += 1
                last_seq = seq

                qw = float(m.group("qw"))
                qx = float(m.group("qx"))
                qy = float(m.group("qy"))
                qz = float(m.group("qz"))
                qnorm_field = m.group("qnorm")
                norms.append(float(qnorm_field) if qnorm_field else quat_norm(qw, qx, qy, qz))

    except serial.SerialException as exc:
        print(f"[imu] Serial error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        pass

    elapsed = max(time.monotonic() - start, 1e-6)
    rate = imu_count / elapsed
    mean_norm = statistics.mean(norms) if norms else 0.0
    norm_ok = 0.98 <= mean_norm <= 1.02 if norms else False

    print()
    print(f"[imu] VIGIA_IMU messages : {imu_count}")
    print(f"[imu] Mean rate          : {rate:.1f} Hz (target >= {args.target_hz:.0f} Hz)")
    print(f"[imu] Sequence gaps      : {gaps}")
    print(f"[imu] Mean quaternion norm: {mean_norm:.4f} (expect 0.98–1.02)")
    if chip_delta:
        print(f"[imu] Chip sample delta  : {chip_delta} ({chip_delta / elapsed:.1f} Hz est.)")

    passed = rate >= args.target_hz and gaps == 0 and norm_ok
    if passed:
        print("[PASS] IMU wire rate and quality look good.")
    else:
        print("[FAIL] IMU wire rate or quality below threshold.")
        if rate < args.target_hz:
            print(f"       Rate {rate:.1f} Hz < {args.target_hz:.0f} Hz — reflash firmware with 100 Hz emit.")
        if gaps:
            print(f"       {gaps} sequence gap(s) — check USB cable/backpressure.")
        if norms and not norm_ok:
            print(f"       Quaternion norm {mean_norm:.4f} out of range.")

    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())

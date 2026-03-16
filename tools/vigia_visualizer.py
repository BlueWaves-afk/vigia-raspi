#!/usr/bin/env python3
"""
vigia_visualizer.py — Mac-side MJPEG stream receiver.

Receives JPEG-encoded annotated frames from Pi via UDP port 5005.
Displays them directly — no coordinate scaling needed.

Usage:
    python3 tools/vigia_visualizer.py --port 5005
"""

import socket
import numpy as np
import cv2
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5005)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(5.0)
    print(f"[UDP] Waiting for Pi stream on port {args.port}...")

    cv2.namedWindow("VIGIA Stream", cv2.WINDOW_NORMAL)

    while True:
        try:
            data, _ = sock.recvfrom(65536)
        except socket.timeout:
            print("[UDP] No data — waiting...")
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        # Decode JPEG directly from UDP payload
        buf = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            continue

        cv2.imshow("VIGIA Stream", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    sock.close()


if __name__ == "__main__":
    main()

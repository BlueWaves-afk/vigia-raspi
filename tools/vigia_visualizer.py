#!/usr/bin/env python3
"""
vigia_visualizer.py — Mac-side UDP telemetry receiver + bounding box overlay.

Receives detection JSON from Pi via UDP port 5005.
Overlays bounding boxes onto a local video file in sync with Pi frame index.

Usage:
    python3 vigia_visualizer.py hazard.mp4
    python3 vigia_visualizer.py hazard.mp4 --port 5005
"""

import socket
import json
import cv2
import sys
import argparse
import threading
import time
from collections import deque

# ── Latest detections from UDP thread ──────────────────────────────
_lock = threading.Lock()
_latest = {"f": 0, "fps": 0.0, "dets": []}


def udp_receiver(port: int):
    """Background thread — receives UDP packets, updates _latest."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.5)
    print(f"[UDP] Listening on 0.0.0.0:{port}")
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            payload = json.loads(data.decode())
            with _lock:
                _latest.update(payload)
        except socket.timeout:
            continue
        except Exception:
            continue


def draw_detections(frame, dets, fps: float, latency_ms: float):
    char_buf = f"FPS:{fps:.1f}  Det:{len(dets)}"
    cv2.putText(frame, char_buf, (8, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_8)

    for d in dets:
        x1, y1, x2, y2 = d["x1"], d["y1"], d["x2"], d["y2"]
        conf = d.get("s", 0.0)
        hazard = conf >= 0.55
        color = (0, 0, 255) if hazard else (0, 200, 80)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{'HAZARD' if hazard else 'SAFE'} {conf:.2f}"
        cv2.putText(frame, label, (x1, max(y1 - 6, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_8)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("video", help="Local video file (e.g. hazard.mp4)")
    parser.add_argument("--port", type=int, default=5005)
    args = parser.parse_args()

    # Start UDP receiver thread
    t = threading.Thread(target=udp_receiver, args=(args.port,), daemon=True)
    t.start()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open {args.video}")
        sys.exit(1)

    fps_video = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frame_delay = int(1000 / fps_video)

    cv2.namedWindow("VIGIA Telemetry", cv2.WINDOW_NORMAL)
    print("[INFO] Press Q to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # loop
            continue

        with _lock:
            dets = _latest.get("dets", [])
            fps = _latest.get("fps", 0.0)

        draw_detections(frame, dets, fps, 0.0)
        cv2.imshow("VIGIA Telemetry", frame)

        if cv2.waitKey(frame_delay) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

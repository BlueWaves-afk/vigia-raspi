#!/usr/bin/env python3
"""
vigia_visualizer.py — Mac-side UDP telemetry receiver + synchronized overlay.

Advances the local video one frame per UDP packet received from Pi.
This keeps Mac display in sync with Pi inference cadence.

Usage:
    python3 tools/vigia_visualizer.py hazard.mp4
    python3 tools/vigia_visualizer.py hazard.mp4 --port 5005
"""

import socket
import json
import cv2
import sys
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("video", help="Local video file (e.g. hazard.mp4)")
    parser.add_argument("--port", type=int, default=5005)
    args = parser.parse_args()

    # UDP socket — blocks until Pi sends a packet, then advances one frame
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(5.0)
    print(f"[UDP] Waiting for Pi on port {args.port}...")

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open {args.video}")
        sys.exit(1)

    cv2.namedWindow("VIGIA Telemetry", cv2.WINDOW_NORMAL)
    print("[INFO] Press Q to quit")

    while True:
        # Block until Pi sends a detection packet
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            print("[UDP] No data from Pi — waiting...")
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        try:
            payload = json.loads(data.decode())
        except Exception:
            continue

        # Advance video one frame per packet
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = cap.read()
            if not ret:
                break

        dets = payload.get("dets", [])
        fps  = payload.get("fps", 0.0)

        # Draw HUD
        cv2.putText(frame, f"Pi FPS:{fps:.1f}  Det:{len(dets)}",
                    (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_8)

        # Draw bounding boxes
        for d in dets:
            x1, y1, x2, y2 = d["x1"], d["y1"], d["x2"], d["y2"]
            conf = d.get("s", 0.0)
            hazard = conf >= 0.55
            color = (0, 0, 255) if hazard else (0, 200, 80)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{'HAZARD' if hazard else 'SAFE'} {conf:.2f}",
                        (x1, max(y1 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_8)

        cv2.imshow("VIGIA Telemetry", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    sock.close()


if __name__ == "__main__":
    main()

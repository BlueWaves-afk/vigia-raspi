#!/usr/bin/env python3
"""
01_export_yolov26.py — Export YOLOv26 Nano to ONNX (FP32, opset 17).

Run on a development machine with Ultralytics installed:
    pip install ultralytics>=8.3

Output: yolov26n.onnx (float32, NCHW, input "images" [1,3,320,320], output "output0")

Next step: 02_inspect_latent_layer.py
"""

import argparse
import sys
from pathlib import Path

try:
    from ultralytics import YOLO
except ImportError:
    sys.exit("ERROR: ultralytics not installed. Run: pip install ultralytics>=8.3")


def main() -> None:
    parser = argparse.ArgumentParser(description="Export YOLOv26n to ONNX FP32")
    parser.add_argument("--weights", default="yolov26n.pt",
                        help="Path to YOLOv26n .pt weights (default: yolov26n.pt)")
    parser.add_argument("--imgsz", type=int, default=320,
                        help="Model input size in pixels (default: 320)")
    parser.add_argument("--output-dir", default=".", type=Path,
                        help="Directory to write yolov26n.onnx (default: .)")
    args = parser.parse_args()

    weights = Path(args.weights)
    if not weights.exists():
        sys.exit(f"ERROR: weights file not found: {weights}")

    args.output_dir.mkdir(parents=True, exist_ok=True)

    print(f"[1/3] Loading {weights} ...")
    model = YOLO(str(weights))

    print(f"[2/3] Exporting to ONNX (opset 17, imgsz={args.imgsz}) ...")
    # simplify=True: onnx-simplifier removes redundant Reshape/Gather nodes
    # dynamic=False: fixed batch=1, prevents shape inference failures on Pi
    export_path = model.export(
        format="onnx",
        imgsz=args.imgsz,
        opset=17,
        simplify=True,
        dynamic=False,
    )

    dest = args.output_dir / Path(export_path).name
    if Path(export_path) != dest:
        import shutil
        shutil.move(export_path, dest)

    print(f"[3/3] Exported → {dest}")
    print()
    print("Next: python3 02_inspect_latent_layer.py --model", dest)


if __name__ == "__main__":
    main()

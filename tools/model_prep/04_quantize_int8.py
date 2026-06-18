#!/usr/bin/env python3
"""
04_quantize_int8.py — INT8 static quantization for KleidiAI UDOT dispatch.

Uses ORT's quantize_static with QDQ format + QUInt8 activations. This matches
the ACL EP's NEGEMMConvolution2d UDOT path on Cortex-A76 (Pi 5).

Why QUInt8 (unsigned) not QInt8?
  ARM KleidiAI UDOT micro-kernels (arm_gemm::GemmHybridQuantized) yield higher
  throughput for unsigned operands on A76. QDQ format allows ACL to fuse the
  Q/DQ pairs into native INT8 GEMM kernels rather than emulating them in FP32.

Calibration data:
  200–500 road images covering diverse lighting, speeds, and hazard types.
  Place them in data/calib_road/ (JPG or PNG).
  The VIGIA rdd2022/indian_roads/urban_issues dataset is a good calibration set.

Run:
    python3 04_quantize_int8.py \\
        --model yolov26n_with_latent.onnx \\
        --calib-dir data/calib_road/ \\
        --output yolov26_nano_int8_with_latent.onnx

Output goes to models/yolov26/ (or --output path).
"""

import argparse
import os
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit("ERROR: opencv-python + numpy required. Run: pip install opencv-python numpy")

try:
    from onnxruntime.quantization import (
        CalibrationDataReader,
        QuantFormat,
        QuantType,
        quantize_static,
    )
except ImportError:
    sys.exit("ERROR: onnxruntime not installed. Run: pip install onnxruntime>=1.18")


class RoadCalibrationReader(CalibrationDataReader):
    """Letterbox-resize road images to 320×320 and yield NCHW float32 tensors."""

    def __init__(self, calib_dir: str, input_size: int = 320):
        self.paths = sorted(
            [str(p) for p in Path(calib_dir).iterdir()
             if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}]
        )
        if not self.paths:
            raise RuntimeError(f"No images found in {calib_dir}")
        print(f"[calib] {len(self.paths)} images in {calib_dir}")
        self.idx = 0
        self.input_size = input_size

    def get_next(self):
        if self.idx >= len(self.paths):
            return None
        img = cv2.imread(self.paths[self.idx])
        self.idx += 1
        if img is None:
            return self.get_next()  # skip unreadable files

        # Letterbox resize (preserves aspect ratio, pads with 114)
        h, w = img.shape[:2]
        s = self.input_size
        scale = min(s / w, s / h)
        nw, nh = int(w * scale), int(h * scale)
        resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((s, s, 3), 114, dtype=np.uint8)
        x0, y0 = (s - nw) // 2, (s - nh) // 2
        canvas[y0:y0 + nh, x0:x0 + nw] = resized

        # BGR→RGB, HWC→NCHW, uint8→float32 /255
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        arr = (rgb.astype(np.float32) / 255.0).transpose(2, 0, 1)[np.newaxis]
        return {"images": arr}


def main() -> None:
    parser = argparse.ArgumentParser(description="INT8 static quantization for YOLOv26n")
    parser.add_argument("--model", required=True,
                        help="Input: yolov26n_with_latent.onnx (FP32)")
    parser.add_argument("--calib-dir", required=True,
                        help="Directory of calibration images (JPG/PNG)")
    parser.add_argument("--output", default=None,
                        help="Output path (default: models/yolov26/vigia_v26n_int8_with_latent.onnx)")
    parser.add_argument("--imgsz", type=int, default=320,
                        help="Model input size (default: 320)")
    args = parser.parse_args()

    in_path = Path(args.model)
    if not in_path.exists():
        sys.exit(f"ERROR: model not found: {in_path}")

    out_path = Path(args.output) if args.output else \
        Path("models/yolov26/vigia_v26n_int8_with_latent.onnx")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    calib_reader = RoadCalibrationReader(args.calib_dir, args.imgsz)

    print(f"Quantizing {in_path} → {out_path}")
    print("Format: QDQ / QUInt8 / per-tensor / reduce_range=True")
    print("(KleidiAI UDOT path: arm_gemm::GemmHybridQuantized on A76)")
    print()

    quantize_static(
        model_input=str(in_path),
        model_output=str(out_path),
        calibration_data_reader=calib_reader,
        quant_format=QuantFormat.QDQ,
        # Unsigned INT8 — matches ACL NEGEMMConvolution2d UDOT dispatch
        activation_type=QuantType.QUInt8,
        weight_type=QuantType.QUInt8,
        # per-tensor: faster on KleidiAI than per-channel (fewer dequant nodes)
        per_channel=False,
        # 7-bit range [0,127]: better ACL compat on A76, slight accuracy trade-off
        reduce_range=True,
        extra_options={
            "ActivationSymmetric": False,  # asymmetric: better accuracy for ReLU
            "WeightSymmetric":     True,   # symmetric weights: standard practice
        },
    )

    size_mb = out_path.stat().st_size / 1_048_576
    print(f"\n[ok] Quantized model saved → {out_path}  ({size_mb:.1f} MB)")
    print()
    print("Deploy to Pi:")
    print(f"  rsync -avz {out_path} vigiasense@100.114.1.98:~/vigia_ws/")
    print()
    print("Update config/vision_params.yaml:")
    print(f"  model_path: \"{out_path}\"")
    print("  latent_layer_name: \"/model.22/cv2/act/Mul_output_0\"  # verify with 02_inspect_latent_layer.py")


if __name__ == "__main__":
    main()

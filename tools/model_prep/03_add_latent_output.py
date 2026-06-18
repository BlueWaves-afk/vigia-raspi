#!/usr/bin/env python3
"""
03_add_latent_output.py — Add the penultimate layer as a named ONNX graph output.

ONNX Runtime only returns tensors that appear in graph.output. The penultimate
layer already exists as an intermediate node — this script promotes it to an
output so ORT returns it from session.Run() alongside the detection head.

The output variant (_with_latent.onnx) is the input to 04_quantize_int8.py.

Run:
    python3 03_add_latent_output.py \\
        --model yolov26n.onnx \\
        --layer "/model.22/cv2/act/Mul_output_0" \\
        --output yolov26n_with_latent.onnx
"""

import argparse
import sys
from pathlib import Path

try:
    import onnx
    import onnx.helper
except ImportError:
    sys.exit("ERROR: onnx not installed. Run: pip install onnx>=1.14")


def main() -> None:
    parser = argparse.ArgumentParser(description="Add latent layer as ONNX graph output")
    parser.add_argument("--model", required=True, help="Input ONNX model (FP32)")
    parser.add_argument("--layer", required=True,
                        help="Exact ONNX tensor name to expose as latent output")
    parser.add_argument("--output", default=None,
                        help="Output path (default: <input>_with_latent.onnx)")
    args = parser.parse_args()

    in_path = Path(args.model)
    if not in_path.exists():
        sys.exit(f"ERROR: model not found: {in_path}")

    out_path = Path(args.output) if args.output else \
        in_path.parent / (in_path.stem + "_with_latent.onnx")

    print(f"Loading {in_path} ...")
    model = onnx.load(str(in_path))

    # Verify the tensor exists in the graph
    all_tensors = {o for node in model.graph.node for o in node.output}
    if args.layer not in all_tensors:
        print(f"ERROR: tensor '{args.layer}' not found in graph.")
        print("Run 02_inspect_latent_layer.py to list available tensors.")
        sys.exit(1)

    # Check if it's already an output
    existing_outputs = [o.name for o in model.graph.output]
    if args.layer in existing_outputs:
        print(f"[ok] Tensor '{args.layer}' is already a graph output — no change needed.")
        print(f"Saving (unchanged) → {out_path}")
        onnx.save(model, str(out_path))
        return

    # Add the tensor as a graph output.
    # shape=None lets ONNX infer it — avoids hardcoding dims which may vary.
    latent_value_info = onnx.helper.make_tensor_value_info(
        args.layer,
        onnx.TensorProto.FLOAT,
        shape=None,
    )
    model.graph.output.append(latent_value_info)

    print(f"[ok] Added latent output: {args.layer}")
    print("Checking model integrity ...")
    onnx.checker.check_model(model)
    print("[ok] Model check passed.")

    onnx.save(model, str(out_path))
    print(f"Saved → {out_path}")
    print()
    print("Outputs now registered:")
    for o in model.graph.output:
        print(f"  {o.name}")
    print()
    print("Next: python3 04_quantize_int8.py --model", out_path, "--calib-dir data/calib_road/")


if __name__ == "__main__":
    main()

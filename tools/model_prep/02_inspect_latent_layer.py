#!/usr/bin/env python3
"""
02_inspect_latent_layer.py — Print all intermediate tensor names in a YOLO ONNX model.

Use this after 01_export_yolov26.py to identify the correct penultimate layer
name for S_t (spatial latent) extraction. Cross-reference with Netron:
    npx netron yolov26n.onnx

The layer immediately before the detection head (/model.22/...) is typically
the output of the last C2f block. It has shape [1, C, H, W] where C=256 and
H=W=20 for a 320×320 input.

Run:
    python3 02_inspect_latent_layer.py --model yolov26n.onnx [--filter model.22]

Output: a list of candidate tensor names. Record the correct one and set it as
    latent_layer_name in config/vision_params.yaml.

Next step: 03_add_latent_output.py --model yolov26n.onnx --layer <chosen_name>
"""

import argparse
import sys

try:
    import onnx
except ImportError:
    sys.exit("ERROR: onnx not installed. Run: pip install onnx>=1.14")


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect ONNX intermediate tensor names")
    parser.add_argument("--model", required=True, help="Path to yolov26n.onnx")
    parser.add_argument("--filter", default="",
                        help="Only show tensors whose name contains this string (e.g. 'model.22')")
    parser.add_argument("--shape", action="store_true",
                        help="Also print inferred output shape (requires onnx shape inference)")
    args = parser.parse_args()

    print(f"Loading {args.model} ...")
    model = onnx.load(args.model)

    if args.shape:
        try:
            from onnx import shape_inference
            model = shape_inference.infer_shapes(model)
            print("Shape inference complete.")
        except Exception as e:
            print(f"Warning: shape inference failed ({e}) — shapes won't be shown.")
            args.shape = False

    # Build shape map from value_info (populated after shape inference)
    shape_map: dict = {}
    if args.shape:
        for vi in model.graph.value_info:
            try:
                dims = [d.dim_value for d in vi.type.tensor_type.shape.dim]
                shape_map[vi.name] = dims
            except Exception:
                pass

    filter_note = f" matching '{args.filter}'" if args.filter else ""
    print(f"\nAll intermediate tensors{filter_note}:")
    print("-" * 72)
    count = 0
    for node in model.graph.node:
        for output in node.output:
            if args.filter and args.filter not in output:
                continue
            shape_str = ""
            if args.shape and output in shape_map:
                shape_str = f"  shape={shape_map[output]}"
            print(f"  op={node.op_type:<16}  tensor={output}{shape_str}")
            count += 1

    print("-" * 72)
    print(f"{count} tensors found.")
    print()
    print("Likely penultimate layer candidates (look for C=256, H=W=20 for 320px input):")
    for node in model.graph.node:
        for output in node.output:
            if "/model.22" in output and "act" in output.lower():
                shape_str = ""
                if args.shape and output in shape_map:
                    shape_str = f"  shape={shape_map[output]}"
                print(f"  *** {output}{shape_str}")


if __name__ == "__main__":
    main()

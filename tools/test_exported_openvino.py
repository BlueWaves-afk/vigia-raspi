import cv2
import time
import numpy as np
try:
    import openvino.runtime as ov
except ImportError:
    import openvino as ov
from pathlib import Path

# --- CONFIGURATION ---
MODEL_XML = "models_320_fp16/yolo26_320_fp16.xml"
VIDEO_SOURCE = "hazard.mp4" # Ensure this file is in the same directory
CONF_THRESHOLD = 0.15       # Adjusted lower for INT8 score shifts
IMAGE_SIZE = 320            # YOLO26 input size

def preprocess(frame):
    """Prepares a single frame for the OpenVINO model."""
    resized = cv2.resize(frame, (IMAGE_SIZE, IMAGE_SIZE))
    # Convert BGR (OpenCV) to RGB
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    # Normalize and Transpose (HWC -> CHW)
    blob = rgb.astype(np.float32) / 255.0
    blob = blob.transpose(2, 0, 1)
    return np.expand_dims(blob, 0)

def main():
    # 1. Initialize OpenVINO
    if not hasattr(ov, "Core"):
        raise ImportError(
            "OpenVINO Runtime is not available in this environment. "
            "Install/upgrade with: pip install -U openvino"
        )

    core = ov.Core()
    # Explicitly using CPU for Mac M2
    model = core.read_model(MODEL_XML)
    compiled_model = core.compile_model(model, "CPU")
    output_layer = compiled_model.output(0)

    # 2. Setup Video
    cap = cv2.VideoCapture(VIDEO_SOURCE)
    if not cap.isOpened():
        print(f"Error: Could not open video {VIDEO_SOURCE}")
        return

    print(f"Starting inference on {VIDEO_SOURCE}...")
    print("Press 'q' to exit.")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        start_time = time.time()

        # 3. Preprocess and Run Inference
        input_data = preprocess(frame)
        results = compiled_model([input_data])[output_layer]

        # 4. Parse model output
        # Supports common layouts:
        # - [1, N, 6] => (x1, y1, x2, y2, score, class)
        # - [1, 5, N] or [1, N, 5] => (cx, cy, w, h, score)
        detections = results[0]
        if detections.ndim == 2 and detections.shape[0] in (5, 6) and detections.shape[1] > detections.shape[0]:
            detections = detections.T
        
        h, w = frame.shape[:2]

        for det in detections:
            if det.shape[0] >= 6:
                x1, y1, x2, y2, score, cls = det[:6]
            elif det.shape[0] == 5:
                cx, cy, bw, bh, score = det
                x1 = cx - (bw / 2.0)
                y1 = cy - (bh / 2.0)
                x2 = cx + (bw / 2.0)
                y2 = cy + (bh / 2.0)
                cls = 0
            else:
                continue

            if score > CONF_THRESHOLD:
                # Rescale coordinates to original frame size
                ix1 = int(max(0, min(w - 1, x1 * w / IMAGE_SIZE)))
                iy1 = int(max(0, min(h - 1, y1 * h / IMAGE_SIZE)))
                ix2 = int(max(0, min(w - 1, x2 * w / IMAGE_SIZE)))
                iy2 = int(max(0, min(h - 1, y2 * h / IMAGE_SIZE)))

                if ix2 <= ix1 or iy2 <= iy1:
                    continue

                # Draw Detection Box
                cv2.rectangle(frame, (ix1, iy1), (ix2, iy2), (0, 255, 0), 2)
                label = f"Pothole: {score:.2f}"
                cv2.putText(frame, label, (ix1, iy1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 5. Calculate & Display FPS
        latency = (time.time() - start_time) * 1000
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(frame, f"Latency: {latency:.1f}ms | FPS: {fps:.1f}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        cv2.imshow("VIGIA-ARM: Road Hazard Perception (INT8)", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
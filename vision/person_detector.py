"""
Lightweight ONNX Runtime Person Detector for ESP32-CAM MJPEG Stream.

- Reads frames from ESP32-CAM (http://<ip>:81/stream) or local webcam ("0").
- Runs YOLOv8n ONNX (COCO class 0 = person) on CPU via onnxruntime.
- Publishes JSON alerts to MQTT when a person is detected (with cooldown).
- Serves an annotated MJPEG stream at http://<host>:<port>/stream for Node-RED.
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import onnxruntime as ort
import paho.mqtt.client as mqtt
from dotenv import load_dotenv

load_dotenv()

# ============================================================================
# Configuration
# ============================================================================
CAM_STREAM_URL     = os.getenv("CAM_STREAM_URL", "http://esp32cam.local:81/stream")
MQTT_ENABLED       = os.getenv("MQTT_ENABLED", "true").lower() == "true"
MQTT_BROKER        = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT          = int(os.getenv("MQTT_PORT", "1883"))
MQTT_TOPIC         = os.getenv("MQTT_TOPIC", "rc/alert/person")
MODEL_PATH         = os.getenv("MODEL_PATH", "yolov8n.onnx")
CONF_THRESHOLD     = float(os.getenv("CONF_THRESHOLD", "0.45"))
ALERT_COOLDOWN_SEC = float(os.getenv("ALERT_COOLDOWN_SEC", "5.0"))
HTTP_HOST          = os.getenv("HTTP_HOST", "0.0.0.0")
HTTP_PORT          = int(os.getenv("HTTP_PORT", "5000"))

INPUT_SIZE = 640
NMS_IOU_THRESHOLD = 0.45
PERSON_CLASS_ID = 0

# Shared latest JPEG frame for the HTTP MJPEG server
latest_jpeg_lock = threading.Lock()
latest_jpeg: bytes | None = None


# ============================================================================
# YOLOv8 ONNX Inference Helpers
# ============================================================================
def letterbox(img: np.ndarray, new_size: int = INPUT_SIZE):
    """Resize image with unchanged aspect ratio using padding."""
    h, w = img.shape[:2]
    scale = min(new_size / h, new_size / w)
    nh, nw = int(round(h * scale)), int(round(w * scale))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)

    pad_w = (new_size - nw) // 2
    pad_h = (new_size - nh) // 2
    padded = np.full((new_size, new_size, 3), 114, dtype=np.uint8)
    padded[pad_h : pad_h + nh, pad_w : pad_w + nw] = resized
    return padded, scale, pad_w, pad_h


class PersonDetector:
    def __init__(self, model_path: str):
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f"Model not found at '{model_path}'. "
                "See vision/README.md for how to download yolov8n.onnx."
            )
        self.session = ort.InferenceSession(
            model_path, providers=["CPUExecutionProvider"]
        )
        self.input_name = self.session.get_inputs()[0].name
        print(f"[AI] Loaded {model_path} on CPUExecutionProvider")

    def detect(self, frame: np.ndarray):
        """Return list of (x1, y1, x2, y2, confidence) for detected persons."""
        img, scale, pad_w, pad_h = letterbox(frame, INPUT_SIZE)
        blob = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))[np.newaxis, ...]

        outputs = self.session.run(None, {self.input_name: blob})[0]
        # YOLOv8 output shape: (1, 84, 8400) -> transpose to (8400, 84)
        preds = np.squeeze(outputs, axis=0).T

        # Person class score is at index 4 + PERSON_CLASS_ID (index 4)
        person_scores = preds[:, 4 + PERSON_CLASS_ID]
        mask = person_scores >= CONF_THRESHOLD
        if not np.any(mask):
            return []

        filtered = preds[mask]
        scores = person_scores[mask]

        cx, cy, bw, bh = filtered[:, 0], filtered[:, 1], filtered[:, 2], filtered[:, 3]
        x1 = (cx - bw / 2 - pad_w) / scale
        y1 = (cy - bh / 2 - pad_h) / scale
        w_box = bw / scale
        h_box = bh / scale

        boxes_xywh = np.stack([x1, y1, w_box, h_box], axis=1).tolist()
        indices = cv2.dnn.NMSBoxes(
            boxes_xywh, scores.tolist(), CONF_THRESHOLD, NMS_IOU_THRESHOLD
        )

        results = []
        if len(indices) > 0:
            h_img, w_img = frame.shape[:2]
            for i in np.array(indices).flatten():
                bx, by, bw_i, bh_i = boxes_xywh[i]
                rx1 = max(0, int(bx))
                ry1 = max(0, int(by))
                rx2 = min(w_img, int(bx + bw_i))
                ry2 = min(h_img, int(by + bh_i))
                results.append((rx1, ry1, rx2, ry2, float(scores[i])))
        return results


# ============================================================================
# MQTT Publisher
# ============================================================================
def create_mqtt_client():
    if not MQTT_ENABLED:
        return None
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        client.loop_start()
        print(f"[MQTT] Connected to {MQTT_BROKER}:{MQTT_PORT} (topic: {MQTT_TOPIC})")
        return client
    except Exception as exc:
        print(f"[MQTT] Could not connect to {MQTT_BROKER}:{MQTT_PORT} ({exc}). Continuing without MQTT.")
        return None


# ============================================================================
# Annotated MJPEG HTTP Server (for Node-RED)
# ============================================================================
class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/stream":
            self.send_response(404)
            self.end_headers()
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        try:
            while True:
                with latest_jpeg_lock:
                    frame_bytes = latest_jpeg
                if frame_bytes is not None:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame_bytes)}\r\n\r\n".encode())
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b"\r\n")
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass

    def log_message(self, format, *args):
        return  # Keep console clean


def start_http_server():
    server = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), StreamHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"[HTTP] Annotated MJPEG stream available at http://localhost:{HTTP_PORT}/stream")


# ============================================================================
# Main Loop
# ============================================================================
def main():
    global latest_jpeg

    detector = PersonDetector(MODEL_PATH)
    mqtt_client = create_mqtt_client()
    start_http_server()

    source = 0 if CAM_STREAM_URL == "0" else CAM_STREAM_URL
    print(f"[CAM] Opening video source: {source}")

    last_alert_time = 0.0

    while True:
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            print("[CAM] Failed to open stream. Retrying in 3s...")
            time.sleep(3)
            continue

        print("[CAM] Stream connected.")
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[CAM] Stream lost. Reconnecting...")
                break

            detections = detector.detect(frame)

            for x1, y1, x2, y2, conf in detections:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(
                    frame,
                    f"PERSON {conf:.2f}",
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                )

            # Publish MQTT alert with cooldown
            now = time.time()
            if detections and (now - last_alert_time >= ALERT_COOLDOWN_SEC):
                last_alert_time = now
                payload = json.dumps(
                    {
                        "event": "person_detected",
                        "count": len(detections),
                        "max_confidence": round(max(d[4] for d in detections), 2),
                        "timestamp": int(now),
                    }
                )
                print(f"[ALERT] {payload}")
                if mqtt_client is not None:
                    mqtt_client.publish(MQTT_TOPIC, payload)

            ok_enc, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok_enc:
                with latest_jpeg_lock:
                    latest_jpeg = buf.tobytes()

        cap.release()
        time.sleep(2)


if __name__ == "__main__":
    main()

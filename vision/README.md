# Vision Person Detection (`vision`)

Lightweight CPU-based person detection service using **ONNX Runtime** + **YOLOv8n** (< 300 MB total disk usage, no PyTorch required).

## Setup
1. Install Python dependencies:
   ```bash
   cd vision
   pip install -r requirements.txt
   ```
2. Download `yolov8n.onnx` (~12 MB) into the `vision/` folder:
   - From Hugging Face / Ultralytics ONNX releases, or export once on a machine with `ultralytics`:
     ```bash
     yolo export model=yolov8n.pt format=onnx imgsz=640
     ```
3. Copy `.env.example` to `.env` and set `CAM_STREAM_URL` to your ESP32-CAM IP (e.g., `http://192.168.1.50:81/stream`) or `0` to test with your webcam.

## Run
```bash
python person_detector.py
```

- **Annotated MJPEG stream (for Node-RED):** `http://localhost:5000/stream`
- **MQTT alert topic:** `rc/alert/person` (JSON payload: `{"event":"person_detected","count":1,"max_confidence":0.87,"timestamp":...}`)

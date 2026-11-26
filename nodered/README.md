# Node-RED Surveillance & Control Flows (`nodered`)

Node-RED **Dashboard 2.0** (`@flowfuse/node-red-dashboard`) flows for the **RC Car Security Surveillance System**.

## Files
- [`flow.json`](flow.json) — **Full Unified Dashboard** (Main ESP32 RC Car control, D-Pad, mode switching, path recording/playback, NTP patrol scheduler, 3x ultrasonic telemetry gauges, ESP32-CAM live MJPEG stream, and PIR + YOLOv8n intruder alerts).
- [`vision_flow.json`](vision_flow.json) — Standalone Vision-only dashboard for testing the ESP32-CAM + YOLOv8n pipeline independently.

## Prerequisites
1. Install **Node-RED** and **`@flowfuse/node-red-dashboard`** (via Manage Palette → Install `@flowfuse/node-red-dashboard`, which matches Module 9 of the course).
2. MQTT Broker: Pre-configured for **`broker.hivemq.com:1883`** under the `iot_finpro_naputriz/rc/...` topic namespace (or update the MQTT broker node to a local Mosquitto instance).
3. *(Optional for AI person detection)* Run `python vision/person_detector.py` so the annotated stream is live at `http://localhost:5000/stream`.

## Import & Run
1. Open Node-RED (`http://localhost:1880`).
2. Click **Menu (☰) → Import → Select a file to import** and choose [`flow.json`](flow.json).
3. *(Optional)* If `esp32cam.local` does not resolve on your network, replace `esp32cam.local` with your ESP32-CAM's IP in the HTTP Request nodes and the `ESP32-CAM MJPEG Stream` template.
4. Click **Deploy**, then open the Dashboard 2.0 UI at **`http://localhost:1880/dashboard/rc-surveillance`**.

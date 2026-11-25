# Node-RED Vision Surveillance Flow (`nodered`)

Standalone Node-RED dashboard flow for the ESP32-CAM + YOLOv8n person-detection pipeline.

## Prerequisites
1. Install **Node-RED** and **`node-red-dashboard`** (via Manage Palette → Install `node-red-dashboard`).
2. Start an MQTT broker (e.g., local **Mosquitto** on `localhost:1883`, or update the `Local MQTT Broker` node to a cloud broker like `broker.hivemq.com`).
3. Start `python vision/person_detector.py` so the annotated stream is live at `http://localhost:5000/stream`.

## Import & Run
1. Open Node-RED (`http://localhost:1880`).
2. Click the **Menu (☰) → Import → Select a file to import** and choose `vision_flow.json`.
3. *(Optional)* If `esp32cam.local` does not resolve on your network, replace `esp32cam.local` with your ESP32-CAM's IP in the two HTTP Request nodes (`ESP32-CAM /flash` and `ESP32-CAM /status`).
4. Click **Deploy**, then open the dashboard at **`http://localhost:1880/ui`**.

## What It Shows
- **Live Camera Feed:** Embedded YOLOv8n-annotated MJPEG stream (`http://localhost:5000/stream`).
- **Intruder Alert Popup & Last Alert:** Listens to MQTT topic `rc/alert/person` and displays a red toast notification + timestamp.
- **Camera Flash LED Toggle:** Sends `GET /flash?state=on|off` to the ESP32-CAM.
- **Check ESP32-CAM Status:** Queries `GET /status` and shows IP + Wi-Fi RSSI.

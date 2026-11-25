# ESP32-CAM Vision Node (`firmware_cam`)

Standalone MJPEG HTTP streaming firmware for the **AI-Thinker ESP32-CAM (OV2640)**.

## Features
- **Dedicated MJPEG stream server** on port `81` (`/stream`) so live video never blocks control requests.
- **Control endpoints** on port `80`:
  - `GET /status` — JSON health check (`ip`, `rssi`, `psram`, `flash`, `uptime_ms`)
  - `GET /capture` — Single JPEG snapshot
  - `GET /flash?state=on|off` — Optional onboard flash LED toggle (**OFF by default** per spec)
- **mDNS support:** `http://esp32cam.local:81/stream`
- **Auto-reconnect:** Checks Wi-Fi every 10 seconds without blocking the HTTP server.

## Setup & Flash
1. Open [config.h](config.h) and set `WIFI_SSID` and `WIFI_PASSWORD`.
2. Connect your FTDI/USB-TTL adapter (5V recommended to avoid brownouts) and short **GPIO 0 → GND** to enter flashing mode.
3. Build & upload with PlatformIO:
   ```bash
   cd firmware_cam
   pio run -t upload
   ```
4. Disconnect **GPIO 0 from GND**, press **RST**, and open the Serial Monitor (`115200` baud) to see the assigned IP address.

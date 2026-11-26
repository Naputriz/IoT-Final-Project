#pragma once

// ============================================================================
// Wi-Fi Configuration (Copy secrets.h.example -> secrets.h for credentials)
// ============================================================================
#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID       "YOUR_WIFI_SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
#endif
#define MDNS_HOSTNAME   "esp32cam"   // Accessible as http://esp32cam.local

// ============================================================================
// HTTP Server Ports
// ============================================================================
#define HTTP_CTRL_PORT   80          // /status, /capture, /flash
#define HTTP_STREAM_PORT 81          // /stream (dedicated so it never blocks control)

// ============================================================================
// Camera Tuning (Balanced for smooth MJPEG + low power)
// ============================================================================
#define CAM_FRAME_SIZE_PSRAM    FRAMESIZE_VGA   // 640x480 when PSRAM is available
#define CAM_FRAME_SIZE_NO_PSRAM FRAMESIZE_QVGA  // 320x240 fallback
#define CAM_JPEG_QUALITY        12              // 10-63 (lower = higher quality)
#define CAM_STREAM_FPS_DELAY_MS 30              // ~20-30 FPS cap to keep Wi-Fi & power stable

// ============================================================================
// AI-Thinker ESP32-CAM (OV2640) Pinout
// ============================================================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Onboard High-Power Flash LED (Disabled by default per project spec)
#define FLASH_LED_PIN      4

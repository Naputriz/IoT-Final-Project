#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "config.h"

// ============================================================================
// Globals
// ============================================================================
static httpd_handle_t ctrlServer   = nullptr;
static httpd_handle_t streamServer = nullptr;
static bool flashEnabled           = false;
static unsigned long lastWifiCheck = 0;

static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char* STREAM_BOUNDARY     = "\r\n--frame\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ============================================================================
// Camera Initialization
// ============================================================================
static bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    if (psramFound()) {
        config.frame_size   = CAM_FRAME_SIZE_PSRAM;
        config.jpeg_quality = CAM_JPEG_QUALITY;
        config.fb_count     = 2;
        config.fb_location  = CAMERA_FB_IN_PSRAM;
    } else {
        config.frame_size   = CAM_FRAME_SIZE_NO_PSRAM;
        config.jpeg_quality = 15;
        config.fb_count     = 1;
        config.fb_location  = CAMERA_FB_IN_DRAM;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t* s = esp_camera_sensor_get();
    if (s != nullptr) {
        s->set_brightness(s, 1);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 1);                          // Night/low-light auto-exposure
        s->set_ae_level(s, 1);                      // Slight brightness boost (-2 to +2)
        s->set_gain_ctrl(s, 1);
        s->set_gainceiling(s, GAINCEILING_16X);     // Higher ISO gain for dim rooms
    }

    Serial.printf("[CAM] OV2640 initialized (PSRAM: %s)\n", psramFound() ? "YES" : "NO");
    return true;
}

// ============================================================================
// HTTP Handlers
// ============================================================================
static esp_err_t streamHandler(httpd_req_t* req) {
    camera_fb_t* fb = nullptr;
    esp_err_t res   = ESP_OK;
    char partBuf[64];

    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");

    Serial.println("[STREAM] Client connected");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[STREAM] Frame capture failed");
            res = ESP_FAIL;
            break;
        }

        size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, fb->len);

        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, partBuf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, reinterpret_cast<const char*>(fb->buf), fb->len);
        }

        esp_camera_fb_return(fb);
        fb = nullptr;

        if (res != ESP_OK) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(CAM_STREAM_FPS_DELAY_MS));
    }

    Serial.println("[STREAM] Client disconnected");
    return res;
}

static esp_err_t captureHandler(httpd_req_t* req) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res = httpd_resp_send(req, reinterpret_cast<const char*>(fb->buf), fb->len);
    esp_camera_fb_return(fb);
    return res;
}

static esp_err_t statusHandler(httpd_req_t* req) {
    char json[256];
    snprintf(
        json,
        sizeof(json),
        "{\"status\":\"ok\",\"ip\":\"%s\",\"rssi\":%d,\"psram\":%s,\"flash\":%s,\"uptime_ms\":%lu}",
        WiFi.localIP().toString().c_str(),
        WiFi.RSSI(),
        psramFound() ? "true" : "false",
        flashEnabled ? "true" : "false",
        millis()
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
}

static esp_err_t flashHandler(httpd_req_t* req) {
    char query[32] = {0};
    char state[8]  = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "state", state, sizeof(state)) == ESP_OK) {
        if (strcmp(state, "on") == 0) {
            flashEnabled = true;
            digitalWrite(FLASH_LED_PIN, HIGH);
        } else if (strcmp(state, "off") == 0) {
            flashEnabled = false;
            digitalWrite(FLASH_LED_PIN, LOW);
        }
    }

    char resp[64];
    snprintf(resp, sizeof(resp), "{\"flash\":%s}", flashEnabled ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, resp, strlen(resp));
}

// ============================================================================
// HTTP Server Startup
// ============================================================================
static void startServers() {
    // Control server (port 80): /status, /capture, /flash
    httpd_config_t ctrlCfg = HTTPD_DEFAULT_CONFIG();
    ctrlCfg.server_port    = HTTP_CTRL_PORT;
    ctrlCfg.ctrl_port      = 32768;

    if (httpd_start(&ctrlServer, &ctrlCfg) == ESP_OK) {
        httpd_uri_t statusUri  = { "/status",  HTTP_GET, statusHandler,  nullptr };
        httpd_uri_t captureUri = { "/capture", HTTP_GET, captureHandler, nullptr };
        httpd_uri_t flashUri   = { "/flash",   HTTP_GET, flashHandler,   nullptr };
        httpd_register_uri_handler(ctrlServer, &statusUri);
        httpd_register_uri_handler(ctrlServer, &captureUri);
        httpd_register_uri_handler(ctrlServer, &flashUri);
        Serial.printf("[HTTP] Control server started on port %d\n", HTTP_CTRL_PORT);
    }

    // Stream server (port 81): /stream
    httpd_config_t streamCfg = HTTPD_DEFAULT_CONFIG();
    streamCfg.server_port    = HTTP_STREAM_PORT;
    streamCfg.ctrl_port      = 32769;

    if (httpd_start(&streamServer, &streamCfg) == ESP_OK) {
        httpd_uri_t streamUri = { "/stream", HTTP_GET, streamHandler, nullptr };
        httpd_register_uri_handler(streamServer, &streamUri);
        Serial.printf("[HTTP] MJPEG stream server started on port %d\n", HTTP_STREAM_PORT);
    }
}

// ============================================================================
// Setup & Loop
// ============================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32-CAM Vision Node Booting ===");

    // Ensure flash LED is OFF by default
    pinMode(FLASH_LED_PIN, OUTPUT);
    digitalWrite(FLASH_LED_PIN, LOW);

    if (!initCamera()) {
        Serial.println("[FATAL] Camera init failed. Restarting in 5s...");
        delay(5000);
        ESP.restart();
    }

    // Connect to Wi-Fi
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // Disable Wi-Fi modem sleep for low-latency MJPEG streaming
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("[WIFI] Connecting");
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
        if (MDNS.begin(MDNS_HOSTNAME)) {
            Serial.printf("[MDNS] http://%s.local\n", MDNS_HOSTNAME);
        }
    } else {
        Serial.println("\n[WIFI] Initial connection timed out; will retry in background.");
    }

    startServers();

    Serial.println("[READY] Endpoints:");
    Serial.printf("  Stream : http://%s:%d/stream\n", WiFi.localIP().toString().c_str(), HTTP_STREAM_PORT);
    Serial.printf("  Capture: http://%s/capture\n",   WiFi.localIP().toString().c_str());
    Serial.printf("  Status : http://%s/status\n",    WiFi.localIP().toString().c_str());
    Serial.printf("  Flash  : http://%s/flash?state=on|off\n", WiFi.localIP().toString().c_str());
}

void loop() {
    // Non-blocking Wi-Fi reconnect monitor every 10 seconds
    if (millis() - lastWifiCheck >= 10000) {
        lastWifiCheck = millis();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[WIFI] Disconnected. Attempting reconnect...");
            WiFi.reconnect();
        }
    }
    delay(100);
}

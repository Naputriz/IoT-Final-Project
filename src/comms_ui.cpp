#include "comms_ui.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <esp_sleep.h>

static WiFiClient sWifiClient;
static PubSubClient sMqttClient(sWifiClient);

static uint32_t sLastWifiAttemptMs = 0;
static uint32_t sLastMqttAttemptMs = 0;
static uint32_t sLastNtpCheckMs    = 0;
static bool sNtpConfigured         = false;
static bool sPowerSaveActive       = false;
static volatile bool sLightSleepFlag = false;

// ============================================================================
// Command Helpers
// ============================================================================
static void enqueueCommand(CommandType type, uint8_t value = 0) {
    ControlCommand cmd{type, value};
    xQueueSend(xCmdQueue, &cmd, 0);
}

static bool parseScheduleString(const String& payload, PatrolSchedule& outSched) {
    if (payload.startsWith("{")) {
        JsonDocument doc;
        if (deserializeJson(doc, payload) != DeserializationError::Ok) {
            return false;
        }
        outSched.enabled = doc["enabled"] | true;
        outSched.loopPatrol = doc["loop"] | false;
        if (doc["time"].is<const char*>()) {
            String t = doc["time"].as<String>();
            int colon = t.indexOf(':');
            if (colon > 0) {
                outSched.hour = static_cast<uint8_t>(constrain(t.substring(0, colon).toInt(), 0, 23));
                outSched.minute = static_cast<uint8_t>(constrain(t.substring(colon + 1).toInt(), 0, 59));
            }
        } else {
            outSched.hour = static_cast<uint8_t>(constrain(doc["hour"] | 0, 0, 23));
            outSched.minute = static_cast<uint8_t>(constrain(doc["minute"] | 0, 0, 59));
        }
        outSched.lastTriggeredMinute = -1;
        return true;
    }

    int colon = payload.indexOf(':');
    if (colon > 0) {
        outSched.enabled = true;
        outSched.hour = static_cast<uint8_t>(constrain(payload.substring(0, colon).toInt(), 0, 23));
        outSched.minute = static_cast<uint8_t>(constrain(payload.substring(colon + 1).toInt(), 0, 59));
        outSched.lastTriggeredMinute = -1;
        return true;
    }

    if (payload == "OFF" || payload == "DISABLE") {
        outSched.enabled = false;
        return true;
    }

    return false;
}

// ============================================================================
// MQTT Callback & Connection Management
// ============================================================================
static void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    String msg;
    msg.reserve(length + 1);
    for (unsigned int i = 0; i < length; i++) {
        msg += static_cast<char>(payload[i]);
    }
    msg.trim();

    String topicStr(topic);
    Serial.printf("[MQTT RX] %s -> %s\n", topic, msg.c_str());

    if (topicStr == TOPIC_CMD_MOVE) {
        String u = msg;
        u.toUpperCase();
        if (u == "FORWARD" || u == "FWD" || u == "W")       enqueueCommand(CMD_MOVE, DIR_FORWARD);
        else if (u == "BACKWARD" || u == "REV" || u == "S") enqueueCommand(CMD_MOVE, DIR_BACKWARD);
        else if (u == "LEFT" || u == "A")                  enqueueCommand(CMD_MOVE, DIR_LEFT);
        else if (u == "RIGHT" || u == "D")                 enqueueCommand(CMD_MOVE, DIR_RIGHT);
        else                                               enqueueCommand(CMD_MOVE, DIR_STOP);
    } else if (topicStr == TOPIC_CMD_MODE) {
        String u = msg;
        u.toUpperCase();
        if (u == "MANUAL")          enqueueCommand(CMD_SET_MODE, MODE_MANUAL);
        else if (u == "AUTONOMOUS") enqueueCommand(CMD_SET_MODE, MODE_AUTONOMOUS);
        else if (u == "STATIONARY") enqueueCommand(CMD_SET_MODE, MODE_STATIONARY);
    } else if (topicStr == TOPIC_CMD_SPEED) {
        int spd = constrain(msg.toInt(), 0, 255);
        enqueueCommand(CMD_SET_SPEED, static_cast<uint8_t>(spd));
    } else if (topicStr == TOPIC_CMD_PATH) {
        String u = msg;
        u.toUpperCase();
        if (u == "REC_START")      enqueueCommand(CMD_PATH_REC_START);
        else if (u == "REC_STOP")  enqueueCommand(CMD_PATH_REC_STOP);
        else if (u == "PLAY")      enqueueCommand(CMD_PATH_PLAY);
        else if (u == "PAUSE")     enqueueCommand(CMD_PATH_PAUSE);
        else if (u == "CLEAR")     enqueueCommand(CMD_PATH_CLEAR);
        else if (u == "LOOP")      enqueueCommand(CMD_PATH_LOOP_TOGGLE);
        else if (u == "DEMO")      enqueueCommand(CMD_PATH_LOAD_DEMO);
    } else if (topicStr == TOPIC_CMD_SCHEDULE) {
        PatrolSchedule updated{};
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            updated = gSchedule;
            if (parseScheduleString(msg, updated)) {
                gSchedule = updated;
            }
            xSemaphoreGive(xStateMutex);
        }
    } else if (topicStr == TOPIC_CMD_SLEEP) {
        String u = msg;
        u.toUpperCase();
        if (u == "LIGHT_SLEEP" || u == "SLEEP") {
            sLightSleepFlag = true;
        }
    }
}

static void maintainWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!sNtpConfigured) {
            configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER_1, NTP_SERVER_2);
            sNtpConfigured = true;
            Serial.printf("[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
        }
        return;
    }

    uint32_t now = millis();
    if (now - sLastWifiAttemptMs >= 5000) {
        sLastWifiAttemptMs = now;
        Serial.printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
}

static void maintainMqtt() {
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }
    if (sMqttClient.connected()) {
        sMqttClient.loop();
        return;
    }

    uint32_t now = millis();
    if (now - sLastMqttAttemptMs < 3000) {
        return;
    }
    sLastMqttAttemptMs = now;

    String clientId = "RC-Surv-ESP32-" + String(static_cast<uint32_t>(ESP.getEfuseMac()), HEX);
    Serial.printf("[MQTT] Connecting to %s:%d as %s...\n",
                  MQTT_BROKER, MQTT_PORT, clientId.c_str());

    if (sMqttClient.connect(clientId.c_str())) {
        Serial.println("[MQTT] Connected!");
        sMqttClient.subscribe(TOPIC_CMD_MOVE);
        sMqttClient.subscribe(TOPIC_CMD_MODE);
        sMqttClient.subscribe(TOPIC_CMD_SPEED);
        sMqttClient.subscribe(TOPIC_CMD_PATH);
        sMqttClient.subscribe(TOPIC_CMD_SCHEDULE);
        sMqttClient.subscribe(TOPIC_CMD_SLEEP);
    } else {
        Serial.printf("[MQTT] Failed (rc=%d), will retry\n", sMqttClient.state());
    }
}

// ============================================================================
// Power Management (Modem Sleep + Light Sleep in STATIONARY Mode)
// ============================================================================
static void updateStationaryPowerProfile() {
    SystemMode mode = MODE_MANUAL;
    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        mode = gCurrentMode;
        xSemaphoreGive(xStateMutex);
    }

    if (mode == MODE_STATIONARY && !sPowerSaveActive) {
        WiFi.setSleep(true);
        setCpuFrequencyMhz(80);
        sPowerSaveActive = true;
        Serial.println("[POWER] STATIONARY: Modem Sleep ON, CPU -> 80 MHz");
    } else if (mode != MODE_STATIONARY && sPowerSaveActive) {
        setCpuFrequencyMhz(240);
        WiFi.setSleep(false);
        sPowerSaveActive = false;
        Serial.println("[POWER] ACTIVE: Modem Sleep OFF, CPU -> 240 MHz");
    }
}

static void enterStationaryLightSleep(uint32_t durationMs) {
    SystemMode mode = MODE_MANUAL;
    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        mode = gCurrentMode;
        xSemaphoreGive(xStateMutex);
    }
    if (mode != MODE_STATIONARY) {
        Serial.println("[POWER] Light Sleep skipped (only allowed in STATIONARY mode)");
        return;
    }

    Serial.printf("[POWER] Entering Light Sleep for %lu ms (EXT0 PIR GPIO %d + Timer)...\n",
                  static_cast<unsigned long>(durationMs), PIN_PIR);
    Serial.flush();

    sMqttClient.disconnect();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(PIN_PIR), 1);
    esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(durationMs) * 1000ULL);
    esp_light_sleep_start();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    sLastWifiAttemptMs = millis();

    AlertEvent evt{};
    evt.timestampMs = millis();
    if (cause == ESP_SLEEP_WAKEUP_EXT0) {
        evt.type = ALERT_PIR_MOTION;
        snprintf(evt.message, sizeof(evt.message), "PIR woke ESP32 from Light Sleep!");
        Serial.println("[POWER] Woken by PIR EXT0 interrupt!");
    } else {
        evt.type = ALERT_SLEEP_WAKE;
        snprintf(evt.message, sizeof(evt.message), "Light Sleep timer wakeup");
        Serial.println("[POWER] Woken by Light Sleep timer");
    }
    xQueueSend(xAlertQueue, &evt, 0);
}

// ============================================================================
// NTP Scheduler & Telemetry / Alert Publishing
// ============================================================================
static void checkNtpSchedule() {
    uint32_t now = millis();
    if (now - sLastNtpCheckMs < 1000) {
        return;
    }
    sLastNtpCheckMs = now;

    struct tm timeinfo{};
    if (!getLocalTime(&timeinfo, 10)) {
        return;
    }

    bool trigger = false;
    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (gSchedule.enabled &&
            timeinfo.tm_hour == gSchedule.hour &&
            timeinfo.tm_min == gSchedule.minute &&
            gSchedule.lastTriggeredMinute != timeinfo.tm_min) {
            gSchedule.lastTriggeredMinute = timeinfo.tm_min;
            trigger = true;
        }
        xSemaphoreGive(xStateMutex);
    }

    if (trigger) {
        Serial.printf("[NTP] Scheduled patrol triggered at %02d:%02d\n",
                      timeinfo.tm_hour, timeinfo.tm_min);
        enqueueCommand(CMD_PATROL_TRIGGER);

        AlertEvent evt{};
        evt.type = ALERT_PATROL_START;
        evt.timestampMs = now;
        snprintf(evt.message, sizeof(evt.message),
                 "Scheduled patrol started (%02d:%02d)", timeinfo.tm_hour, timeinfo.tm_min);
        xQueueSend(xAlertQueue, &evt, 0);
    }
}

static void drainAlertQueue() {
    AlertEvent evt{};
    while (xQueueReceive(xAlertQueue, &evt, 0) == pdTRUE) {
        JsonDocument doc;
        doc["message"] = evt.message;
        doc["timestamp_ms"] = evt.timestampMs;

        const char* targetTopic = TOPIC_ALERT_SYSTEM;
        switch (evt.type) {
            case ALERT_PIR_MOTION:
                doc["event"] = "pir_motion";
                targetTopic = TOPIC_ALERT_PIR;
                break;
            case ALERT_OBSTACLE:
                doc["event"] = "obstacle";
                doc["distance_cm"] = roundf(evt.distanceCm * 10.0f) / 10.0f;
                break;
            case ALERT_PATROL_START:
                doc["event"] = "patrol_start";
                break;
            case ALERT_PATROL_DONE:
                doc["event"] = "patrol_done";
                break;
            case ALERT_SLEEP_WAKE:
                doc["event"] = "sleep_wake";
                break;
        }

        char buf[192];
        size_t len = serializeJson(doc, buf, sizeof(buf));
        if (sMqttClient.connected()) {
            sMqttClient.publish(targetTopic, buf, len);
        }
        Serial.printf("[ALERT -> %s] %s\n", targetTopic, buf);
    }
}

static void publishTelemetry() {
    if (!gTelemetryDue) {
        return;
    }
    gTelemetryDue = false;

    if (!sMqttClient.connected()) {
        return;
    }

    SystemMode mode = MODE_MANUAL;
    UltrasonicData us{};
    MotionStatus ms{};
    PatrolSchedule sched{};

    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        mode = gCurrentMode;
        us = gUltrasonic;
        ms = gMotionStatus;
        sched = gSchedule;
        xSemaphoreGive(xStateMutex);
    }

    char timeBuf[16] = "--:--:--";
    struct tm timeinfo{};
    if (getLocalTime(&timeinfo, 5)) {
        snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d",
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    }

    JsonDocument doc;
    doc["mode"] = modeToString(mode);
    doc["dir"] = dirToString(ms.activeDir);
    doc["speed"] = ms.speedPwm;
    doc["front_cm"] = roundf(us.frontCm * 10.0f) / 10.0f;
    doc["left_cm"] = roundf(us.leftCm * 10.0f) / 10.0f;
    doc["right_cm"] = roundf(us.rightCm * 10.0f) / 10.0f;
    doc["recording"] = ms.isRecording;
    doc["playing"] = ms.isPlaying;
    doc["loop"] = ms.loopPlayback;
    doc["bypassing"] = ms.isBypassing;
    doc["path_count"] = ms.pathCount;
    doc["step_idx"] = ms.currentStep;
    doc["step_rem_ms"] = ms.stepRemainingMs;
    doc["cpu_mhz"] = getCpuFrequencyMhz();
    doc["modem_sleep"] = sPowerSaveActive;
    doc["ntp_time"] = timeBuf;
    doc["sched_en"] = sched.enabled;

    char schedBuf[8];
    snprintf(schedBuf, sizeof(schedBuf), "%02u:%02u", sched.hour, sched.minute);
    doc["sched_time"] = schedBuf;

    char payload[384];
    size_t len = serializeJson(doc, payload, sizeof(payload));
    sMqttClient.publish(TOPIC_TELEMETRY, payload, len);
}

// ============================================================================
// Non-Blocking Serial CLI (Wokwi & Bench Testing)
// ============================================================================
static void pollSerialCommands() {
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r' || c == ' ') {
            continue;
        }
        switch (toupper(c)) {
            case 'W': enqueueCommand(CMD_MOVE, DIR_FORWARD); break;
            case 'S': enqueueCommand(CMD_MOVE, DIR_BACKWARD); break;
            case 'A': enqueueCommand(CMD_MOVE, DIR_LEFT); break;
            case 'D': enqueueCommand(CMD_MOVE, DIR_RIGHT); break;
            case 'X': enqueueCommand(CMD_MOVE, DIR_STOP); break;
            case '1': enqueueCommand(CMD_SET_MODE, MODE_MANUAL); break;
            case '2': enqueueCommand(CMD_SET_MODE, MODE_AUTONOMOUS); break;
            case '3': enqueueCommand(CMD_SET_MODE, MODE_STATIONARY); break;
            case 'R': enqueueCommand(CMD_PATH_REC_START); break;
            case 'T': enqueueCommand(CMD_PATH_REC_STOP); break;
            case 'P': enqueueCommand(CMD_PATH_PLAY); break;
            case 'C': enqueueCommand(CMD_PATH_CLEAR); break;
            case 'L': enqueueCommand(CMD_PATH_LOAD_DEMO); break;
            case 'Z': sLightSleepFlag = true; break;
            case '?': {
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("[STATUS] Mode=%s Dir=%s F=%.1f L=%.1f R=%.1f Steps=%u CPU=%uMHz\n",
                                  modeToString(gCurrentMode),
                                  dirToString(gMotionStatus.activeDir),
                                  gUltrasonic.frontCm, gUltrasonic.leftCm, gUltrasonic.rightCm,
                                  gMotionStatus.pathCount, getCpuFrequencyMhz());
                    xSemaphoreGive(xStateMutex);
                }
                break;
            }
            default:
                break;
        }
    }
}

// ============================================================================
// Public Init & Task (Core 0)
// ============================================================================
void commsInit() {
    sMqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    sMqttClient.setCallback(onMqttMessage);
    sMqttClient.setBufferSize(512);
}

void vCommsTask(void* pvParameters) {
    (void)pvParameters;

    for (;;) {
        pollSerialCommands();
        updateStationaryPowerProfile();

        if (sLightSleepFlag) {
            sLightSleepFlag = false;
            enterStationaryLightSleep(LIGHT_SLEEP_DURATION_MS);
        }

        maintainWiFi();
        maintainMqtt();
        checkNtpSchedule();
        drainAlertQueue();
        publishTelemetry();

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

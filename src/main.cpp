#include "config.h"
#include "motion_control.h"
#include "sensors_cam.h"
#include "comms_ui.h"

// ============================================================================
// Global FreeRTOS Primitives & Shared State
// ============================================================================
QueueHandle_t xCmdQueue         = nullptr;
QueueHandle_t xAlertQueue       = nullptr;
SemaphoreHandle_t xStateMutex   = nullptr;
SemaphoreHandle_t xPathMutex    = nullptr;
TimerHandle_t xHeartbeatTimer   = nullptr;

SystemMode gCurrentMode         = MODE_MANUAL;
UltrasonicData gUltrasonic      = {US_OUT_OF_RANGE_CM, US_OUT_OF_RANGE_CM, US_OUT_OF_RANGE_CM, 0};
PatrolSchedule gSchedule        = {false, 0, 0, false, -1};
MotionStatus gMotionStatus      = {DIR_STOP, DEFAULT_MOTOR_PWM, false, false, false, false, 0, 0, 0};
PathStep pathBuffer[MAX_PATH_STEPS] = {};
uint8_t gPathCount              = 0;
volatile bool gTelemetryDue     = false;

static void IRAM_ATTR onHeartbeatTimer(TimerHandle_t xTimer) {
    (void)xTimer;
    gTelemetryDue = true;
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== RC Car Security Surveillance System (Main ESP32) ===");
    Serial.println("Serial CLI: [W/A/S/D/X]=Move [1/2/3]=Mode [R/T/P/C/L]=Path [Z]=LightSleep [?]=Status");

    xStateMutex = xSemaphoreCreateMutex();
    xPathMutex  = xSemaphoreCreateMutex();
    xCmdQueue   = xQueueCreate(16, sizeof(ControlCommand));
    xAlertQueue = xQueueCreate(12, sizeof(AlertEvent));

    if (!xStateMutex || !xPathMutex || !xCmdQueue || !xAlertQueue) {
        Serial.println("[FATAL] Failed to create FreeRTOS synchronization primitives!");
        while (true) {
            delay(1000);
        }
    }

    sensorsInit();
    motionInit();
    commsInit();

    xHeartbeatTimer = xTimerCreate(
        "TelemetryTimer",
        pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS),
        pdTRUE,
        nullptr,
        onHeartbeatTimer
    );
    if (xHeartbeatTimer != nullptr) {
        xTimerStart(xHeartbeatTimer, 0);
    }

    // Core 0: Wi-Fi, MQTT, NTP Scheduler, Power Management
    xTaskCreatePinnedToCore(
        vCommsTask,
        "CommsTask",
        8192,
        nullptr,
        2,
        nullptr,
        CORE_COMMS
    );

    // Core 1: Sequential Ultrasonic Polling & PIR Intruder Monitoring
    xTaskCreatePinnedToCore(
        vSensorTask,
        "SensorTask",
        4096,
        nullptr,
        3,
        nullptr,
        CORE_REALTIME
    );

    // Core 1: L298N PWM Drive, Path Recording/Playback, Obstacle Bypass
    xTaskCreatePinnedToCore(
        vMotionTask,
        "MotionTask",
        4096,
        nullptr,
        2,
        nullptr,
        CORE_REALTIME
    );

    Serial.println("[BOOT] FreeRTOS tasks pinned (Core 0: Comms, Core 1: Sensors+Motion)");
}

void loop() {
    vTaskDelete(nullptr);
}

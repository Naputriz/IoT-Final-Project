#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>

// ============================================================================
// Network, MQTT & NTP Configuration (override via include/secrets.h)
// ============================================================================
#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID           "Wokwi-GUEST"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD       ""
#endif

#ifndef MQTT_BROKER
#define MQTT_BROKER         "broker.hivemq.com"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT           1883
#endif

#ifndef MQTT_USER
#define MQTT_USER           ""
#endif

#ifndef MQTT_PASS
#define MQTT_PASS           ""
#endif

// NTP Configuration (GMT+7 / WIB = 25200 seconds)
#define NTP_SERVER_1        "pool.ntp.org"
#define NTP_SERVER_2        "time.nist.gov"
#define GMT_OFFSET_SEC      25200L
#define DAYLIGHT_OFFSET_SEC 0

// ============================================================================
// MQTT Topics (Unified namespace matching vision/ and nodered/)
// ============================================================================
#define TOPIC_PREFIX        "iot_finpro_naputriz/rc"

// Subscribed Command Topics
#define TOPIC_CMD_MOVE      TOPIC_PREFIX "/cmd/move"
#define TOPIC_CMD_MODE      TOPIC_PREFIX "/cmd/mode"
#define TOPIC_CMD_SPEED     TOPIC_PREFIX "/cmd/speed"
#define TOPIC_CMD_PATH      TOPIC_PREFIX "/cmd/path"
#define TOPIC_CMD_SCHEDULE  TOPIC_PREFIX "/cmd/schedule"
#define TOPIC_CMD_SLEEP     TOPIC_PREFIX "/cmd/sleep"

// Published Telemetry & Alert Topics
#define TOPIC_TELEMETRY     TOPIC_PREFIX "/telemetry"
#define TOPIC_ALERT_PIR     TOPIC_PREFIX "/alert/pir"
#define TOPIC_ALERT_SYSTEM  TOPIC_PREFIX "/alert/system"

// ============================================================================
// Shared 13-GPIO Pinout (ESP32 DevKit V1)
// - Avoids flash pins GPIO 6-11 and strapping pins GPIO 0, 2, 5, 12, 15
// - Input-only GPIOs 34 & 35 are used strictly for HC-SR04 ECHO inputs
// ============================================================================
// L298N H-Bridge Motor Driver (6 GPIOs)
#define PIN_MOTOR_ENA       25
#define PIN_MOTOR_IN1       26  // Left Motor Forward  (Wokwi LED 1)
#define PIN_MOTOR_IN2       27  // Left Motor Reverse  (Wokwi LED 2)
#define PIN_MOTOR_IN3       14  // Right Motor Forward (Wokwi LED 3)
#define PIN_MOTOR_IN4       13  // Right Motor Reverse (Wokwi LED 4)
#define PIN_MOTOR_ENB       32

// 3x HC-SR04 Ultrasonic Sensors (6 GPIOs)
#define PIN_US_FRONT_TRIG   18
#define PIN_US_FRONT_ECHO   34  // Input-only
#define PIN_US_LEFT_TRIG    19
#define PIN_US_LEFT_ECHO    35  // Input-only
#define PIN_US_RIGHT_TRIG   21
#define PIN_US_RIGHT_ECHO   22

// HC-SR501 PIR Motion Sensor (1 GPIO — RTC_GPIO8 for Light Sleep EXT0 wakeup)
#define PIN_PIR             33

// ============================================================================
// Timing, Safety & Buffer Constants
// ============================================================================
#define US_STAGGER_DELAY_MS     35      // >= 30 ms between sensors to prevent crosstalk
#define US_PULSE_TIMEOUT_US     25000UL // ~4.3 m max range timeout
#define US_OUT_OF_RANGE_CM      400.0f
#define OBSTACLE_THRESHOLD_CM   20.0f   // Emergency stop / bypass threshold
#define SIDE_OBSTACLE_CM        15.0f   // Side guard threshold during manual turns

#define PIR_ALERT_COOLDOWN_MS   3000UL
#define OBSTACLE_COOLDOWN_MS    2500UL
#define TELEMETRY_INTERVAL_MS   2000UL
#define LIGHT_SLEEP_DURATION_MS 10000UL

#define MAX_PATH_STEPS          64
#define MIN_STEP_DURATION_MS    50UL
#define DEFAULT_MOTOR_PWM       190
#define MOTOR_PWM_FREQ          5000
#define MOTOR_PWM_RES_BITS      8

// FreeRTOS Core Pinning
#define CORE_COMMS              0
#define CORE_REALTIME           1

// ============================================================================
// Shared Enums & Data Structures
// ============================================================================
enum SystemMode : uint8_t {
    MODE_MANUAL = 0,
    MODE_AUTONOMOUS = 1,
    MODE_STATIONARY = 2
};

enum MoveDirection : uint8_t {
    DIR_STOP = 0,
    DIR_FORWARD = 1,
    DIR_BACKWARD = 2,
    DIR_LEFT = 3,
    DIR_RIGHT = 4
};

struct PathStep {
    MoveDirection direction;
    uint32_t durationMs;
};

enum CommandType : uint8_t {
    CMD_NONE = 0,
    CMD_MOVE,
    CMD_SET_MODE,
    CMD_SET_SPEED,
    CMD_PATH_REC_START,
    CMD_PATH_REC_STOP,
    CMD_PATH_PLAY,
    CMD_PATH_PAUSE,
    CMD_PATH_CLEAR,
    CMD_PATH_LOOP_TOGGLE,
    CMD_PATH_LOAD_DEMO,
    CMD_PATROL_TRIGGER,
    CMD_LIGHT_SLEEP
};

struct ControlCommand {
    CommandType type;
    uint8_t value;
};

enum AlertType : uint8_t {
    ALERT_PIR_MOTION = 0,
    ALERT_OBSTACLE,
    ALERT_PATROL_START,
    ALERT_PATROL_DONE,
    ALERT_SLEEP_WAKE
};

struct AlertEvent {
    AlertType type;
    float distanceCm;
    uint32_t timestampMs;
    char message[64];
};

struct UltrasonicData {
    float frontCm;
    float leftCm;
    float rightCm;
    uint32_t updatedAtMs;
};

struct PatrolSchedule {
    bool enabled;
    uint8_t hour;
    uint8_t minute;
    bool loopPatrol;
    int8_t lastTriggeredMinute;
};

struct MotionStatus {
    MoveDirection activeDir;
    uint8_t speedPwm;
    bool isRecording;
    bool isPlaying;
    bool loopPlayback;
    bool isBypassing;
    uint8_t pathCount;
    uint8_t currentStep;
    uint32_t stepRemainingMs;
};

// ============================================================================
// FreeRTOS Handles & Shared State (defined in src/main.cpp)
// LOCK ORDERING RULE: Never acquire xStateMutex while holding xPathMutex.
// Always acquire xStateMutex -> xPathMutex if both are needed, or lock
// them independently and release before acquiring the other.
// ============================================================================
extern QueueHandle_t xCmdQueue;
extern QueueHandle_t xAlertQueue;
extern SemaphoreHandle_t xStateMutex;
extern SemaphoreHandle_t xPathMutex;
extern TimerHandle_t xHeartbeatTimer;

extern SystemMode gCurrentMode;
extern UltrasonicData gUltrasonic;
extern PatrolSchedule gSchedule;
extern MotionStatus gMotionStatus;
extern PathStep pathBuffer[MAX_PATH_STEPS];
extern uint8_t gPathCount;
extern volatile bool gTelemetryDue;

// Helper conversion utilities
inline const char* modeToString(SystemMode m) {
    switch (m) {
        case MODE_MANUAL:     return "MANUAL";
        case MODE_AUTONOMOUS: return "AUTONOMOUS";
        case MODE_STATIONARY: return "STATIONARY";
        default:              return "UNKNOWN";
    }
}

inline const char* dirToString(MoveDirection d) {
    switch (d) {
        case DIR_FORWARD:  return "FORWARD";
        case DIR_BACKWARD: return "BACKWARD";
        case DIR_LEFT:     return "LEFT";
        case DIR_RIGHT:    return "RIGHT";
        default:           return "STOP";
    }
}

#include "sensors_cam.h"

static volatile bool sPirTriggered = false;
static uint32_t sLastPirAlertMs = 0;

void IRAM_ATTR pirISR() {
    sPirTriggered = true;
}

void sensorsInit() {
    pinMode(PIN_US_FRONT_TRIG, OUTPUT);
    pinMode(PIN_US_FRONT_ECHO, INPUT);
    pinMode(PIN_US_LEFT_TRIG, OUTPUT);
    pinMode(PIN_US_LEFT_ECHO, INPUT);
    pinMode(PIN_US_RIGHT_TRIG, OUTPUT);
    pinMode(PIN_US_RIGHT_ECHO, INPUT);

    digitalWrite(PIN_US_FRONT_TRIG, LOW);
    digitalWrite(PIN_US_LEFT_TRIG, LOW);
    digitalWrite(PIN_US_RIGHT_TRIG, LOW);

    pinMode(PIN_PIR, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_PIR), pirISR, RISING);
}

float readUltrasonicCm(uint8_t trigPin, uint8_t echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, US_PULSE_TIMEOUT_US);
    if (duration == 0) {
        return US_OUT_OF_RANGE_CM;
    }
    return (static_cast<float>(duration) * 0.0343f) * 0.5f;
}

void vSensorTask(void* pvParameters) {
    (void)pvParameters;

    for (;;) {
        SystemMode mode = MODE_MANUAL;
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            mode = gCurrentMode;
            xSemaphoreGive(xStateMutex);
        }

        if (mode == MODE_STATIONARY) {
            // Skip ultrasonic triggering in stationary mode to conserve power
            bool motionDetected = false;
            if (sPirTriggered) {
                sPirTriggered = false;
                motionDetected = true;
            } else if (digitalRead(PIN_PIR) == HIGH) {
                motionDetected = true;
            }

            uint32_t now = millis();
            if (motionDetected && (now - sLastPirAlertMs >= PIR_ALERT_COOLDOWN_MS)) {
                sLastPirAlertMs = now;

                AlertEvent alert{};
                alert.type = ALERT_PIR_MOTION;
                alert.distanceCm = 0.0f;
                alert.timestampMs = now;
                snprintf(alert.message, sizeof(alert.message),
                         "PIR motion detected in STATIONARY mode");

                xQueueSend(xAlertQueue, &alert, 0);
                Serial.println("[SENSOR] PIR intruder alert queued!");
            }

            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Clear any stale PIR flag while car is actively moving
        sPirTriggered = false;

        // Sequential 3x HC-SR04 polling with >= 30 ms stagger to prevent acoustic crosstalk
        float front = readUltrasonicCm(PIN_US_FRONT_TRIG, PIN_US_FRONT_ECHO);
        vTaskDelay(pdMS_TO_TICKS(US_STAGGER_DELAY_MS));

        float left = readUltrasonicCm(PIN_US_LEFT_TRIG, PIN_US_LEFT_ECHO);
        vTaskDelay(pdMS_TO_TICKS(US_STAGGER_DELAY_MS));

        float right = readUltrasonicCm(PIN_US_RIGHT_TRIG, PIN_US_RIGHT_ECHO);

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            gUltrasonic.frontCm = front;
            gUltrasonic.leftCm = left;
            gUltrasonic.rightCm = right;
            gUltrasonic.updatedAtMs = millis();
            xSemaphoreGive(xStateMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(US_STAGGER_DELAY_MS));
    }
}

#include "motion_control.h"

// LEDC channels for ESP32 Arduino Core < 3.0 compatibility
static constexpr uint8_t PWM_CH_LEFT  = 0;
static constexpr uint8_t PWM_CH_RIGHT = 1;

enum BypassState : uint8_t {
    BYPASS_NONE = 0,
    BYPASS_BRAKE,
    BYPASS_REVERSE,
    BYPASS_TURN_OUT,
    BYPASS_CLEAR_FWD,
    BYPASS_TURN_IN,
    BYPASS_SETTLE
};

// Internal motion state (owned exclusively by vMotionTask on Core 1)
static SystemMode sLocalMode          = MODE_MANUAL;
static MoveDirection sManualDir       = DIR_STOP;
static MoveDirection sAppliedDir      = DIR_STOP;
static uint8_t sSpeedPwm              = DEFAULT_MOTOR_PWM;

// Path recording state
static bool sIsRecording              = false;
static bool sRecStartedMoving         = false;
static MoveDirection sRecCurrentDir   = DIR_STOP;
static uint32_t sRecStepStartMs       = 0;

// Autonomous playback state
static bool sIsPlaying                = false;
static bool sLoopPlayback             = false;
static uint8_t sPlayStepIdx           = 0;
static uint32_t sStepRemainingMs      = 0;
static uint32_t sLastTickMs           = 0;

// Obstacle bypass state
static BypassState sBypassState       = BYPASS_NONE;
static uint32_t sBypassStateStartMs   = 0;
static MoveDirection sBypassTurnDir   = DIR_RIGHT;
static uint32_t sLastObstacleAlertMs  = 0;

// ============================================================================
// Low-Level L298N Motor Driver
// ============================================================================
static void writeMotorPwm(uint8_t leftPwm, uint8_t rightPwm) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PIN_MOTOR_ENA, leftPwm);
    ledcWrite(PIN_MOTOR_ENB, rightPwm);
#else
    ledcWrite(PWM_CH_LEFT, leftPwm);
    ledcWrite(PWM_CH_RIGHT, rightPwm);
#endif
}

static void applyMotorHardware(MoveDirection dir, uint8_t pwm) {
    sAppliedDir = dir;

    switch (dir) {
        case DIR_FORWARD:
            digitalWrite(PIN_MOTOR_IN1, HIGH);
            digitalWrite(PIN_MOTOR_IN2, LOW);
            digitalWrite(PIN_MOTOR_IN3, HIGH);
            digitalWrite(PIN_MOTOR_IN4, LOW);
            writeMotorPwm(pwm, pwm);
            break;
        case DIR_BACKWARD:
            digitalWrite(PIN_MOTOR_IN1, LOW);
            digitalWrite(PIN_MOTOR_IN2, HIGH);
            digitalWrite(PIN_MOTOR_IN3, LOW);
            digitalWrite(PIN_MOTOR_IN4, HIGH);
            writeMotorPwm(pwm, pwm);
            break;
        case DIR_LEFT:
            digitalWrite(PIN_MOTOR_IN1, LOW);
            digitalWrite(PIN_MOTOR_IN2, HIGH);
            digitalWrite(PIN_MOTOR_IN3, HIGH);
            digitalWrite(PIN_MOTOR_IN4, LOW);
            writeMotorPwm(pwm, pwm);
            break;
        case DIR_RIGHT:
            digitalWrite(PIN_MOTOR_IN1, HIGH);
            digitalWrite(PIN_MOTOR_IN2, LOW);
            digitalWrite(PIN_MOTOR_IN3, LOW);
            digitalWrite(PIN_MOTOR_IN4, HIGH);
            writeMotorPwm(pwm, pwm);
            break;
        case DIR_STOP:
        default:
            digitalWrite(PIN_MOTOR_IN1, LOW);
            digitalWrite(PIN_MOTOR_IN2, LOW);
            digitalWrite(PIN_MOTOR_IN3, LOW);
            digitalWrite(PIN_MOTOR_IN4, LOW);
            writeMotorPwm(0, 0);
            break;
    }
}

void stopMotors() {
    applyMotorHardware(DIR_STOP, 0);
}

void motionInit() {
    pinMode(PIN_MOTOR_IN1, OUTPUT);
    pinMode(PIN_MOTOR_IN2, OUTPUT);
    pinMode(PIN_MOTOR_IN3, OUTPUT);
    pinMode(PIN_MOTOR_IN4, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_MOTOR_ENA, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
    ledcAttach(PIN_MOTOR_ENB, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
#else
    ledcSetup(PWM_CH_LEFT, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_ENA, PWM_CH_LEFT);
    ledcSetup(PWM_CH_RIGHT, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_ENB, PWM_CH_RIGHT);
#endif

    stopMotors();
}

// ============================================================================
// Alert & Shared State Helpers
// ============================================================================
static void emitAlert(AlertType type, float distanceCm, const char* msg) {
    AlertEvent alert{};
    alert.type = type;
    alert.distanceCm = distanceCm;
    alert.timestampMs = millis();
    snprintf(alert.message, sizeof(alert.message), "%s", msg);
    xQueueSend(xAlertQueue, &alert, 0);
}

static void emitObstacleAlert(float distanceCm, const char* context) {
    uint32_t now = millis();
    if (now - sLastObstacleAlertMs < OBSTACLE_COOLDOWN_MS) {
        return;
    }
    sLastObstacleAlertMs = now;

    char msg[64];
    snprintf(msg, sizeof(msg), "Obstacle %.1f cm (%s)", distanceCm, context);
    emitAlert(ALERT_OBSTACLE, distanceCm, msg);
}

static void syncMotionStatus(uint8_t pathCount) {
    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        gMotionStatus.activeDir = sAppliedDir;
        gMotionStatus.speedPwm = sSpeedPwm;
        gMotionStatus.isRecording = sIsRecording;
        gMotionStatus.isPlaying = sIsPlaying;
        gMotionStatus.loopPlayback = sLoopPlayback;
        gMotionStatus.isBypassing = (sBypassState != BYPASS_NONE);
        gMotionStatus.pathCount = pathCount;
        gMotionStatus.currentStep = sPlayStepIdx;
        gMotionStatus.stepRemainingMs = sStepRemainingMs;
        xSemaphoreGive(xStateMutex);
    }
}

// ============================================================================
// Path Recording & Buffer Management (Protected by xPathMutex)
// ============================================================================
static void commitRecordedStep(MoveDirection dir, uint32_t durationMs) {
    if (durationMs < MIN_STEP_DURATION_MS) {
        return;
    }
    if (xSemaphoreTake(xPathMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        if (gPathCount < MAX_PATH_STEPS) {
            pathBuffer[gPathCount].direction = dir;
            pathBuffer[gPathCount].durationMs = durationMs;
            gPathCount++;
            Serial.printf("[PATH] Recorded step #%u: %s (%lu ms)\n",
                          gPathCount, dirToString(dir),
                          static_cast<unsigned long>(durationMs));
        }
        xSemaphoreGive(xPathMutex);
    }
}

static void startRecording() {
    if (xSemaphoreTake(xPathMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        gPathCount = 0;
        xSemaphoreGive(xPathMutex);
    }
    sIsRecording = true;
    sRecStartedMoving = false;
    sRecCurrentDir = DIR_STOP;
    sRecStepStartMs = millis();
    Serial.println("[PATH] Recording started (buffer cleared)");
}

static void stopRecording() {
    if (!sIsRecording) {
        return;
    }
    uint32_t elapsed = millis() - sRecStepStartMs;
    if (sRecStartedMoving && sRecCurrentDir != DIR_STOP) {
        commitRecordedStep(sRecCurrentDir, elapsed);
    }
    sIsRecording = false;
    sRecStartedMoving = false;
    sRecCurrentDir = DIR_STOP;
    Serial.println("[PATH] Recording stopped");
}

static void recordDirectionTransition(MoveDirection newDir) {
    if (!sIsRecording || newDir == sRecCurrentDir) {
        return;
    }

    uint32_t now = millis();
    if (!sRecStartedMoving) {
        if (newDir == DIR_STOP) {
            return;
        }
        sRecStartedMoving = true;
        sRecCurrentDir = newDir;
        sRecStepStartMs = now;
        return;
    }

    commitRecordedStep(sRecCurrentDir, now - sRecStepStartMs);
    sRecCurrentDir = newDir;
    sRecStepStartMs = now;
}

static void clearPathBuffer() {
    stopRecording();
    sIsPlaying = false;
    sBypassState = BYPASS_NONE;
    sPlayStepIdx = 0;
    sStepRemainingMs = 0;
    if (xSemaphoreTake(xPathMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        gPathCount = 0;
        xSemaphoreGive(xPathMutex);
    }
    stopMotors();
    Serial.println("[PATH] Buffer cleared");
}

static void loadDemoPath() {
    stopRecording();
    sIsPlaying = false;
    sBypassState = BYPASS_NONE;
    if (xSemaphoreTake(xPathMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        const PathStep demo[] = {
            {DIR_FORWARD, 2000},
            {DIR_RIGHT,    600},
            {DIR_FORWARD, 2000},
            {DIR_RIGHT,    600}
        };
        gPathCount = sizeof(demo) / sizeof(demo[0]);
        for (uint8_t i = 0; i < gPathCount; i++) {
            pathBuffer[i] = demo[i];
        }
        xSemaphoreGive(xPathMutex);
    }
    sPlayStepIdx = 0;
    sStepRemainingMs = 0;
    Serial.println("[PATH] Loaded 4-step demo patrol path");
}

static bool startPlayback(bool forceFromStart) {
    uint8_t count = 0;
    uint32_t firstDuration = 0;
    if (xSemaphoreTake(xPathMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        count = gPathCount;
        if (count > 0) {
            if (forceFromStart || sPlayStepIdx >= count || sStepRemainingMs == 0) {
                sPlayStepIdx = 0;
                sStepRemainingMs = pathBuffer[0].durationMs;
            }
            firstDuration = sStepRemainingMs;
        }
        xSemaphoreGive(xPathMutex);
    }

    if (count == 0 || firstDuration == 0) {
        sIsPlaying = false;
        stopMotors();
        Serial.println("[PATH] Cannot start playback: pathBuffer is empty");
        return false;
    }

    sIsPlaying = true;
    sBypassState = BYPASS_NONE;
    sLastTickMs = millis();
    Serial.printf("[PATH] Playback started (%u steps)\n", count);
    return true;
}

// ============================================================================
// Mode Transition & Command Processing
// ============================================================================
static void setSystemMode(SystemMode newMode, bool autoStartPatrol) {
    if (newMode == sLocalMode && !autoStartPatrol) {
        return;
    }

    if (newMode != MODE_MANUAL) {
        stopRecording();
        sManualDir = DIR_STOP;
    }
    if (newMode != MODE_AUTONOMOUS) {
        sIsPlaying = false;
        sBypassState = BYPASS_NONE;
    }
    if (newMode == MODE_STATIONARY) {
        stopMotors();
    }

    sLocalMode = newMode;
    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        gCurrentMode = newMode;
        xSemaphoreGive(xStateMutex);
    }

    Serial.printf("[MODE] Switched to %s\n", modeToString(newMode));

    if (newMode == MODE_AUTONOMOUS && autoStartPatrol) {
        startPlayback(true);
    } else if (newMode != MODE_MANUAL) {
        stopMotors();
    }
}

static void processCommand(const ControlCommand& cmd) {
    switch (cmd.type) {
        case CMD_SET_MODE: {
            SystemMode target = static_cast<SystemMode>(cmd.value);
            if (target <= MODE_STATIONARY) {
                setSystemMode(target, target == MODE_AUTONOMOUS);
            }
            break;
        }
        case CMD_MOVE: {
            if (sLocalMode != MODE_MANUAL) {
                Serial.println("[CMD] Ignored manual move outside MANUAL mode");
                break;
            }
            MoveDirection dir = static_cast<MoveDirection>(cmd.value);
            if (dir <= DIR_RIGHT) {
                sManualDir = dir;
            }
            break;
        }
        case CMD_SET_SPEED:
            sSpeedPwm = cmd.value;
            break;
        case CMD_PATH_REC_START:
            if (sLocalMode == MODE_MANUAL) {
                startRecording();
            } else {
                Serial.println("[PATH] Recording requires MANUAL mode");
            }
            break;
        case CMD_PATH_REC_STOP:
            stopRecording();
            break;
        case CMD_PATH_PLAY:
            if (sLocalMode != MODE_AUTONOMOUS) {
                setSystemMode(MODE_AUTONOMOUS, false);
            }
            startPlayback(false);
            break;
        case CMD_PATH_PAUSE:
            sIsPlaying = false;
            sBypassState = BYPASS_NONE;
            stopMotors();
            Serial.println("[PATH] Playback paused");
            break;
        case CMD_PATH_CLEAR:
            clearPathBuffer();
            break;
        case CMD_PATH_LOOP_TOGGLE:
            sLoopPlayback = !sLoopPlayback;
            Serial.printf("[PATH] Loop playback: %s\n", sLoopPlayback ? "ON" : "OFF");
            break;
        case CMD_PATH_LOAD_DEMO:
            loadDemoPath();
            break;
        case CMD_PATROL_TRIGGER: {
            bool loopFlag = false;
            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                loopFlag = gSchedule.loopPatrol;
                xSemaphoreGive(xStateMutex);
            }
            sLoopPlayback = loopFlag;
            setSystemMode(MODE_AUTONOMOUS, true);
            break;
        }
        default:
            break;
    }
}

// ============================================================================
// Autonomous Obstacle Bypass State Machine (Non-Blocking millis())
// ============================================================================
static void updateAutonomousBypass(const UltrasonicData& us, uint32_t now) {
    uint32_t elapsed = now - sBypassStateStartMs;
    MoveDirection oppDir = (sBypassTurnDir == DIR_LEFT) ? DIR_RIGHT : DIR_LEFT;

    switch (sBypassState) {
        case BYPASS_BRAKE:
            applyMotorHardware(DIR_STOP, 0);
            if (elapsed >= 250) {
                sBypassState = BYPASS_REVERSE;
                sBypassStateStartMs = now;
            }
            break;
        case BYPASS_REVERSE:
            applyMotorHardware(DIR_BACKWARD, sSpeedPwm);
            if (elapsed >= 400) {
                sBypassTurnDir = (us.leftCm >= us.rightCm) ? DIR_LEFT : DIR_RIGHT;
                sBypassState = BYPASS_TURN_OUT;
                sBypassStateStartMs = now;
            }
            break;
        case BYPASS_TURN_OUT:
            applyMotorHardware(sBypassTurnDir, sSpeedPwm);
            if (elapsed >= 450) {
                sBypassState = BYPASS_CLEAR_FWD;
                sBypassStateStartMs = now;
            }
            break;
        case BYPASS_CLEAR_FWD:
            if (us.frontCm < OBSTACLE_THRESHOLD_CM) {
                emitObstacleAlert(us.frontCm, "Bypass re-check");
                sBypassState = BYPASS_BRAKE;
                sBypassStateStartMs = now;
                break;
            }
            applyMotorHardware(DIR_FORWARD, sSpeedPwm);
            if (elapsed >= 600) {
                sBypassState = BYPASS_TURN_IN;
                sBypassStateStartMs = now;
            }
            break;
        case BYPASS_TURN_IN:
            applyMotorHardware(oppDir, sSpeedPwm);
            if (elapsed >= 450) {
                sBypassState = BYPASS_SETTLE;
                sBypassStateStartMs = now;
            }
            break;
        case BYPASS_SETTLE:
        default:
            applyMotorHardware(DIR_STOP, 0);
            if (elapsed >= 200) {
                sBypassState = BYPASS_NONE;
                sLastTickMs = now;
                Serial.println("[BYPASS] Complete, resuming path playback");
            }
            break;
    }
}

// ============================================================================
// Motion Task (Core 1, Priority 2, 20 ms period)
// ============================================================================
void vMotionTask(void* pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);

    for (;;) {
        ControlCommand cmd{};
        while (xQueueReceive(xCmdQueue, &cmd, 0) == pdTRUE) {
            processCommand(cmd);
        }

        UltrasonicData us{US_OUT_OF_RANGE_CM, US_OUT_OF_RANGE_CM, US_OUT_OF_RANGE_CM, 0};
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            us = gUltrasonic;
            xSemaphoreGive(xStateMutex);
        }

        uint32_t now = millis();
        uint8_t currentPathCount = 0;

        if (sLocalMode == MODE_MANUAL) {
            MoveDirection effectiveDir = sManualDir;

            if (effectiveDir == DIR_FORWARD && us.frontCm < OBSTACLE_THRESHOLD_CM) {
                effectiveDir = DIR_STOP;
                emitObstacleAlert(us.frontCm, "Manual front stop");
            } else if (effectiveDir == DIR_LEFT && us.leftCm < SIDE_OBSTACLE_CM) {
                effectiveDir = DIR_STOP;
                emitObstacleAlert(us.leftCm, "Manual left stop");
            } else if (effectiveDir == DIR_RIGHT && us.rightCm < SIDE_OBSTACLE_CM) {
                effectiveDir = DIR_STOP;
                emitObstacleAlert(us.rightCm, "Manual right stop");
            }

            applyMotorHardware(effectiveDir, sSpeedPwm);
            recordDirectionTransition(effectiveDir);

            if (xSemaphoreTake(xPathMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                currentPathCount = gPathCount;
                xSemaphoreGive(xPathMutex);
            }
        } else if (sLocalMode == MODE_AUTONOMOUS) {
            if (!sIsPlaying) {
                stopMotors();
                if (xSemaphoreTake(xPathMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    currentPathCount = gPathCount;
                    xSemaphoreGive(xPathMutex);
                }
            } else if (sBypassState != BYPASS_NONE) {
                updateAutonomousBypass(us, now);
                if (xSemaphoreTake(xPathMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    currentPathCount = gPathCount;
                    xSemaphoreGive(xPathMutex);
                }
            } else {
                PathStep activeStep{DIR_STOP, 0};
                if (xSemaphoreTake(xPathMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    currentPathCount = gPathCount;
                    if (sPlayStepIdx < gPathCount) {
                        activeStep = pathBuffer[sPlayStepIdx];
                    }
                    xSemaphoreGive(xPathMutex);
                }

                if (currentPathCount == 0 || sPlayStepIdx >= currentPathCount) {
                    sIsPlaying = false;
                    stopMotors();
                } else {
                    bool obstacleAhead =
                        (activeStep.direction == DIR_FORWARD && us.frontCm < OBSTACLE_THRESHOLD_CM) ||
                        (activeStep.direction == DIR_LEFT && (us.frontCm < OBSTACLE_THRESHOLD_CM || us.leftCm < SIDE_OBSTACLE_CM)) ||
                        (activeStep.direction == DIR_RIGHT && (us.frontCm < OBSTACLE_THRESHOLD_CM || us.rightCm < SIDE_OBSTACLE_CM));

                    if (obstacleAhead) {
                        float minDist = min(us.frontCm, min(us.leftCm, us.rightCm));
                        emitObstacleAlert(minDist, "Autonomous bypass");
                        sBypassState = BYPASS_BRAKE;
                        sBypassStateStartMs = now;
                        stopMotors();
                    } else {
                        uint32_t dt = now - sLastTickMs;
                        sLastTickMs = now;

                        applyMotorHardware(activeStep.direction, sSpeedPwm);

                        if (dt >= sStepRemainingMs) {
                            sStepRemainingMs = 0;
                            sPlayStepIdx++;

                            if (xSemaphoreTake(xPathMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                currentPathCount = gPathCount;
                                if (sPlayStepIdx >= gPathCount) {
                                    if (sLoopPlayback && gPathCount > 0) {
                                        sPlayStepIdx = 0;
                                        sStepRemainingMs = pathBuffer[0].durationMs;
                                    } else {
                                        sIsPlaying = false;
                                    }
                                } else {
                                    sStepRemainingMs = pathBuffer[sPlayStepIdx].durationMs;
                                }
                                xSemaphoreGive(xPathMutex);
                            }

                            if (!sIsPlaying) {
                                stopMotors();
                                emitAlert(ALERT_PATROL_DONE, 0.0f, "Autonomous patrol completed");
                                Serial.println("[PATH] Playback completed");
                            }
                        } else {
                            sStepRemainingMs -= dt;
                        }
                    }
                }
            }
        } else {
            // MODE_STATIONARY: Motors strictly disabled
            stopMotors();
            if (xSemaphoreTake(xPathMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                currentPathCount = gPathCount;
                xSemaphoreGive(xPathMutex);
            }
        }

        syncMotionStatus(currentPathCount);
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

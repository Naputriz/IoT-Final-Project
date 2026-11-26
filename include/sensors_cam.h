#pragma once

#include "config.h"

void sensorsInit();
float readUltrasonicCm(uint8_t trigPin, uint8_t echoPin);
void vSensorTask(void* pvParameters);

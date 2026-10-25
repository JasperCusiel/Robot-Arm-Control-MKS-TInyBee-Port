/*
 * PAROL6-compatible firmware port for the MKS TinyBee V1 (ESP32).
 * Modified implementation, 2026. Released under GPL-3.0.
 */
#include <Arduino.h>

#include "controller.h"

RobotController robot;

void setup() { robot.begin(); }
void loop() { robot.update(); }


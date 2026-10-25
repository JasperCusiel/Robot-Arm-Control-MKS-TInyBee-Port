#pragma once

#include <Arduino.h>

namespace config {

constexpr uint8_t JOINT_COUNT = 6;
constexpr uint32_t COMMANDER_BAUD = 3000000;

// TinyBee I2S shift-register pins. These are fixed by the PCB.
constexpr int I2S_WS_PIN = 26;
constexpr int I2S_BCK_PIN = 25;
constexpr int I2S_DATA_PIN = 27;

// The five onboard sockets, in X, Y, Z, E0, E1 order. Values are output-bit
// indices, not ESP32 GPIO numbers.
constexpr uint8_t STEP_BIT[5]   = {1, 4, 7, 10, 13};
constexpr uint8_t DIR_BIT[5]    = {2, 5, 8, 11, 14};
constexpr uint8_t ENABLE_BIT[5] = {0, 3, 6, 9, 12};

// Physical joint assignment. Joint indices are zero-based here: 0=J1 ... 5=J6.
// Each joint must appear exactly once across ONBOARD_SOCKET_JOINT and
// EXTERNAL_JOINT. Current arm: X=J1, external=J2, Y=J3, Z=J4, E0=J5, E1=J6.
constexpr uint8_t ONBOARD_SOCKET_JOINT[5] = {0, 2, 3, 4, 5};
constexpr uint8_t EXTERNAL_JOINT = 1;

// External J2 STEP/DIR driver connected to EXP1.
constexpr int EXTERNAL_STEP_PIN = 16;    // EXP1 pin labelled LCD_D7 / TXD2
constexpr int EXTERNAL_DIR_PIN = 17;     // EXP1 pin labelled LCD_D5 / RXD2
constexpr int EXTERNAL_ENABLE_PIN = 15;  // EXP1 pin labelled LCD_EN

// X/Y/Z endstop connectors followed by input-capable spare connectors.
// GPIO35, GPIO34 and GPIO39 have no internal pull-up: fit an external 4.7k-10k
// pull-up to 3.3 V when using a dry-contact switch.
constexpr int HOME_PIN[JOINT_COUNT] = {33, 32, 22, 35, 34, 39};

constexpr bool ESTOP_ENABLED = true;
constexpr int ESTOP_PIN = 147;       // EXP1 BTN_ENC
constexpr bool ESTOP_ACTIVE_LOW = true;

constexpr uint8_t FAN1_BIT = 19;  // Physical FAN1 connector, IO147
constexpr uint8_t FAN1_DUTY = 0; // 0–255; 51 = 20%
constexpr uint32_t FAN_PWM_HZ = 1000;

// Optional Commander digital I/O. -1 disables a channel safely.
constexpr int INPUT_PIN[2] = {-1, -1};
constexpr int OUTPUT_PIN[2] = {-1, -1};

constexpr uint32_t I2S_SAMPLE_RATE = 250000;
constexpr uint32_t EXTERNAL_TICK_RATE = 100000;
constexpr uint32_t CONTROL_RATE = 1000;
constexpr uint32_t HOME_TIMEOUT_MS = 45000;

struct JointConfig {
  bool direction_inverted;
  bool enable_active_low;
  bool home_active_low;
  int8_t home_direction;
  int32_t minimum_steps;
  int32_t maximum_steps;
  int32_t home_position_steps;
  uint32_t home_backoff_steps;
  float home_fast_steps_per_second;
  float home_slow_steps_per_second;
  float maximum_steps_per_second;
  float acceleration_steps_per_second2;
  float steps_per_degree;
};

// SAFE PLACEHOLDERS: configure these six rows for the actual arm before
// connecting motors. Defaults assume 200-step motors, 1/16 microstepping,
// no gearbox and +/-180 degrees. Speeds are intentionally conservative.
    constexpr JointConfig JOINT[JOINT_COUNT] = {
        {false, true, true, -1, -6720, 6720, 0, 160,
         800, 200, 6222.25f, 268801.2f, 74.667f}, // J1: X

        {false, true, true, -1, 0, 9600, 0, 160,
         800, 200, 4444.42f, 191998.8f, 53.333f}, // J2: external

        {false, true, true, -1, 0, 5120, 0, 160,
         800, 200, 2370.33f, 102398.4f, 28.444f}, // J3: Y

        {false, true, true, -1, -6720, 6720, 0, 160,
         800, 200, 2074.00f, 89596.8f, 24.888f}, // J4: Z

        {false, true, true, -1, -2000, 2000, 0, 160,
         800, 200, 1851.83f, 79999.2f, 22.222f}, // J5: E0

        {false, true, true, -1, -8388608, 8388607, 0, 160,
         500, 150, 148.13f, 31996.8f, 8.888f}, // J6: E1, unlimited
      };

}  // namespace config

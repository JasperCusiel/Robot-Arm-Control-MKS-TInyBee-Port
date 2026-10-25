#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "robot_config.h"

class MotionEngine {
 public:
  bool begin();
  void enable();
  void disable();
  bool enabled() const { return enabled_; }

  void setVelocity(uint8_t joint, float steps_per_second);
  void haltJoint(uint8_t joint);
  void setAllVelocities(const int32_t velocity[config::JOINT_COUNT]);
  void stop();
  void controlTick();

  int32_t position(uint8_t joint) const;
  int32_t velocity(uint8_t joint) const;
  void setPosition(uint8_t joint, int32_t position);
  bool positionError(uint8_t joint) const;
  void clearErrors();
  void setHomed(uint8_t joint, bool homed);
  bool homed(uint8_t joint) const;

  static uint32_t renderI2SSample();

 private:
  struct Axis {
    volatile int32_t position = 0;
    volatile int32_t actual_velocity = 0;
    volatile int32_t target_velocity = 0;
    volatile uint32_t phase = 0;
    volatile uint32_t phase_increment = 0;
    volatile int8_t direction = 1;
    volatile bool pulse_high = false;
    volatile bool homed = false;
    volatile bool position_error = false;
  };

  static MotionEngine* instance_;
  static void IRAM_ATTR externalTimerISR();
  static bool IRAM_ATTR renderAxis(Axis& axis, uint8_t joint);
  static void writeI2SBit(uint32_t& word, uint8_t bit, bool value);
  void updatePhaseIncrement(uint8_t joint);
  bool IRAM_ATTR limitWouldBeExceeded(uint8_t joint, int8_t direction) const;
  void setDriverEnable(bool enabled);

  Axis axis_[config::JOINT_COUNT];
  hw_timer_t* external_timer_ = nullptr;
  volatile bool enabled_ = false;
  volatile uint32_t i2s_word_ = 0;
};

extern MotionEngine motion;

#include "motion.h"

#include <math.h>
#include <soc/gpio_struct.h>

#include "tinybee_i2s.h"

MotionEngine motion;
MotionEngine* MotionEngine::instance_ = nullptr;

namespace {
constexpr uint64_t kPhaseScale = uint64_t(1) << 32;

inline void IRAM_ATTR nativePinWrite(int pin, bool high) {
  if (high) GPIO.out_w1ts = uint32_t(1) << pin;
  else GPIO.out_w1tc = uint32_t(1) << pin;
}
}

bool MotionEngine::begin() {
  instance_ = this;
  pinMode(config::EXTERNAL_STEP_PIN, OUTPUT);
  pinMode(config::EXTERNAL_DIR_PIN, OUTPUT);
  pinMode(config::EXTERNAL_ENABLE_PIN, OUTPUT);
  digitalWrite(config::EXTERNAL_STEP_PIN, LOW);
  setDriverEnable(false);

  if (!tinybeeI2SBegin(renderI2SSample)) return false;

  external_timer_ = timerBegin(1, 80, true);  // 1 MHz timer clock
  if (!external_timer_) return false;
  timerAttachInterrupt(external_timer_, &externalTimerISR, true);
  timerAlarmWrite(external_timer_, 1000000UL / config::EXTERNAL_TICK_RATE, true);
  timerAlarmEnable(external_timer_);
  return true;
}

void MotionEngine::writeI2SBit(uint32_t& word, uint8_t bit, bool value) {
  if (value) word |= uint32_t(1) << bit;
  else word &= ~(uint32_t(1) << bit);
}

void MotionEngine::setDriverEnable(bool enabled) {
  uint32_t word = i2s_word_;
  for (uint8_t socket = 0; socket < 5; ++socket) {
    const uint8_t joint = config::ONBOARD_SOCKET_JOINT[socket];
    const bool output = config::JOINT[joint].enable_active_low ? !enabled : enabled;
    writeI2SBit(word, config::ENABLE_BIT[socket], output);
  }
  i2s_word_ = word;
  const uint8_t external = config::EXTERNAL_JOINT;
  const bool external_output = config::JOINT[external].enable_active_low ? !enabled : enabled;
  digitalWrite(config::EXTERNAL_ENABLE_PIN, external_output);
}

void MotionEngine::enable() {
  enabled_ = true;
  setDriverEnable(true);
}

void MotionEngine::disable() {
  stop();
  enabled_ = false;
  setDriverEnable(false);
}

void MotionEngine::setVelocity(uint8_t joint, float steps_per_second) {
  if (joint >= config::JOINT_COUNT) return;
  const float maximum = config::JOINT[joint].maximum_steps_per_second;
  steps_per_second = constrain(steps_per_second, -maximum, maximum);
  axis_[joint].target_velocity = lroundf(steps_per_second);
}

void MotionEngine::haltJoint(uint8_t joint) {
  if (joint >= config::JOINT_COUNT) return;
  axis_[joint].target_velocity = 0;
  axis_[joint].actual_velocity = 0;
  axis_[joint].phase_increment = 0;
}

void MotionEngine::setAllVelocities(const int32_t velocity[config::JOINT_COUNT]) {
  for (uint8_t joint = 0; joint < config::JOINT_COUNT; ++joint)
    setVelocity(joint, velocity[joint]);
}

void MotionEngine::stop() {
  for (uint8_t joint = 0; joint < config::JOINT_COUNT; ++joint) {
    axis_[joint].target_velocity = 0;
    axis_[joint].actual_velocity = 0;
    axis_[joint].phase_increment = 0;
  }
}

void MotionEngine::controlTick() {
  for (uint8_t joint = 0; joint < config::JOINT_COUNT; ++joint) {
    Axis& axis = axis_[joint];
    const int32_t target = axis.target_velocity;
    int32_t actual = axis.actual_velocity;
    const int32_t calculated_change = lroundf(
        config::JOINT[joint].acceleration_steps_per_second2 / config::CONTROL_RATE);
    const int32_t change = calculated_change > 1 ? calculated_change : 1;
    if (actual < target) actual = actual + change < target ? actual + change : target;
    else if (actual > target) actual = actual - change > target ? actual - change : target;
    axis.actual_velocity = actual;
    updatePhaseIncrement(joint);
  }
}

void MotionEngine::updatePhaseIncrement(uint8_t joint) {
  const uint32_t tick_rate = joint == config::EXTERNAL_JOINT
                                 ? config::EXTERNAL_TICK_RATE
                                 : config::I2S_SAMPLE_RATE;
  const uint32_t speed = abs(axis_[joint].actual_velocity);
  axis_[joint].phase_increment = uint32_t((uint64_t(speed) * kPhaseScale) / tick_rate);
}

bool IRAM_ATTR MotionEngine::limitWouldBeExceeded(uint8_t joint, int8_t direction) const {
  const Axis& axis = axis_[joint];
  if (!axis.homed) return false;
  return (direction > 0 && axis.position >= config::JOINT[joint].maximum_steps) ||
         (direction < 0 && axis.position <= config::JOINT[joint].minimum_steps);
}

bool IRAM_ATTR MotionEngine::renderAxis(Axis& axis, uint8_t joint) {
  if (axis.pulse_high) {
    axis.pulse_high = false;
    return false;
  }
  const int32_t velocity = axis.actual_velocity;
  if (velocity == 0 || !instance_->enabled_) return false;
  const int8_t direction = velocity > 0 ? 1 : -1;
  axis.direction = direction;
  const uint32_t previous = axis.phase;
  axis.phase += axis.phase_increment;
  if (axis.phase >= previous) return false;

  if (instance_->limitWouldBeExceeded(joint, direction)) {
    axis.position_error = true;
    axis.actual_velocity = 0;
    axis.target_velocity = 0;
    axis.phase_increment = 0;
    return false;
  }
  axis.position += direction;
  axis.pulse_high = true;
  return true;
}

// uint32_t MotionEngine::renderI2SSample() {
//   MotionEngine* self = instance_;
//   if (!self) return 0;
//   uint32_t word = self->i2s_word_;
//   for (uint8_t socket = 0; socket < 5; ++socket) {
//     const uint8_t joint = config::ONBOARD_SOCKET_JOINT[socket];
//     Axis& axis = self->axis_[joint];
//     const bool direction_output = (axis.direction > 0) ^ config::JOINT[joint].direction_inverted;
//     writeI2SBit(word, config::DIR_BIT[socket], direction_output);
//     const bool pulse = renderAxis(axis, joint);
//     writeI2SBit(word, config::STEP_BIT[socket], pulse);
//   }
//   self->i2s_word_ = word;
//   return word;
// }
uint32_t MotionEngine::renderI2SSample() {
    MotionEngine* self = instance_;
    if (!self) return 0;

    uint32_t word = self->i2s_word_;

    static uint32_t fan_phase = 0;

    constexpr uint32_t fan_period_samples =
        config::I2S_SAMPLE_RATE / config::FAN_PWM_HZ;

    const uint32_t fan_on_samples =
        (fan_period_samples * config::FAN1_DUTY) / 255U;

    const bool fan_on =
        config::FAN1_DUTY == 255 ||
        fan_phase < fan_on_samples;

    writeI2SBit(word, config::FAN1_BIT, fan_on);

    if (++fan_phase >= fan_period_samples) {
        fan_phase = 0;
    }

    for (uint8_t socket = 0; socket < 5; ++socket) {
        const uint8_t joint = config::ONBOARD_SOCKET_JOINT[socket];
        Axis& axis = self->axis_[joint];

        // Refresh enable continuously to prevent an I2S state race.
        const bool enable_output =
            config::JOINT[joint].enable_active_low
                ? !self->enabled_
                : self->enabled_;

        writeI2SBit(
            word,
            config::ENABLE_BIT[socket],
            enable_output
        );

        const bool direction_output =
            (axis.direction > 0) ^
            config::JOINT[joint].direction_inverted;

        writeI2SBit(
            word,
            config::DIR_BIT[socket],
            direction_output
        );

        const bool pulse = renderAxis(axis, joint);

        writeI2SBit(
            word,
            config::STEP_BIT[socket],
            pulse
        );
    }

    self->i2s_word_ = word;
    return word;
}

void IRAM_ATTR MotionEngine::externalTimerISR() {
  MotionEngine* self = instance_;
  if (!self) return;
  const uint8_t joint = config::EXTERNAL_JOINT;
  Axis& axis = self->axis_[joint];
  const bool direction_output = (axis.direction > 0) ^ config::JOINT[joint].direction_inverted;
  nativePinWrite(config::EXTERNAL_DIR_PIN, direction_output);
  const bool pulse = renderAxis(axis, joint);
  nativePinWrite(config::EXTERNAL_STEP_PIN, pulse);
}

int32_t MotionEngine::position(uint8_t joint) const {
  return joint < config::JOINT_COUNT ? axis_[joint].position : 0;
}

int32_t MotionEngine::velocity(uint8_t joint) const {
  return joint < config::JOINT_COUNT ? axis_[joint].actual_velocity : 0;
}

void MotionEngine::setPosition(uint8_t joint, int32_t position) {
  if (joint < config::JOINT_COUNT) axis_[joint].position = position;
}

bool MotionEngine::positionError(uint8_t joint) const {
  return joint < config::JOINT_COUNT && axis_[joint].position_error;
}

void MotionEngine::clearErrors() {
  for (auto& axis : axis_) axis.position_error = false;
}

void MotionEngine::setHomed(uint8_t joint, bool homed) {
  if (joint < config::JOINT_COUNT) axis_[joint].homed = homed;
}

bool MotionEngine::homed(uint8_t joint) const {
  return joint < config::JOINT_COUNT && axis_[joint].homed;
}

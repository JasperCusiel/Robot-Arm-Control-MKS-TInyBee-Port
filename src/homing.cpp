#include "homing.h"

#include "motion.h"
#include "robot_config.h"

HomingController homing;

void HomingController::begin() {
  for (uint8_t joint = 0; joint < config::JOINT_COUNT; ++joint) {
    const int pin = config::HOME_PIN[joint];
    if (pin < 0) continue;
    // Input-only GPIO34-39 do not support the ESP32's internal pull-up.
    pinMode(pin, pin >= 34 ? INPUT : INPUT_PULLUP);
  }
}

bool HomingController::switchTriggered(uint8_t joint) const {
  const int pin = config::HOME_PIN[joint];
  if (pin < 0) return false;
  const bool level = digitalRead(pin);
  return config::JOINT[joint].home_active_low ? !level : level;
}

void HomingController::start() {
  cancel();
  for (uint8_t joint = 0; joint < config::JOINT_COUNT; ++joint)
    motion.setHomed(joint, false);
  joint_ = 0;
  beginJoint();
}

void HomingController::cancel() {
  motion.stop();
  state_ = State::Idle;
}

void HomingController::beginJoint() {
  if (joint_ >= config::JOINT_COUNT) {
    motion.stop();
    state_ = State::Complete;
    return;
  }
  if (config::HOME_PIN[joint_] < 0) {
    fail();
    return;
  }
  stage_started_ms_ = millis();
  if (switchTriggered(joint_)) {
    backoff_target_ = motion.position(joint_) -
        config::JOINT[joint_].home_direction * int32_t(config::JOINT[joint_].home_backoff_steps);
    motion.haltJoint(joint_);
    state_ = State::Backoff;
    motion.setVelocity(joint_, -config::JOINT[joint_].home_direction *
                                  config::JOINT[joint_].home_fast_steps_per_second);
  } else {
    state_ = State::SeekFast;
    motion.setVelocity(joint_, config::JOINT[joint_].home_direction *
                                  config::JOINT[joint_].home_fast_steps_per_second);
  }
}

void HomingController::nextJoint() {
  motion.haltJoint(joint_);
  motion.setPosition(joint_, config::JOINT[joint_].home_position_steps);
  motion.setHomed(joint_, true);
  ++joint_;
  beginJoint();
}

void HomingController::fail() {
  motion.stop();
  state_ = State::Failed;
}

void HomingController::update() {
  if (!active()) return;
  if (millis() - stage_started_ms_ > config::HOME_TIMEOUT_MS) {
    fail();
    return;
  }

  switch (state_) {
    case State::SeekFast:
      if (switchTriggered(joint_)) {
        backoff_target_ = motion.position(joint_) -
            config::JOINT[joint_].home_direction * int32_t(config::JOINT[joint_].home_backoff_steps);
        motion.haltJoint(joint_);
        motion.setVelocity(joint_, -config::JOINT[joint_].home_direction *
                                      config::JOINT[joint_].home_fast_steps_per_second);
        state_ = State::Backoff;
        stage_started_ms_ = millis();
      }
      break;

    case State::Backoff: {
      const int8_t away = -config::JOINT[joint_].home_direction;
      const bool reached = away > 0 ? motion.position(joint_) >= backoff_target_
                                    : motion.position(joint_) <= backoff_target_;
      if (reached && !switchTriggered(joint_)) {
        motion.haltJoint(joint_);
        motion.setVelocity(joint_, config::JOINT[joint_].home_direction *
                                      config::JOINT[joint_].home_slow_steps_per_second);
        state_ = State::SeekSlow;
        stage_started_ms_ = millis();
      }
      break;
    }

    case State::SeekSlow:
      if (switchTriggered(joint_)) nextJoint();
      break;

    default:
      break;
  }
}

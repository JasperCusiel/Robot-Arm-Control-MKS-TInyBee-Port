#pragma once

#include <Arduino.h>
#include <stdint.h>

class HomingController {
 public:
  void begin();
  void start();
  void cancel();
  void update();
  bool active() const { return state_ != State::Idle && state_ != State::Complete && state_ != State::Failed; }
  bool failed() const { return state_ == State::Failed; }

 private:
  enum class State : uint8_t { Idle, SeekFast, Backoff, SeekSlow, Complete, Failed };
  State state_ = State::Idle;
  uint8_t joint_ = 0;
  uint32_t stage_started_ms_ = 0;
  int32_t backoff_target_ = 0;

  bool switchTriggered(uint8_t joint) const;
  void beginJoint();
  void nextJoint();
  void fail();
};

extern HomingController homing;


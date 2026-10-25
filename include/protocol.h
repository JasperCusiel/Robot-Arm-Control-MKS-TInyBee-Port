#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "robot_config.h"

struct CommanderCommand {
  int32_t position[config::JOINT_COUNT]{};
  int32_t velocity[config::JOINT_COUNT]{};
  uint8_t command = 255;
  uint8_t affected_joint = 0;
  uint8_t io = 0;
  uint8_t timeout_ms = 0;
  int16_t gripper_position = 0;
  int16_t gripper_speed = 0;
  int16_t gripper_current = 0;
  uint8_t gripper_command = 0;
  uint8_t gripper_mode = 0;
  uint8_t gripper_id = 0;
};

struct RobotTelemetry {
  int32_t position[config::JOINT_COUNT]{};
  int32_t velocity[config::JOINT_COUNT]{};
  bool homed[config::JOINT_COUNT]{};
  bool input[2]{};
  bool output[2]{};
  bool estop = true;
  bool position_error[config::JOINT_COUNT]{};
  uint16_t command_interval_us = 0;
  uint8_t timeout_error = 0;
  uint8_t command_echo = 255;
};

class ParolProtocol {
 public:
  bool poll(Stream& stream, CommanderCommand& command);
  void send(Stream& stream, const RobotTelemetry& telemetry);

 private:
  enum class State : uint8_t { SeekHeader, ReadLength, ReadBody };
  State state_ = State::SeekHeader;
  uint8_t header_count_ = 0;
  uint8_t expected_length_ = 0;
  uint8_t body_count_ = 0;
  uint8_t body_[255]{};

  bool consume(uint8_t value, CommanderCommand& command);
  bool unpack(CommanderCommand& command) const;
  void reset();
};

int32_t decodeSigned24(const uint8_t* bytes);
int16_t decodeSigned16(const uint8_t* bytes);
void encodeSigned24(int32_t value, uint8_t* bytes);
void encodeSigned16(int16_t value, uint8_t* bytes);


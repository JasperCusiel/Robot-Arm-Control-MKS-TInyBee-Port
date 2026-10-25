#include "protocol.h"

#include <string.h>

namespace {
constexpr uint8_t kIncomingPayloadBytes = 50;
constexpr uint8_t kIncomingFrameBodyBytes = 52;  // payload + 0x01, 0x02
constexpr uint8_t kOutgoingFrameBodyBytes = 56;

uint8_t packBits(const bool bits[8]) {
  uint8_t value = 0;
  for (uint8_t i = 0; i < 8; ++i) value |= uint8_t(bits[i]) << (7U - i);
  return value;
}

void writeByte(Stream& stream, uint8_t value) { stream.write(value); }
}

int32_t decodeSigned24(const uint8_t* bytes) {
  int32_t value = (int32_t(bytes[0]) << 16) | (int32_t(bytes[1]) << 8) | bytes[2];
  if (value & 0x00800000L) value |= int32_t(0xFF000000UL);
  return value;
}

int16_t decodeSigned16(const uint8_t* bytes) {
  return int16_t((uint16_t(bytes[0]) << 8) | bytes[1]);
}

void encodeSigned24(int32_t value, uint8_t* bytes) {
  bytes[0] = uint8_t(value >> 16);
  bytes[1] = uint8_t(value >> 8);
  bytes[2] = uint8_t(value);
}

void encodeSigned16(int16_t value, uint8_t* bytes) {
  bytes[0] = uint8_t(value >> 8);
  bytes[1] = uint8_t(value);
}

void ParolProtocol::reset() {
  state_ = State::SeekHeader;
  header_count_ = 0;
  expected_length_ = 0;
  body_count_ = 0;
}

bool ParolProtocol::poll(Stream& stream, CommanderCommand& command) {
  bool received = false;
  while (stream.available() > 0) {
    const int value = stream.read();
    if (value >= 0 && consume(uint8_t(value), command)) received = true;
  }
  return received;
}

bool ParolProtocol::consume(uint8_t value, CommanderCommand& command) {
  switch (state_) {
    case State::SeekHeader:
      if (value == 0xFF) {
        if (++header_count_ == 3) state_ = State::ReadLength;
      } else {
        header_count_ = 0;
      }
      return false;

    case State::ReadLength:
      expected_length_ = value;
      body_count_ = 0;
      // Only the 52-byte Commander frame is valid. Rejecting anything else here
      // also guarantees body_ can never be overrun by a corrupt length byte.
      if (expected_length_ != kIncomingFrameBodyBytes) reset();
      else state_ = State::ReadBody;
      return false;

    case State::ReadBody:
      body_[body_count_++] = value;
      if (body_count_ < expected_length_) return false;
      {
        const bool framed = body_[expected_length_ - 2] == 0x01 &&
                            body_[expected_length_ - 1] == 0x02;
        const bool valid = framed && unpack(command);
        reset();
        return valid;
      }
  }
  reset();
  return false;
}

bool ParolProtocol::unpack(CommanderCommand& command) const {
  if (expected_length_ != kIncomingFrameBodyBytes) return false;
  for (uint8_t joint = 0; joint < config::JOINT_COUNT; ++joint)
    command.position[joint] = decodeSigned24(&body_[joint * 3]);
  for (uint8_t joint = 0; joint < config::JOINT_COUNT; ++joint)
    command.velocity[joint] = decodeSigned24(&body_[18 + joint * 3]);
  command.command = body_[36];
  command.affected_joint = body_[37];
  command.io = body_[38];
  command.timeout_ms = body_[39];
  command.gripper_position = decodeSigned16(&body_[40]);
  command.gripper_speed = decodeSigned16(&body_[42]);
  command.gripper_current = decodeSigned16(&body_[44]);
  command.gripper_command = body_[46];
  command.gripper_mode = body_[47];
  command.gripper_id = body_[48];
  // byte 49 is retained as the upstream placeholder CRC. Upstream firmware
  // does not validate it, so compatibility requires the same behaviour.
  return kIncomingPayloadBytes == 50;
}

void ParolProtocol::send(Stream& stream, const RobotTelemetry& telemetry) {
  writeByte(stream, 0xFF); writeByte(stream, 0xFF); writeByte(stream, 0xFF);
  writeByte(stream, kOutgoingFrameBodyBytes);

  uint8_t encoded[3];
  for (uint8_t i = 0; i < config::JOINT_COUNT; ++i) {
    encodeSigned24(telemetry.position[i], encoded);
    stream.write(encoded, 3);
  }
  for (uint8_t i = 0; i < config::JOINT_COUNT; ++i) {
    encodeSigned24(telemetry.velocity[i], encoded);
    stream.write(encoded, 3);
  }

  bool flags[8] = {};
  for (uint8_t i = 0; i < config::JOINT_COUNT; ++i) flags[i] = telemetry.homed[i];
  flags[6] = flags[7] = true;
  writeByte(stream, packBits(flags));

  memset(flags, 0, sizeof(flags));
  flags[0] = telemetry.input[0]; flags[1] = telemetry.input[1];
  flags[2] = telemetry.output[0]; flags[3] = telemetry.output[1];
  flags[4] = telemetry.estop; flags[5] = flags[6] = flags[7] = true;
  writeByte(stream, packBits(flags));

  flags[0] = flags[1] = flags[2] = flags[3] = flags[4] = flags[5] = false;
  flags[6] = flags[7] = true;
  writeByte(stream, packBits(flags));  // temperature errors unavailable

  for (uint8_t i = 0; i < config::JOINT_COUNT; ++i) flags[i] = telemetry.position_error[i];
  flags[6] = flags[7] = true;
  writeByte(stream, packBits(flags));

  encodeSigned16(int16_t(telemetry.command_interval_us), encoded);
  stream.write(encoded, 2);
  writeByte(stream, telemetry.timeout_error);
  writeByte(stream, telemetry.command_echo);

  // Gripper telemetry placeholder, matching the 56-byte upstream frame and its
  // idle defaults (speed 1, current 2, status 1, object detected 1).
  writeByte(stream, 0);                  // ID
  encodeSigned16(0, encoded); stream.write(encoded, 2);  // position
  encodeSigned16(1, encoded); stream.write(encoded, 2);  // speed
  encodeSigned16(2, encoded); stream.write(encoded, 2);  // current
  writeByte(stream, 1);                  // status
  writeByte(stream, 1);                  // object detection
  writeByte(stream, 212);                // upstream placeholder CRC
  writeByte(stream, 0x01); writeByte(stream, 0x02);
}


#pragma once

#include <Arduino.h>

#include "protocol.h"

class RobotController {
 public:
  void begin();
  void update();

 private:
  ParolProtocol protocol_;
  CommanderCommand command_;
  uint32_t last_packet_us_ = 0;
  uint32_t last_control_us_ = 0;
  uint8_t timeout_ms_ = 0;
  bool timeout_error_ = false;

  void handlePacket();
  void applyPositionFollower();
  void updateSafety();
  void fillTelemetry(RobotTelemetry& telemetry) const;
  bool estopActive() const;
  bool readOptionalInput(int pin) const;
};


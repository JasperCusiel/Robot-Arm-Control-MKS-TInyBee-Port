#include "controller.h"

#include <limits.h>
#include <math.h>

#include "commander_port.h"
#include "commander_units.h"
#include "homing.h"
#include "motion.h"
#include "robot_config.h"

namespace {
#if COMMANDER_ON_UART2
HardwareSerial& commanderPort = Serial2;
#else
HardwareSerial& commanderPort = Serial;
#endif

constexpr uint8_t CMD_HOME = 100;
constexpr uint8_t CMD_ENABLE = 101;
constexpr uint8_t CMD_DISABLE = 102;
constexpr uint8_t CMD_CLEAR_ERROR = 103;
constexpr uint8_t CMD_JOG = 123;
constexpr uint8_t CMD_POSITION = 156;
constexpr uint8_t CMD_IDLE = 255;

// PAROL Commander reports the command interval in 1.4222 us timer ticks.
constexpr float kTicksPerMicrosecond = 0.703125f;

// Upstream "disabled" (102) only stops accepting motion commands; the drivers
// stay energised and keep holding the arm. Mirror that here: when inhibited the
// drivers are left as they are, motion is stopped and jog/position are ignored
// until the next Enable (101). Kept at file scope so controller.h is untouched.
bool commands_inhibited = false;

// Latched while the GPIO E-stop input is active (only meaningful if enabled in
// robot_config.h). Used to detect the rising edge.
bool estop_latched = false;

// After a Home (100) the Commander's HomeCommand waits to SEE at least one joint
// report "not homed" before it waits for "all homed" (and otherwise sits for 20 s
// until its own timeout). So after Home the joints are reported un-homed for this
// many non-Home packets (~0.1 s at 100 Hz) and then flipped to homed.
constexpr uint8_t kHomeHoldPackets = 10;
uint8_t home_hold_packets = 0;

bool commandsAllowed() { return motion.enabled() && !commands_inhibited; }

// Called with the current E-stop level every time it is sampled.
void applyEstop(bool active) {
  if (!active) {
    estop_latched = false;
    return;
  }
  homing.cancel();
  motion.disable();  // stops motion and drops the driver enables
  commands_inhibited = true;
  home_hold_packets = 0;
  if (!estop_latched) {
    estop_latched = true;
    // Power was (or may have been) cut, so an open-loop arm can have moved.
    // Make Commander re-home (park the arm in standby, then press Home).
    for (uint8_t joint = 0; joint < config::JOINT_COUNT; ++joint) motion.setHomed(joint, false);
  }
}
}  // namespace

void RobotController::begin() {
#if COMMANDER_ON_UART2
  commanderPort.begin(config::COMMANDER_BAUD, SERIAL_8N1, COMMANDER_UART2_RX_PIN,
                      COMMANDER_UART2_TX_PIN);
#else
  commanderPort.begin(config::COMMANDER_BAUD);
#endif
  // Input-only GPIO34-39 have no internal pull-up.
  for (int pin : config::INPUT_PIN)
    if (pin >= 0) pinMode(pin, pin >= 34 ? INPUT : INPUT_PULLUP);
  for (int pin : config::OUTPUT_PIN) if (pin >= 0) pinMode(pin, OUTPUT);
  if (config::ESTOP_ENABLED) pinMode(config::ESTOP_PIN, INPUT_PULLUP);
  homing.begin();
  if (!motion.begin()) {
    // Fail safe: never enable the drivers if the I2S engine did not start.
    for (;;) delay(1000);
  }
  motion.disable();
  last_control_us_ = micros();
}

bool RobotController::estopActive() const {
  if (!config::ESTOP_ENABLED) return false;
  const bool level = digitalRead(config::ESTOP_PIN);
  return config::ESTOP_ACTIVE_LOW ? !level : level;
}

bool RobotController::readOptionalInput(int pin) const {
  return pin >= 0 ? digitalRead(pin) : false;
}

void RobotController::handlePacket() {
  const uint32_t now = micros();
  const uint32_t interval = now - last_packet_us_;
  last_packet_us_ = now;
  timeout_ms_ = command_.timeout_ms;

  // Commander packs OUT1/OUT2 as big-endian flag indices 2 and 3, i.e. bits 5 and 4.
  const bool outputs[2] = {bool(command_.io & 0x20), bool(command_.io & 0x10)};
  for (uint8_t i = 0; i < 2; ++i)
    if (config::OUTPUT_PIN[i] >= 0) digitalWrite(config::OUTPUT_PIN[i], outputs[i]);

  const bool estop = estopActive();
  applyEstop(estop);

  // Finish a pending Home: flip the joints to homed once the hold has elapsed.
  if (!estop && command_.command != CMD_HOME && home_hold_packets > 0 && --home_hold_packets == 0)
    for (uint8_t joint = 0; joint < config::JOINT_COUNT; ++joint) motion.setHomed(joint, true);

  if (!estop) {
    switch (command_.command) {
      case CMD_ENABLE:
        timeout_error_ = false;
        commands_inhibited = false;
        motion.enable();
        break;
      case CMD_DISABLE:
        // Match upstream: stop and ignore motion commands, but keep holding torque.
        homing.cancel();
        motion.stop();
        commands_inhibited = true;
        break;
      case CMD_CLEAR_ERROR:
        homing.cancel();
        motion.clearErrors();
        timeout_error_ = false;
        break;
      case CMD_HOME:
        // Homing bypass: do not move. The arm is assumed to be parked in the
        // PAROL6 standby pose, so report that pose (converted to this arm's
        // motor steps). Joints read un-homed for a short hold (see above) and
        // are then reported homed.
        //
        // The PAROL Commander never sends Enable (101) except to recover from an
        // E-stop, and the upstream firmware boots enabled, so Home also enables
        // the drivers here. Without this nothing would ever move.
        homing.cancel();
        motion.stop();
        for (uint8_t joint = 0; joint < config::JOINT_COUNT; ++joint) {
          motion.setPosition(joint, commander::toMotorSteps(joint, commander::STANDBY_STEPS[joint]));
          motion.setHomed(joint, false);
        }
        home_hold_packets = kHomeHoldPackets;
        timeout_error_ = false;
        commands_inhibited = false;
        motion.enable();
        break;
      case CMD_JOG:
        if (commandsAllowed() && !homing.active())
          for (uint8_t joint = 0; joint < config::JOINT_COUNT; ++joint)
            motion.setVelocity(joint, float(command_.velocity[joint]) * commander::unitScale(joint));
        break;
      case CMD_POSITION:
        if (commandsAllowed() && !homing.active()) applyPositionFollower();
        break;
      case CMD_IDLE:
      default:
        if (!homing.active()) motion.stop();
        break;
    }
  }

  RobotTelemetry telemetry;
  fillTelemetry(telemetry);
  const float ticks = float(interval) * kTicksPerMicrosecond;
  telemetry.command_interval_us = uint16_t(ticks > float(UINT16_MAX) ? UINT16_MAX : uint32_t(ticks));
  protocol_.send(commanderPort, telemetry);
}

void RobotController::applyPositionFollower() {
  for (uint8_t joint = 0; joint < config::JOINT_COUNT; ++joint) {
    // Commander positions/velocities are in Commander steps; convert to this
    // arm's motor steps before comparing with the motion engine.
    const float scale = commander::unitScale(joint);
    const int32_t target = commander::toMotorSteps(joint, command_.position[joint]);
    const int32_t error = target - motion.position(joint);
    // Preserve the upstream PAROL6 controller: average trajectory feed-forward
    // with the velocity required to remove the position error in 10 ms.
    const float requested = 0.5f * (float(command_.velocity[joint]) * scale + error / 0.010f);
    motion.setVelocity(joint, requested);
  }
}

void RobotController::updateSafety() {
  const bool estop = estopActive();
  applyEstop(estop);
  if (estop) return;
  if (timeout_ms_ > 0 && last_packet_us_ != 0 &&
      uint32_t(micros() - last_packet_us_) > uint32_t(timeout_ms_) * 1000UL) {
    // Upstream never enforced this timeout. Stop moving, but keep the drivers
    // energised so a gravity-loaded arm is not dropped. Enable (101) resumes.
    timeout_error_ = true;
    homing.cancel();
    motion.stop();
    commands_inhibited = true;
  }
}

void RobotController::fillTelemetry(RobotTelemetry& telemetry) const {
  for (uint8_t joint = 0; joint < config::JOINT_COUNT; ++joint) {
    telemetry.position[joint] = commander::toCommanderSteps(joint, motion.position(joint));
    telemetry.velocity[joint] = commander::toCommanderSteps(joint, motion.velocity(joint));
    telemetry.homed[joint] = motion.homed(joint);
    telemetry.position_error[joint] = motion.positionError(joint);
  }
  telemetry.input[0] = readOptionalInput(config::INPUT_PIN[0]);
  telemetry.input[1] = readOptionalInput(config::INPUT_PIN[1]);
  telemetry.output[0] = config::OUTPUT_PIN[0] >= 0 && digitalRead(config::OUTPUT_PIN[0]);
  telemetry.output[1] = config::OUTPUT_PIN[1] >= 0 && digitalRead(config::OUTPUT_PIN[1]);
  // 1 = E-stop released / healthy (upstream sends the raw NC switch level).
  // With no GPIO E-stop wired (power is cut in hardware) this is always 1.
  telemetry.estop = !estopActive();
  telemetry.timeout_error = timeout_error_ ? 1 : 0;
  telemetry.command_echo = command_.command;
}

void RobotController::update() {
  if (protocol_.poll(commanderPort, command_)) handlePacket();
  homing.update();
  updateSafety();

  const uint32_t interval = 1000000UL / config::CONTROL_RATE;
  const uint32_t now = micros();
  while (uint32_t(now - last_control_us_) >= interval) {
    last_control_us_ += interval;
    motion.controlTick();
  }
}

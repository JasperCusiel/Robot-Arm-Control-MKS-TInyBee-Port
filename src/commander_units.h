#pragma once
/*
 * Unit conversion between PAROL Commander "steps" and this arm's real motor steps.
 *
 * PAROL Commander hard-codes 200 full steps x 32 microsteps and the PAROL6
 * reduction ratios [6.4, 20, 20*38/42, 4, 4, 10] when it converts steps to
 * joint angles. Everything that crosses the serial link (positions, velocities,
 * the standby pose) is therefore in "Commander steps". Everything inside the
 * motion engine (positions, velocities, min/max travel in robot_config.h) is in
 * real motor steps for YOUR arm. This header holds the conversion factors.
 */
#include <math.h>
#include <stdint.h>

#include "robot_config.h"

static_assert(config::JOINT_COUNT == 6, "commander_units.h assumes six joints");

namespace commander {

constexpr float kBaseStepsPerDegree = 32.0f * 200.0f / 360.0f;

// What Commander assumes for each joint (PAROL6 geometry). Do not edit.
constexpr float STEPS_PER_DEGREE[6] = {
    kBaseStepsPerDegree * 6.4f,
    kBaseStepsPerDegree * 20.0f,
    kBaseStepsPerDegree * (20.0f * 38.0f / 42.0f),
    kBaseStepsPerDegree * 4.0f,
    kBaseStepsPerDegree * 4.0f,
    kBaseStepsPerDegree * 10.0f,
};

// PAROL6 standby pose in Commander steps: J1 +90, J2 -90, J3 180, J4 0, J5 0, J6 180 deg.
// The firmware reports this pose after the Home command because the arm is
// assumed to be parked in it.
constexpr int32_t STANDBY_STEPS[6] = {10240, -32000, 57905, 0, 0, 32000};

}  // namespace commander

namespace config {

// EDIT THIS: real motor steps per degree of joint motion on YOUR arm, i.e.
//   motor_full_steps_per_rev * microsteps * reduction / 360
// The defaults equal the PAROL6 values, which gives a scale of exactly 1.0 so
// behaviour is unchanged until you fill in your real numbers.
// robot_config.h minimum_steps / maximum_steps must use the same numbers:
//   steps = joint_angle_degrees (Commander convention, e.g. J2 standby = -90) * this value
constexpr float PHYSICAL_STEPS_PER_DEGREE[6] = {
    commander::STEPS_PER_DEGREE[0],  // J1
    commander::STEPS_PER_DEGREE[1],  // J2
    commander::STEPS_PER_DEGREE[2],  // J3
    commander::STEPS_PER_DEGREE[3],  // J4
    commander::STEPS_PER_DEGREE[4],  // J5
    commander::STEPS_PER_DEGREE[5],  // J6
};

}  // namespace config

namespace commander {

// motor steps per Commander step for this joint
inline float unitScale(uint8_t joint) {
  return config::PHYSICAL_STEPS_PER_DEGREE[joint] / STEPS_PER_DEGREE[joint];
}

inline int32_t toMotorSteps(uint8_t joint, int32_t commander_steps) {
  return int32_t(lroundf(float(commander_steps) * unitScale(joint)));
}

inline int32_t toCommanderSteps(uint8_t joint, int32_t motor_steps) {
  return int32_t(lroundf(float(motor_steps) / unitScale(joint)));
}

}  // namespace commander

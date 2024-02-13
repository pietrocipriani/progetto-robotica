#pragma once
#ifndef _UTIL_CONST_HPP_
#define _UTIL_CONST_HPP_

#include "planner.hpp"

namespace controller::util {

constexpr int gripper_joint_count = 2;

// Default homing configuration for the UR5 manipulator. Exported by `params.py`.
constexpr Scalar ur5_default_homing_config[] = {-0.32, -0.78, -2.56, -1.63, -1.57, 3.49};
constexpr double ur5_default_homing_gripper_pos = 0.0;
const Vector<6> ur5_default_homing_config_vec(ur5_default_homing_config);

constexpr Time dt = 0.01;
constexpr double frequency_hz = 1 / dt;
constexpr double gripper_speed = 0.8;
constexpr double theta6_speed = M_PI; // 0.25 turns/s, used only initially to reach a valid config

} // namespace controller::util

#endif // _UTIL_CONST_HPP_
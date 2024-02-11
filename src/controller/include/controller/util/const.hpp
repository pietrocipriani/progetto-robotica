#pragma once
#ifndef _UTIL_CONST_HPP_
#define _UTIL_CONST_HPP_

#include "planner.hpp"

namespace controller::util {

// Default homing configuration for the UR5 manipulator. Exported by `params.py`.
constexpr Scalar ur5_default_homing_config[] = {-0.32, -0.78, -2.56, -1.63, -1.57, 3.49};
constexpr double ur5_default_homing_gripper_pos = 0.0;
const Vector<6> ur5_default_homing_config_vec(ur5_default_homing_config);

} // namespace controller::util

#endif // _UTIL_CONST_HPP_
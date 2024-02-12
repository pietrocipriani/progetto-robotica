#pragma once
#ifndef _CONTROL_COMPOSITE_MOVEMENTS_HPP_
#define _CONTROL_COMPOSITE_MOVEMENTS_HPP_

#include "config_publisher.hpp"
#include "planner.hpp"
#include "model.hpp"

namespace controller::control {

void move_block(
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos,
	const planner::BlockMovement& movement
);

void go_in_homing_config(
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
);

} // namespace controller::control

#endif // _CONTROL_COMPOSITE_MOVEMENTS_HPP_
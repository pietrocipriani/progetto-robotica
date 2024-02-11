#pragma once
#ifndef _CONTROL_MOVE_BLOCK_HPP_
#define _CONTROL_MOVE_BLOCK_HPP_

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

} // namespace controller::control

#endif // _CONTROL_MOVE_BLOCK_HPP_
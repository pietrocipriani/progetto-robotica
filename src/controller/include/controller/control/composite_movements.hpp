#pragma once
#ifndef _CONTROL_COMPOSITE_MOVEMENTS_HPP_
#define _CONTROL_COMPOSITE_MOVEMENTS_HPP_

#include "config_publisher.hpp"
#include "planner.hpp"
#include "model.hpp"

namespace controller::control {

/**
 * @brief Makes sure that the theta6 parameter is in range (-pi, pi), which is a requirement
 * for planner we use. In particular, moves the sixth joint linearly to position 0.0, so that
 * it's in the middle and the configuration will be surely valid.
 * 
 * @param config_publisher where to publish robot joint configurations so that they are performed
 *                         in the simulation
 * @param configuration the current joints config stored inside, before the robot has been initialized;
 *                      will be changes to ensure valid initialization
 * @param prev_gripper_pos the current gripper position, will not be changed
 */
void ensure_valid_initial_config(
	control::ConfigPublisher& config_publisher,
	model::UR5::Configuration& configuration,
	double prev_gripper_pos
);

/**
 * @brief Performs the steps needed to move a block according to the provided movement, and
 * therefore handles:
 * - going above the block
 * - closing the gripper to get hold of the block
 * - picking up the block
 * - moving over to the target pad
 * - droping the block by reopening the gripper
 * 
 * The movements are planned using the planner and sent to the simulation config publisher.
 * 
 * @param config_publisher where to publish robot joint configurations so that they are performed
 *                         in the simulation
 * @param robot the robot with the current joints config stored inside
 * @param prev_gripper_pos the current gripper position
 * @param movement the movement to perform (assumes the `start` and `target` poses both refer to
 *                 the same block)
 */
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
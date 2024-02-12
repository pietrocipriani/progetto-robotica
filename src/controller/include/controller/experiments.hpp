#pragma once
#ifndef _EXPERIMENTS_HPP_
#define _EXPERIMENTS_HPP_

#include "planner.hpp"
#include "model.hpp"
#include "controller/control/config_publisher.hpp"
#include "controller/control/config_reader.hpp"
#include "controller/world/workspace.hpp"

#include <type_traits>

/// This file contains various experiments we have done along the way.
/// `full` represents the final experiment that actually
/// combines all of our work to recognize blocks positions and move
/// them to their target pads using kinematics and planning.
namespace controller::experiment {

/// The type of functions that run experiments.
using ExperimentFunc = void (*) (
    ros::NodeHandle&,
    world::Spawner&,
    world::Deleter&,
    control::ConfigPublisher&,
    model::UR5&,
    double&
);

/**
 * @brief This is the final experiment, that sets up a workspace, then waits for blocks to be
 * detected, then picks the blocks up until it has finished moving all blocks to their target pads.
 * 
 * @param node_handle the ROS node handle, used to get block positions from the `position_detection`
 *                    publisher 
 * @param spawner used to spawn blocks in the world
 * @param deleter used to delete blocks in the world
 * @param config_publisher used to publish configs to the robot's joint states topic, which means
 *                         that this object is used to send commands to the robot
 * @param robot the robot, already initialized in its current position
 * @param prev_gripper_pos the gripper position, already initialized to its current position
 */
void full(
	ros::NodeHandle& node_handle,
	world::Spawner& spawner,
	world::Deleter& deleter,
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
);

/**
 * @brief This experiment spawns one block of every type at a time at the center of the workspace,
 * and then moves it to the corresponding target block pad. No block detection is used, since the
 * spawn position of blocks is hardcoded.
 * 
 * @param node_handle the ROS node handle, used to get block positions from the `position_detection`
 *                    publisher 
 * @param spawner used to spawn blocks in the world
 * @param deleter used to delete blocks in the world
 * @param config_publisher used to publish configs to the robot's joint states topic, which means
 *                         that this object is used to send commands to the robot
 * @param robot the robot, already initialized in its current position
 * @param prev_gripper_pos the gripper position, already initialized to its current position
 */
void all_blocks(
	ros::NodeHandle& node_handle,
	world::Spawner& spawner,
	world::Deleter& deleter,
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
);

/**
 * @brief Spawns a few blocks in a few places and moves them to a few other places, one at a time.
 * Used to test whether kinematics and planning works well.
 * 
 * @param node_handle the ROS node handle, used to get block positions from the `position_detection`
 *                    publisher 
 * @param spawner used to spawn blocks in the world
 * @param deleter used to delete blocks in the world
 * @param config_publisher used to publish configs to the robot's joint states topic, which means
 *                         that this object is used to send commands to the robot
 * @param robot the robot, already initialized in its current position
 * @param prev_gripper_pos the gripper position, already initialized to its current position
 */
void selected_fixed_positions(
	ros::NodeHandle& node_handle,
	world::Spawner& spawner,
	world::Deleter& deleter,
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
);

/**
 * @brief Sets up the workspace with blocks placed randomly in unknown positions. It also spawns the
 * block pads and moves the robot in its homing config. The resulting workspace corresponds to the
 * starting workspace used by the `full` experiment, and can therefore be used for checking if block
 * detection works well via the external scripts (e.g. `inspect_detected_blocks.py` or
 * `precise_placement.py`).
 * 
 * @param node_handle the ROS node handle, used to get block positions from the `position_detection`
 *                    publisher 
 * @param spawner used to spawn blocks in the world
 * @param deleter used to delete blocks in the world
 * @param config_publisher used to publish configs to the robot's joint states topic, which means
 *                         that this object is used to send commands to the robot
 * @param robot the robot, already initialized in its current position
 * @param prev_gripper_pos the gripper position, already initialized to its current position
 */
void workspace(
	ros::NodeHandle& node_handle,
	world::Spawner& spawner,
	world::Deleter& deleter,
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
);

} // namespace controller::experiment

#endif // _EXPERIMENTS_HPP_
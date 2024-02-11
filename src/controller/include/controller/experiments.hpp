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

using ExperimentFunc = void (*) (
    ros::NodeHandle&,
    world::Spawner&,
    world::Deleter&,
    control::ConfigPublisher&,
    model::UR5&,
    double&
);

void full(
	ros::NodeHandle& node_handle,
	world::Spawner& spawner,
	world::Deleter& deleter,
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
);

void all_blocks(
	ros::NodeHandle& node_handle,
	world::Spawner& spawner,
	world::Deleter& deleter,
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
);

void selected_fixed_positions(
	ros::NodeHandle& node_handle,
	world::Spawner& spawner,
	world::Deleter& deleter,
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
);

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
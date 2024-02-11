#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <memory>
#include <string>
#include <stdexcept>
#include <random>

#include "planner.hpp"
#include "model.hpp"

#include "controller/world/spawner.hpp"
#include "controller/world/workspace.hpp"
#include "controller/control/config_publisher.hpp"
#include "controller/control/config_reader.hpp"
#include "controller/pos/position_receiver.hpp"
#include "controller/util/const.hpp"
using namespace controller;

constexpr Time dt = 0.005;
constexpr double frequency_hz = 1 / dt;
constexpr double gripper_speed = 0.8;

void move_block(
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos,
	const planner::BlockMovement& movement
) {
	const planner::Block block = movement.start.block;
	auto configs = planner::plan_movement(robot, movement, dt);
	ROS_INFO("Movement planned");

	const double open_gripper = planner::get_open_gripper_pos(block);
	const double closed_gripper = planner::get_closed_gripper_pos(block);

	if (open_gripper > prev_gripper_pos) {
		// if picking up the next block requires a more open position, open the gripper right away
		config_publisher.publish_gripper_sequence(robot.config, prev_gripper_pos, open_gripper, gripper_speed, frequency_hz);
		prev_gripper_pos = open_gripper;
	} // otherwise keep prev_gripper_pos to make a single closing movement later

	config_publisher.publish_config_sequence(configs.lazy_picking, open_gripper, frequency_hz);
	config_publisher.publish_gripper_sequence(robot.config, prev_gripper_pos, closed_gripper, gripper_speed, frequency_hz);
	ros::Duration(1, 0).sleep();

	config_publisher.publish_config_sequence(configs.lazy_dropping, closed_gripper, frequency_hz);
	config_publisher.publish_gripper_sequence(robot.config, closed_gripper, open_gripper, gripper_speed, frequency_hz);
	prev_gripper_pos = open_gripper;

	ROS_INFO("Finished");
}

void main_selected_fixed_positions(
	ros::NodeHandle& node_handle,
	world::Spawner& spawner,
	world::Deleter& deleter,
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
) {
	std::vector<planner::BlockPose> poses{
		planner::BlockPose(planner::Block::B_4x1_L, 0.6, 0.7, 2),
		planner::BlockPose::pad_pose(planner::Block::B_4x1_L),
		planner::BlockPose(planner::Block::B_3x1_U, 0.5, 0.5, 0.5),
		planner::BlockPose::pad_pose(planner::Block::B_3x1_U),
		planner::BlockPose(planner::Block::B_2x1_L, 0.1, 0.4, 4),
		planner::BlockPose::pad_pose(planner::Block::B_2x1_L),
	};

	for (int i=0; i<poses.size()-1; i += 2) {
		const planner::Block block = poses[i].block;

		spawner.spawn_block(block,
			poses[i].pose.linear().x(), poses[i].pose.linear().y(),
			poses[i].pose.angular()[0], false,
			util::Color{255, 0, 0, 255}, true);

		const planner::BlockMovement movement{
			poses[i],
			poses[i+1]
		};

		move_block(config_publisher, robot, prev_gripper_pos, movement);
	}
}

void main_all_blocks(
	ros::NodeHandle& node_handle,
	world::Spawner& spawner,
	world::Deleter& deleter,
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
) {
	world::clear_workspace(deleter);
	world::spawn_missing_pads(spawner);

	constexpr double x=0.5, y=0.5, angle=0;
	for (const planner::Block block : planner::all_blocks) {
		spawner.spawn_block(block, x, y, angle, false,
			util::gen_bright_color(), false);

		const planner::BlockMovement movement{
			planner::BlockPose(block, x, y, angle),
			planner::BlockPose::pad_pose(block)
		};

		move_block(config_publisher, robot, prev_gripper_pos, movement);
	}
}

void main_workspace(
	ros::NodeHandle& node_handle,
	world::Spawner& spawner,
	world::Deleter& deleter,
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
) {
	auto configs = planner::plan_movement(
		robot,
		kinematics::direct(
			robot,
			model::UR5::Configuration(util::ur5_default_homing_config_vec)
		),
		dt
	);
	config_publisher.publish_config_sequence(configs, prev_gripper_pos, frequency_hz);
	ROS_INFO("Homing config reached");

	//world::clear_workspace(deleter);
	world::setup_workspace(spawner, true);
	ROS_INFO("Workspace ready");
}

void main_real(
	ros::NodeHandle& node_handle,
	world::Spawner& spawner,
	world::Deleter& deleter,
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
) {
	auto configs = planner::plan_movement(
		robot,
		kinematics::direct(
			robot,
			model::UR5::Configuration(util::ur5_default_homing_config_vec)
		),
		dt
	);
	config_publisher.publish_config_sequence(configs, prev_gripper_pos, frequency_hz);
	ROS_INFO("Homing config reached");

	world::clear_workspace(deleter);
	world::setup_workspace(spawner, true);
	ROS_INFO("Workspace ready");

	std::vector<position_detection::BlockPosition> blocks =
		pos::wait_for_new_block_positions(node_handle).blocks;
	sort(blocks.begin(), blocks.end(), [](const auto& a, const auto& b) { return a.confidence > b.confidence; });

	for (const position_detection::BlockPosition block_pos : blocks) {
		planner::Block block = planner::Block::B_1x1_H;
		for (const planner::Block block_hypothesis : planner::all_blocks) {
			if (planner::get_name(block_hypothesis) == block_pos.block_type) {
				block = block_hypothesis;
				break;
			}
		}
		auto [x, y, z] = block_pos.point;
		double angle = block_pos.angle;

		const planner::BlockMovement movement{
			planner::BlockPose(block, x, y, angle),
			planner::BlockPose::pad_pose(block)
		};

		ROS_INFO("Picking up block %s x=%.2f y=%.2f z=%.2f angle=%.2f conf=%.4f",
			block_pos.block_type.c_str(), x, y, z, angle, block_pos.confidence);
		move_block(config_publisher, robot, prev_gripper_pos, movement);
	}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");
    if (argc != 1) {
        ROS_WARN("Usage: controller");
        return 1;
    }
	ROS_INFO("Initialized controller");

    ros::NodeHandle n;
	world::Spawner spawner{n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model")};
	world::Deleter deleter{n.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model")};
	control::ConfigPublisher config_publisher{
		n.advertise<std_msgs::Float64MultiArray>(
			"/ur5/joint_group_pos_controller/command", 1),
		/* gripper_joint_count = */ 2,
	};
	ROS_INFO("Created clients and publisher");

	auto [initial_config, prev_gripper_pos] = control::joint_state_to_config(
		ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states", n));
	model::UR5 robot{initial_config};
	ROS_INFO("created robot with initial config");

	main_real(n, spawner, deleter, config_publisher, robot, prev_gripper_pos);
}
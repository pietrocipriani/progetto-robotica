#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <memory>
#include <string>
#include <stdexcept>
#include <random>

#include "planner.hpp"
#include "model.hpp"

#include "controller/world/block_spawner.hpp"
#include "controller/control/config_publisher.hpp"
#include "controller/control/config_reader.hpp"
using namespace controller;

constexpr Time dt = 0.01;
constexpr double frequency_hz = 1 / dt;
constexpr double gripper_speed = 0.8;

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");
    if (argc != 1) {
        ROS_INFO("usage: controller");
        return 1;
    }
	ROS_INFO("initialized controller");

    ros::NodeHandle n;
	control::ConfigPublisher config_publisher{
		n.advertise<std_msgs::Float64MultiArray>(
			"/ur5/joint_group_pos_controller/command", 1),
		2,
	};
	world::BlockSpawner block_spawner{n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model")};
	ROS_INFO("created publisher and client");

	auto [prev_config, prev_gripper_pos] = control::joint_state_to_config(
		ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states", n));
	model::UR5 robot{prev_config};


	std::vector<planner::BlockPose> poses{
		planner::BlockPose(planner::Block::B_1x1_H, 0.5, 0.5, 0.5),
		planner::BlockPose(planner::Block::B_1x1_H, 0.8, 0.6, 1),
		planner::BlockPose(planner::Block::B_4x1_H, 0.1, 0.4, 4),
		planner::BlockPose(planner::Block::B_4x1_H, 0.8, 0.4, 5),
		planner::BlockPose(planner::Block::B_2x2_U, 0.6, 0.7, 2),
		planner::BlockPose(planner::Block::B_2x2_U, 0.2, 0.6, 3),
	};


	for (int i=0; i<poses.size()-1; i += 2) {
		block_spawner.spawn_block(poses[i].block,
			poses[i].pose.linear().x(), poses[i].pose.linear().y(),
			poses[i].pose.angular()[0], false,
			util::Color{255, 0, 0, 255});

		const planner::BlockMovement movement{
			poses[i],
			poses[i+1]
		};

		auto configs = planner::plan_movement(robot, movement, dt);
		ROS_INFO("Movement planned");


		const double open_gripper = planner::get_open_gripper_pos(poses[i].block);
		const double closed_gripper = planner::get_closed_gripper_pos(poses[i].block);

		if (open_gripper > prev_gripper_pos) {
			// if picking up the next block requires a more open position, open the gripper right away
			config_publisher.publish_gripper_sequence(prev_config, prev_gripper_pos, open_gripper, gripper_speed, frequency_hz);
			prev_gripper_pos = open_gripper;
		} // otherwise keep prev_gripper_pos to make a single closing movement later

		auto interm_config = config_publisher.publish_config_sequence(configs.lazy_picking, open_gripper, frequency_hz);
		config_publisher.publish_gripper_sequence(interm_config, prev_gripper_pos, closed_gripper, gripper_speed, frequency_hz);

		prev_config = config_publisher.publish_config_sequence(configs.lazy_dropping, closed_gripper, frequency_hz);
		config_publisher.publish_gripper_sequence(prev_config, closed_gripper, open_gripper, gripper_speed, frequency_hz);
		prev_gripper_pos = open_gripper;

		ROS_INFO("Finished");
	}
}
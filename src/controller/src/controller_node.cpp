#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <memory>
#include <string>
#include <stdexcept>
#include <random>

#include "planner.hpp"
#include "model.hpp"

#include "controller/world/block_spawner.hpp"
#include "controller/control/config_publisher.hpp"
using namespace controller;

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

	constexpr Time dt = 0.01;
	constexpr double frequency_hz = 1 / dt;
	constexpr double gripper_speed = 0.8;
	model::UR5 robot;

	std::vector<planner::BlockPose> poses{
		planner::BlockPose(planner::Block::B_4x1_L, 0.5, 0.5, 0.5),
		planner::BlockPose(planner::Block::B_4x1_L, 0.8, 0.6, 1),
		planner::BlockPose(planner::Block::B_2x2_H, 0.6, 0.7, 2),
		planner::BlockPose(planner::Block::B_2x2_H, 0.2, 0.6, 3),
		planner::BlockPose(planner::Block::B_1x1_H, 0.1, 0.4, 4),
		planner::BlockPose(planner::Block::B_1x1_H, 0.8, 0.4, 5),
	};

	// for (int i=0; i<poses.size()-1; i += 1) {
	// 	spawn_block(client, poses[i].pose.linear().x(), poses[i].pose.linear().y());
	// }
	// return 0;

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

		auto cfg1 = config_publisher.publish_config_sequence(configs.lazy_picking, open_gripper, frequency_hz);
		config_publisher.publish_gripper_sequence(cfg1, open_gripper, closed_gripper, gripper_speed, frequency_hz);
		auto cfg2 = config_publisher.publish_config_sequence(configs.lazy_dropping, closed_gripper, frequency_hz);
		config_publisher.publish_gripper_sequence(cfg2, closed_gripper, open_gripper, gripper_speed, frequency_hz);
		ROS_INFO("Finished");
	}
}
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
	constexpr double scale = 1;
	model::UR5 robot;

	std::vector<planner::BlockPose> poses{
		planner::BlockPose(0.5, 0.5, 0, 0),
		planner::BlockPose(0.8, 0.6, 0, 0),
		planner::BlockPose(0.6, 0.7, 0, 0),
		planner::BlockPose(0.2, 0.6, 0, 0),
		planner::BlockPose(0.1, 0.4, 0, 0),
		planner::BlockPose(0.8, 0.4, 0, 0),
	};

	// for (int i=0; i<poses.size()-1; i += 1) {
	// 	spawn_block(client, poses[i].pose.linear().x(), poses[i].pose.linear().y());
	// }
	// return 0;

	for (int i=0; i<poses.size()-1; i += 2) {
		if (i%2 == 0) {
			block_spawner.spawn_block(world::BlockType::B_1x1_H,
				poses[i].pose.linear().x(), poses[i].pose.linear().y(),
				0, false,
				util::Color{255, 0, 0, 255});
		}

		const planner::BlockMovement movement{
			poses[i],
			poses[i+1]
		};

		auto configs = planner::plan_movement(robot, movement, dt);
		ROS_INFO("Movement planned");

		config_publisher.publish_config_sequence(configs.lazy_picking, 0, scale / dt);
		config_publisher.publish_config_sequence(configs.lazy_dropping, 2.6, scale / dt);
		ROS_INFO("Finished");
	}
}
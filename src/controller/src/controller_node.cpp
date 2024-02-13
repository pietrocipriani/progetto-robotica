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
#include "controller/control/position_receiver.hpp"
#include "controller/control/composite_movements.hpp"
#include "controller/experiments.hpp"
#include "controller/util/const.hpp"
using namespace controller;


int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");
	experiment::ExperimentFunc exp = experiment::full;
    if (argc == 2) {
		if (argv[1] == std::string("full")) {
			exp = experiment::full;
		} else if (argv[1] == std::string("all_blocks")) {
			exp = experiment::all_blocks;
		} else if (argv[1] == std::string("selected_fixed_positions")) {
			exp = experiment::selected_fixed_positions;
		} else if (argv[1] == std::string("workspace")) {
			exp = experiment::workspace;
		} else {
        	ROS_ERROR("Unknown experiment name: %s", argv[1]);
        	return 2;
		}
    } else if (argc > 2) {
        ROS_ERROR("Usage: controller [full|all_blocks|selected_fixed_positions|workspace]");
        return 1;
	}
	ROS_INFO("Initialized controller");

    ros::NodeHandle n;
	world::Spawner spawner{n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model")};
	world::Deleter deleter{n.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model")};
	control::ConfigPublisher config_publisher{
		n.advertise<std_msgs::Float64MultiArray>(
			"/ur5/joint_group_pos_controller/command", 1),
		util::gripper_joint_count,
	};
	ROS_INFO("Created clients and publisher");

	auto [initial_config, prev_gripper_pos] = control::joint_state_to_config(
		ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states", n));
	control::ensure_valid_initial_config(config_publisher, initial_config, prev_gripper_pos);
	model::UR5 robot{initial_config};
	ROS_INFO("Created robot with initial config");

	exp(n, spawner, deleter, config_publisher, robot, prev_gripper_pos);
}
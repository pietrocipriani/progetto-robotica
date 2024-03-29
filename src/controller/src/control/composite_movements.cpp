#include "controller/control/composite_movements.hpp"

#include "controller/util/const.hpp"
#include "controller/util/string.hpp"

namespace controller::control {

void ensure_valid_initial_config(
	control::ConfigPublisher& config_publisher,
	model::UR5::Configuration& configuration,
	double prev_gripper_pos
) {
	ROS_INFO("Starting to ensure valid initial config, config = %s",
		util::config_to_string(configuration, prev_gripper_pos).c_str());

	const Scalar initial = configuration[5];
	const int steps = std::abs(initial) / util::theta6_speed * util::frequency_hz;

	ros::Rate rate(util::frequency_hz);
    for (int i = 0; i < steps; ++i) {
		configuration[5] = initial * (steps - i) / steps;
        config_publisher.publish_config(configuration, prev_gripper_pos);
		rate.sleep();
    }

	configuration[5] = 0.0;
    config_publisher.publish_config(configuration, prev_gripper_pos);

	ROS_INFO("Finished ensuring valid initial config, config = %s",
		util::config_to_string(configuration, prev_gripper_pos).c_str());
}

void move_block(
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos,
	const planner::BlockMovement& movement
) {
	try {
		const planner::Block block = movement.start.block;
		const char * block_name = planner::get_name(block);
		ROS_INFO("Starting to pick up block %s, config = %s", block_name,
			util::config_to_string(robot.config, prev_gripper_pos).c_str());

		auto configs = planner::plan_movement(robot, movement, util::dt);

		const double open_gripper = planner::get_open_gripper_pos(block);
		const double closed_gripper = planner::get_closed_gripper_pos(block);

		if (open_gripper > prev_gripper_pos) {
			// if picking up the next block requires a more open position, open the gripper right away
			config_publisher.publish_gripper_sequence(robot.config, prev_gripper_pos, open_gripper, util::gripper_speed, util::frequency_hz);
			prev_gripper_pos = open_gripper;
		} // otherwise keep prev_gripper_pos to make a single closing movement later

		config_publisher.publish_config_sequence(configs.lazy_picking, open_gripper, util::frequency_hz);
		config_publisher.publish_gripper_sequence(robot.config, prev_gripper_pos, closed_gripper, util::gripper_speed, util::frequency_hz);
		ros::Duration(1, 0).sleep();

		config_publisher.publish_config_sequence(configs.lazy_dropping, closed_gripper, util::frequency_hz);
		config_publisher.publish_gripper_sequence(robot.config, closed_gripper, open_gripper, util::gripper_speed, util::frequency_hz);
		prev_gripper_pos = open_gripper;

		ROS_INFO("Finished picking up block %s, config = %s", block_name,
			util::config_to_string(robot.config,prev_gripper_pos).c_str());
	} catch (std::exception e) {
		ROS_ERROR("Couldn't plan block movement, exception was thrown: %s", e.what());
	}
}

void go_in_homing_config(
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
) {
	ROS_INFO("Starting to go to homing config, config = %s",
		util::config_to_string(robot.config, prev_gripper_pos).c_str());

	auto configs = planner::plan_movement(
		robot,
		kinematics::direct(
			robot,
			model::UR5::Configuration(util::ur5_default_homing_config_vec)
		),
		util::dt
	);

	config_publisher.publish_config_sequence(configs, prev_gripper_pos, util::frequency_hz);

	ROS_INFO("Finished going to homing config, config = %s",
		util::config_to_string(robot.config, prev_gripper_pos).c_str());
}

} // namespace controller::control
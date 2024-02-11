#include "controller/control/move_block.hpp"

#include "controller/util/const.hpp"

namespace controller::control {

void move_block(
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos,
	const planner::BlockMovement& movement
) {
	const planner::Block block = movement.start.block;
	auto configs = planner::plan_movement(robot, movement, util::dt);
	ROS_INFO("Movement planned");

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

	ROS_INFO("Finished");
}

} // namespace controller::control
#include "controller/experiments.hpp"

#include "controller/util/const.hpp"
#include "controller/control/position_receiver.hpp"
#include "controller/control/composite_movements.hpp"

namespace controller::experiment {

void full(
	ros::NodeHandle& node_handle,
	world::Spawner& spawner,
	world::Deleter& deleter,
	control::ConfigPublisher& config_publisher,
	model::UR5& robot,
	double& prev_gripper_pos
) {
	world::clear_workspace(deleter);
	world::setup_workspace(spawner, true);
	ROS_INFO("Workspace ready");

	// do not "look only once", but try looking again after at most max_count blocks,
	// and gradually lower the confidence threshold
	for (auto [min_confidence, max_count] : std::initializer_list<std::pair<double, size_t>>{
			{0.8, 5}, {0.7, 4}, {0.6, 2}, {0.2, 2}, {0.2, 2}, {0.2, 2}, {0.0, 11}}) {
		
		// move the robot arm away from the camera, so block positions can be recognized better
		control::go_in_homing_config(config_publisher, robot, prev_gripper_pos);
		auto blocks = control::wait_for_new_block_positions(node_handle);
		auto movements = control::filter_map_movements_to_pads(blocks, min_confidence, max_count);

		// the movements we have here passed our confidence threshold, so we can execute them
		for (auto&& movement : movements) {
			control::move_block(config_publisher, robot, prev_gripper_pos, movement);
		}
	}
}

void all_blocks(
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

		control::move_block(config_publisher, robot, prev_gripper_pos, movement);
	}
}

void selected_fixed_positions(
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

		control::move_block(config_publisher, robot, prev_gripper_pos, movement);
	}
}

void workspace(
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
		util::dt
	);
	config_publisher.publish_config_sequence(configs, prev_gripper_pos, util::frequency_hz);
	ROS_INFO("Homing config reached");

	//world::clear_workspace(deleter);
	world::setup_workspace(spawner, true);
	ROS_INFO("Workspace ready");
}

} // namespace controller::experiments
#include "controller/control/config_reader.hpp"

#include "controller/util/const.hpp"

namespace controller::control {

std::pair<model::UR5::Configuration, double> joint_state_to_config(
    sensor_msgs::JointState::ConstPtr joint_state
) {
    Vector<6> config = util::ur5_default_homing_config_vec;
	double gripper_pos = util::ur5_default_homing_gripper_pos;
	for (int i=0; i<joint_state->name.size(); ++i) {
		const auto& name = joint_state->name[i];
		const auto& position = joint_state->position[i];
		if (name == "elbow_joint") {
			config[2] = position;
		} else if (name == "hand_1_joint") {
			gripper_pos = position;
		} else if (name == "shoulder_lift_joint") {
			config[1] = position;
		} else if (name == "shoulder_pan_joint") {
			config[0] = position;
		} else if (name == "wrist_1_joint") {
			config[3] = position;
		} else if (name == "wrist_2_joint") {
			config[4] = position;
		} else if (name == "wrist_3_joint") {
			config[5] = position;
		}
	}

    return {model::UR5::Configuration{config}, gripper_pos};
}

} // namespace controller::control
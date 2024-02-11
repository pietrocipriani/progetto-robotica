#include "controller/control/config_publisher.hpp"

namespace controller::control {

std_msgs::Float64MultiArray ConfigPublisher::config_to_ros(
    const model::UR5::Configuration& config,
    double gripper_pos
) {
	std_msgs::Float64MultiArray res;
	res.data = std::vector<double>(model::UR5::dof + gripper_joint_count, gripper_pos);

	const auto& v = config.vector().data();
    std::copy(v, v + model::UR5::dof, res.data.begin());

	return res;
}


ConfigPublisher::ConfigPublisher(ros::Publisher&& _publisher, size_t _gripper_joint_count)
    : publisher{_publisher}, gripper_joint_count{_gripper_joint_count} {};


void ConfigPublisher::publish_config(
    const model::UR5::Configuration& config,
    double gripper_pos
) {
    std_msgs::Float64MultiArray msg = config_to_ros(config, gripper_pos);

    std::stringstream ss;
    std::copy(msg.data.begin(), msg.data.end(), std::ostream_iterator<double>(ss, " "));
	ROS_DEBUG("Publishing config: %s", ss.str().c_str());

    publisher.publish(msg);
}

void ConfigPublisher::publish_gripper_sequence(
    const model::UR5::Configuration& lastConfig,
    double gripper_pos_beg,
    double gripper_pos_end,
    double gripper_speed,
    double frequency_hz
) {
    const int steps = std::abs(gripper_pos_end - gripper_pos_beg) / gripper_speed * frequency_hz;

	ros::Rate rate(frequency_hz);
    for (int i = 0; i < steps; ++i) {
        publish_config(lastConfig, (gripper_pos_beg * (steps - i) + gripper_pos_end * i) / steps);
		rate.sleep();
    }

    publish_config(lastConfig, gripper_pos_end);
}

model::UR5::Configuration ConfigPublisher::publish_config_sequence(
    planner::MovementSequence::ConfigGenerator& configs,
    double gripper_pos,
    double frequency_hz
) {
    if (configs.begin() == configs.end()) {
        // should never be reached!
        ROS_ERROR("Empty queue passed to publish_config_sequence");
    }

	ros::Rate rate(frequency_hz);
    model::UR5::Configuration last(model::UR5::Configuration::Base{});

	for (auto config : configs) {
		publish_config(config, gripper_pos);
        std::swap(config, last);
		rate.sleep();
	}

    return last;
}

} // namespace controller::control
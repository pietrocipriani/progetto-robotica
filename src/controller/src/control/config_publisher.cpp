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
    ss << "Publishing config: ";
    std::copy(msg.data.begin(), msg.data.end(), std::ostream_iterator<double>(ss, " "));
    ss << std::endl;
	ROS_INFO_STREAM(ss.str());

    publisher.publish(msg);
}


model::UR5::Configuration ConfigPublisher::publish_config_sequence(
    planner::MovementSequence::ConfigSequence& queue,
    double gripper_pos,
    double frequencyHz
) {
	ros::Rate rate(frequencyHz);
	while (!queue.empty()) {
        const auto& config = queue.front();
		publisher.publish(config_to_ros(config, gripper_pos));

        if (queue.size() == 1) {
            return config;
        }

		queue.pop();
		rate.sleep();
	}

    // should never be reached!
    ROS_ERROR("Empty queue passed to publish_config_sequence");
    return model::UR5::Configuration(model::UR5::Configuration::Base{});
}

} // namespace controller::control
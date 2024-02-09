#pragma once
#ifndef _CONTROL_CONGIG_PUBLISHER_HPP_
#define _CONTROL_CONGIG_PUBLISHER_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "planner.hpp"
#include "model.hpp"

namespace controller::control {

class ConfigPublisher {
    ros::Publisher publisher;
    const size_t gripper_joint_count;

    std_msgs::Float64MultiArray config_to_ros(
        const model::UR5::Configuration& config,
        double gripper_pos
    );

public:
    ConfigPublisher(ros::Publisher&& publisher, size_t gripper_joint_count);

    void publish_config(
        const model::UR5::Configuration& config,
        double gripper_pos
    );

    /**
     * @return the last configuration obtained from the config sequence
    */
    model::UR5::Configuration publish_config_sequence(
        planner::MovementSequence::ConfigSequence& queue,
        double gripper_pos,
        double frequency_hz
    );
};

} // namespace controller::control

#endif // _CONTROL_CONGIG_PUBLISHER_HPP_
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

    void publish_gripper_sequence(
        const model::UR5::Configuration& lastConfig,
        double gripper_pos_beg,
        double gripper_pos_end,
        double gripper_speed, // [pos]/s
        double frequency_hz
    );

    /**
     * @return the last configuration obtained from the config sequence
    */
    model::UR5::Configuration publish_config_sequence(
        planner::MovementSequence::ConfigGenerator& configs,
        double gripper_pos,
        double frequency_hz
    );
};

} // namespace controller::control

#endif // _CONTROL_CONGIG_PUBLISHER_HPP_
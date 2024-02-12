#pragma once
#ifndef _CONTROL_CONGIG_PUBLISHER_HPP_
#define _CONTROL_CONGIG_PUBLISHER_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "planner.hpp"
#include "model.hpp"

namespace controller::control {

/**
 * @brief Publishes messages of type `std_msgs::Float64MultiArray` to the
 * "/ur5/joint_group_pos_controller/command" topic, to make the robot perform movements.
 */
class ConfigPublisher {
    ros::Publisher publisher;
    const size_t gripper_joint_count;

    /**
     * @brief Converts a robot config to a message of type "std_msgs/Float64MultiArray".
     * 
     * @param config the joint config to convert
     * @param gripper_pos the gripper position
     * @return std_msgs::Float64MultiArray the converted array, with 8 elements corresponding to
     *         the 6 joints of the robotic arm + 2 joints for the gripper 
     */
    std_msgs::Float64MultiArray config_to_ros(
        const model::UR5::Configuration& config,
        double gripper_pos
    );

public:
    /**
     * @brief Construct a new Config Publisher object
     * 
     * @param publisher the ROS publisher to send data to
     * @param gripper_joint_count the number of gripper joints, whose position is to be put at the
     *                            end of the published arrays with duplicated this amount of times
     */
    ConfigPublisher(ros::Publisher&& publisher, size_t gripper_joint_count);

    /**
     * @brief Puslishes a config directly to the robot, to make it perform a movement.
     * 
     * @param config the robot joint config to publish
     * @param gripper_pos the position of the gripper joints
     */
    void publish_config(
        const model::UR5::Configuration& config,
        double gripper_pos
    );

    /**
     * @brief Opens or closes the gripper gradually, by keeping the robot joints fixed and only
     * moving the gripper joints. The movement is linear, one step at a time, and takes some time.
     * 
     * @param last_config the robot joint config, that will remain fixed throughout the
     *                    open/closing motion
     * @param gripper_pos_beg the current gripper joints position
     * @param gripper_pos_end the target gripper joints position
     * @param gripper_speed the speed of the gripper, in [pos]/s (probably rad/s)
     * @param frequency_hz every how often to publish a config
     */
    void publish_gripper_sequence(
        const model::UR5::Configuration& last_config,
        double gripper_pos_beg,
        double gripper_pos_end,
        double gripper_speed, // [pos]/s
        double frequency_hz
    );

    /**
     * @brief Publishes the provided config sequence one step at a time, and returns the last
     * provided config, which will be the new robot configuration after the movement. The gripper
     * joints position will remain fixed throughout the movement.
     * 
     * @param configs the lazy config generator providing a sequence of configs
     * @param gripper_pos the gripper joints position, that will remain fixed throughout the motion
     * @param frequency_hz every how often to publish a config
     * @return model::UR5::Configuration the last configuration obtained from the config sequence
     */
    model::UR5::Configuration publish_config_sequence(
        planner::MovementSequence::ConfigGenerator& configs,
        double gripper_pos,
        double frequency_hz
    );
};

} // namespace controller::control

#endif // _CONTROL_CONGIG_PUBLISHER_HPP_
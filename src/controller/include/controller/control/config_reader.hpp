#pragma once
#ifndef _CONTROL_CONFIG_READER_HPP_
#define _CONTROL_CONFIG_READER_HPP_

#include <sensor_msgs/JointState.h>
#include <utility>
#include "model.hpp"

namespace controller::control {

/**
 * @brief converts a joint state message obtained from sensors on the robot to the robot
 * configuration format used internally. 
 * 
 * @param joint_state the `sensor_msgs/JointState` message to convert
 * @return std::pair<model::UR5::Configuration,double> the converted robot configuration,
 *         and the position of the gripper
 */
std::pair<model::UR5::Configuration, double> joint_state_to_config(
    sensor_msgs::JointState::ConstPtr joint_state
);

} // namespace controller::control

#endif // _CONTROL_CONFIG_READER_HPP_
#pragma once
#ifndef _CONTROL_CONFIG_READER_HPP_
#define _CONTROL_CONFIG_READER_HPP_

#include <sensor_msgs/JointState.h>
#include <utility>
#include "model.hpp"

namespace controller::control {

std::pair<model::UR5::Configuration, double> joint_state_to_config(
    sensor_msgs::JointState::ConstPtr joint_state
);

} // namespace controller::control

#endif // _CONTROL_CONFIG_READER_HPP_
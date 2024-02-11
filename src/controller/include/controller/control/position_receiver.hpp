#pragma once
#ifndef _CONTROL_POSITION_RECEIVER_HPP_
#define _CONTROL_POSITION_RECEIVER_HPP_

#include <ros/ros.h>
#include "position_detection/BlockPositions.h"

namespace controller::control {

std::vector<position_detection::BlockPosition> wait_for_new_block_positions(ros::NodeHandle& node_handle);

} // namespace controller::control

#endif // _CONTROL_POSITION_RECEIVER_HPP_
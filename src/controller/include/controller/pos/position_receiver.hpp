#pragma once
#ifndef _POS_POSITION_RECEIVER_HPP_
#define _POS_POSITION_RECEIVER_HPP_

#include <ros/ros.h>
#include "position_detection/BlockPositions.h"

namespace controller::pos {

position_detection::BlockPositions wait_for_new_block_positions(ros::NodeHandle& node_handle);

} // namespace controller::receive

#endif // _POS_POSITION_RECEIVER_HPP_
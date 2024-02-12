#pragma once
#ifndef _CONTROL_POSITION_RECEIVER_HPP_
#define _CONTROL_POSITION_RECEIVER_HPP_

#include <ros/ros.h>
#include "position_detection/BlockPositions.h"
#include "planner.hpp"

namespace controller::control {

std::vector<position_detection::BlockPosition> wait_for_new_block_positions(ros::NodeHandle& node_handle);

planner::Block block_pos_to_type(const position_detection::BlockPosition& block_pos);

std::vector<planner::BlockMovement> filter_map_movements_to_pads(
    const std::vector<position_detection::BlockPosition>& blocks,
    double min_confidence,
    size_t max_count
);

} // namespace controller::control

#endif // _CONTROL_POSITION_RECEIVER_HPP_
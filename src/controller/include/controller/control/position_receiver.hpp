#pragma once
#ifndef _CONTROL_POSITION_RECEIVER_HPP_
#define _CONTROL_POSITION_RECEIVER_HPP_

#include <ros/ros.h>
#include "position_detection/BlockPositions.h"
#include "planner.hpp"

namespace controller::control {

/**
 * @brief Waits for new `position_detection::BlockPositions` messages on topic "/block_positions",
 * making sure they are not older than when this function started listening, otherwise they'd be
 * outdated.
 * 
 * @param node_handle the ROS node handle on which to call waitForMessage()
 * @return std::vector<position_detection::BlockPosition> a list of block position messages, sorted
 *         by confidence (higher confidence at the beginning)
 */
std::vector<position_detection::BlockPosition> wait_for_new_block_positions(ros::NodeHandle& node_handle);

/**
 * @brief Obtains the block type stored in a `position_detection::BlockPosition` message.
 */
planner::Block block_pos_to_type(const position_detection::BlockPosition& block_pos);

/**
 * @brief Filters the provided block positions and maps them to block movements that can be planned
 * by the robot to move blocks to their corresponding target pads. The following filters are
 * applied:
 * - blocks with a confidence lower than `min_confidence` are removed
 * - at most `max_count` objects are returned
 * - blocks which are already on their corresponding target pad are removed (e.g. the `1x1_H` block
 *   would be removed if it were already on the `1x1_H` pad)
 * 
 * @param blocks a list of block position messages, sorted by confidence (higher confidence at the
 *               beginning)
 * @param min_confidence all returned blocks will have at least this confidence
 * @param max_count at most this amount of blocks will be returned, preferring higher confidence
 *                  ones
 * @return std::vector<planner::BlockMovement> the block movements to move blocks on the workspace
 *         to their corresponding pad
 */
std::vector<planner::BlockMovement> filter_map_movements_to_pads(
    const std::vector<position_detection::BlockPosition>& blocks,
    double min_confidence,
    size_t max_count
);

} // namespace controller::control

#endif // _CONTROL_POSITION_RECEIVER_HPP_
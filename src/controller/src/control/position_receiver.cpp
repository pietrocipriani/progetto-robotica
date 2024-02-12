#include "controller/control/position_receiver.hpp"

namespace controller::control {

std::vector<position_detection::BlockPosition> wait_for_new_block_positions(ros::NodeHandle& n) {
    ros::Time initial_time = ros::Time::now();
    initial_time.sec += 5; // leave a bit more time for the image to be passed around
    ROS_INFO("Waiting for new block positions on topic /block_positions, initial_time=%d", initial_time.sec);

    while (true) {
        auto block_positions =
            ros::topic::waitForMessage<position_detection::BlockPositions>("/block_positions", n);

        if (!block_positions) {
            ROS_ERROR("Got null block positions from /block_positions");
            return {}; // something crashed in the ros ecosystem, return gracefully
        }

        // header.stamp contains the original timestamp of the point cloud
        // on which the detections were calcuated
        if (block_positions->header.stamp > initial_time) {
            ROS_INFO("Received recent block positions at time %d", block_positions->header.stamp.sec);

            std::vector<position_detection::BlockPosition> blocks = block_positions->blocks;
	        sort(blocks.begin(), blocks.end(), [](const auto& a, const auto& b) {
                return a.confidence > b.confidence; });

            return blocks;
        }

        ROS_INFO("Old block positions received at time %d, retrying", block_positions->header.stamp.sec);
    }
}

planner::Block block_pos_to_type(const position_detection::BlockPosition& block_pos) {
    for (const planner::Block block : planner::all_blocks) {
        if (planner::get_name(block) == block_pos.block_type) {
            return block;
        }
    }
    ROS_ERROR("Unknown block name found in position_detection/BlockPosition message: %s",
        block_pos.block_type.c_str());
    return planner::Block::B_1x1_H;
}

std::vector<planner::BlockMovement> filter_map_movements_to_pads(
    const std::vector<position_detection::BlockPosition>& blocks,
    double min_confidence,
    size_t max_count
) {
    std::vector<planner::BlockMovement> res;
    for (const auto& block_pos : blocks) {
        if (block_pos.confidence < min_confidence || res.size() >= max_count) {
            // blocks is an array sorted by confidence, see wait_for_new_block_positions
            return res;
        }

        planner::Block block = block_pos_to_type(block_pos);
		auto [x, y, z] = block_pos.point;
		double angle = block_pos.angle;

        planner::BlockPose start_pose(block, x, y, angle);
        planner::BlockPose end_pose = planner::BlockPose::pad_pose(block);

        if (start_pose.collides(end_pose) || x > 0.88 || x < 0.07) {
            // the block is already where it should be, so no need to move it
            continue;
        }

        ROS_INFO("Block %s passed filter: x=%.2f y=%.2f z=%.2f angle=%.2f conf=%.4f",
            block_pos.block_type.c_str(), x, y, z, angle, block_pos.confidence);
        res.push_back(planner::BlockMovement(start_pose, end_pose));
    }
    return res;
}

} // namespace controller::pos
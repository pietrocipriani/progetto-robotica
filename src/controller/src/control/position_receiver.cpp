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

} // namespace controller::pos
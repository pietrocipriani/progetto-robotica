#include "controller/pos/position_receiver.hpp"

namespace controller::pos {

position_detection::BlockPositions wait_for_new_block_positions(ros::NodeHandle& n) {
    ros::Time initial_time = ros::Time::now();
    initial_time.sec += 5;
    ROS_INFO("Waiting for new block positions on topic /block_positions, initial_time=%d", initial_time.sec);
    while (true) {
        auto block_positions =
            ros::topic::waitForMessage<position_detection::BlockPositions>("/block_positions", n);
        if (block_positions->header.stamp > initial_time) {
            ROS_INFO("Received recent block positions at time %d", block_positions->header.stamp.sec);
            return *block_positions;
        }
        ROS_INFO("Old block positions received at time %d, retrying", block_positions->header.stamp.sec);
    }
}

} // namespace controller::pos
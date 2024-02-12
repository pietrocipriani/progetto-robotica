#pragma once
#ifndef _WORLD_DELETER_HPP_
#define _WORLD_DELETER_HPP_

#include <ros/ros.h>

#include "planner.hpp"

namespace controller::world {

/**
 * @brief Sends messages of `gazebo_msgs::DeleteModel` type to the "/gazebo/delete_model" topic.
 */
class Deleter {

    ros::ServiceClient client;

public:
    Deleter(ros::ServiceClient&& client);

    /**
     * @brief Deletes a model based on its name.
     * 
     * @param model_name the Gazebo name of the model to delete
     */
    void delete_model(const std::string& model_name);

    /**
     * @brief Deletes the model with the name corresponding to the block type passed,
     * so this only works if the model was spawned by Spawner with `random_name = false`.
     * 
     * @param block_type the type of block to delete
     */
    void delete_block(planner::Block block_type);

    /**
     * @brief Deletes the model with the name corresponding to the pad corresponding to the block
     * type passed. The pad would have been spawned by Spawner using the default name.
     * 
     * @param block_type the block corresponding to the pad to delete
     */
    void delete_pad(planner::Block block_type);

};

} // namespace controller::world

#endif // _WORLD_DELETER_HPP_
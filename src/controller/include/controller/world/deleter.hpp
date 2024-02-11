#pragma once
#ifndef _WORLD_DELETER_HPP_
#define _WORLD_DELETER_HPP_

#include <ros/ros.h>

#include "planner.hpp"

namespace controller::world {

class Deleter {

    ros::ServiceClient client;

public:
    Deleter(ros::ServiceClient&& client);

    void delete_model(const std::string& model_name);

    // only works if the model was spawned with random_name=false
    void delete_block(planner::Block block_type);

    void delete_pad(planner::Block block_type);

};

} // namespace controller::world

#endif // _WORLD_DELETER_HPP_
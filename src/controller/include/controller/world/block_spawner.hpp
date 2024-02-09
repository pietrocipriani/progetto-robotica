#pragma once
#ifndef _WORLD_BLOCK_SPAWNER_HPP_
#define _WORLD_BLOCK_SPAWNER_HPP_

#include <ros/ros.h>

#include "block_type.hpp"
#include "controller/util/color.hpp"

namespace controller::world {

class BlockSpawner {

    ros::ServiceClient client;

public:
    BlockSpawner(ros::ServiceClient&& client);

    void spawn_block(
        planner::Block block_type,
        double x,
        double y,
        double angle,
        bool upside_down,
        const util::Color& color
    );
};

} // namespace controller::world

#endif // _WORLD_BLOCK_SPAWNER_HPP_
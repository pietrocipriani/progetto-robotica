#pragma once
#ifndef _WORLD_BLOCK_SPAWNER_HPP_
#define _WORLD_BLOCK_SPAWNER_HPP_

#include <ros/ros.h>

#include "block_type.hpp"
#include "controller/util/color.hpp"

namespace controller::world {

/**
 * @brief Sends messages of `gazebo_msgs::SpawnModel` type to the "/gazebo/spawn_sdf_model" topic,
 * including in-place generated SDF files pointing to the actual `mesh.stl`.
 */
class Spawner {

    ros::ServiceClient client;

public:
    Spawner(ros::ServiceClient&& client);

    /**
     * @brief Spawns a block in the Gazebo simulation, if one with the same name does not already
     * exist, with a SDF file embedded in C++ code pointing to the actual `mesh.stl`.
     * 
     * @param block_type the block to spawn
     * @param x x
     * @param y y
     * @param angle angle wrt z axis
     * @param upside_down whether to spawn the model upside down
     * @param color the color of the model
     * @param random_name if true, appends the color of the model to the model name, to have a
     *                    pseudorandom name that allows spawning multiple instances of the same
     *                    block. If false and a block with the default name already exists, no
     *                    block will be spawned.
     */
    void spawn_block(
        planner::Block block_type,
        double x,
        double y,
        double angle,
        bool upside_down,
        const util::Color& color,
        bool random_name
    );

    /**
     * @brief Spawns a target pad in the Gazebo simulation, if one with the same name does not
     * already exist, with a SDF file embedded in C++ code pointing to the actual `mesh.stl`.
     * Pads are the targets where blocks will be places, and are just a model with
     * some text indications, placed lying on the workspace table.
     * 
     * @param pad_for_block_type the block corresponding to the target pad. The actual pad position
     *                           will be obtained from planner::get_pad_position()
     * @param x x
     * @param y y
     * @param angle angle wrt z axis
     */
    void spawn_pad(
        planner::Block pad_for_block_type,
        double x,
        double y,
        double angle
    );
};

} // namespace controller::world

#endif // _WORLD_BLOCK_SPAWNER_HPP_
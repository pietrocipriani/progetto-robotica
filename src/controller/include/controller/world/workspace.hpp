#pragma once
#ifndef _WORLD_WORKSPACE_HPP_
#define _WORLD_WORKSPACE_HPP_

#include "spawner.hpp"
#include "deleter.hpp"

namespace controller::world {

/**
 * @brief Sets the up workspace by spawning missing pads and by randomly spawning missing blocks.
 * 
 * @param spawner used to spawn models in Gazebo
 * @param avoid_pads whether to avoid spawning blocks on top of pads
 */
void setup_workspace(Spawner& spawner, bool avoid_pads);

/**
 * @brief Spawns missing pads in the positions defined by planner::get_pad_position().
 * 
 * @param spawner used to spawn models in Gazebo
 */
void spawn_missing_pads(Spawner& spawner);

/**
 * @brief Spawns missing blocks in a random distribution, while avoiding to spawn them below the
 * robotic arm, where there is the internal cylinder with a singularity. If a block already exist
 * in the simulation, it will not be respawned.
 * 
 * @param spawner used to spawn models in Gazebo
 * @param avoid_pads whether to avoid spawning blocks on top of pads
 */
void spawn_blocks(Spawner& spawner, bool avoid_pads);

/**
 * @brief Clears the workspace by removing all blocks and pads. Note that models spawned with
 * `random_name = true` will not be deleted, because their name is not fixed.
 * 
 * @param deleter used to delete models in Gazebo
 */
void clear_workspace(Deleter& deleter);

} // namespace controller::world

#endif // _WORLD_WORKSPACE_HPP_
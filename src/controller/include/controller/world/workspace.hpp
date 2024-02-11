#pragma once
#ifndef _WORLD_WORKSPACE_HPP_
#define _WORLD_WORKSPACE_HPP_

#include "spawner.hpp"

namespace controller::world {

void setup_workspace(Spawner& spawner, bool avoid_pads);

void spawn_missing_pads(Spawner& spawner);

void spawn_blocks(Spawner& spawner, bool avoid_pads);

} // namespace controller::world

#endif // _WORLD_WORKSPACE_HPP_
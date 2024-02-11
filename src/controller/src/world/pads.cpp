#include "controller/world/pads.hpp"

namespace controller::world {

void spawn_missing_pads(Spawner& spawner) {
    for (const planner::Block block : planner::all_blocks) {
        ROS_INFO("Spawning pad %d", static_cast<int>(block));
        auto [x, y, angle] = planner::get_pad_position(block);
        // if the pad already exists this will fail,
        // but it's ok since we don't want duplicate pads
        spawner.spawn_pad(block, x, y, angle - M_PI_2);
    }
}

} // namespace controller::world
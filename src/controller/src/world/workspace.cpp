#include "controller/world/workspace.hpp"

#include <vector>
#include <random>

#include "planner.hpp"
#include "controller/util/color.hpp"

namespace controller::world {

constexpr int MAX_TRIES_TO_PLACE_BLOCK = 1000;
constexpr double WORKSPACE_MIN_X = 0.05;
constexpr double WORKSPACE_MAX_X = 0.92;
constexpr double WORKSPACE_MIN_Y = 0.30;
constexpr double WORKSPACE_MAX_Y = 0.75;

void setup_workspace(Spawner& spawner, bool avoid_pads) {
    spawn_missing_pads(spawner);
    spawn_blocks(spawner, avoid_pads);
}

void spawn_missing_pads(Spawner& spawner) {
    for (const planner::Block block : planner::all_blocks) {
        ROS_INFO("Spawning pad %d", static_cast<int>(block));
        auto [x, y, angle] = planner::get_pad_position(block);
        // if the pad already exists this will fail,
        // but it's ok since we don't want duplicate pads
        spawner.spawn_pad(block, x, y, angle - M_PI_2);
    }
}

void spawn_blocks(Spawner& spawner, bool avoid_pads) {
    static std::mt19937 rng = std::mt19937(std::random_device()());
    static std::uniform_real_distribution<> gen_x(WORKSPACE_MIN_X, WORKSPACE_MAX_X);
    static std::uniform_real_distribution<> gen_y(WORKSPACE_MIN_Y, WORKSPACE_MAX_Y);
    static std::uniform_real_distribution<> gen_angle(0.0, M_2_PI);

    std::vector<planner::BlockPose> used_poses;
    if (avoid_pads) {
        for (const planner::Block& block : planner::all_blocks) {
            used_poses.push_back(planner::BlockPose::pad_pose(block));
        }
    }

    for (const planner::Block& block : planner::all_blocks) {
        bool placed = false;
        for (int i = 0; i < MAX_TRIES_TO_PLACE_BLOCK; ++i) {
            const double x = gen_x(rng), y = gen_y(rng), angle = gen_angle(rng);
            const planner::BlockPose pose(block, x, y, angle);

            bool obstructed = false;
            for (const planner::BlockPose& other_pose : used_poses) {
                if (pose.collides(other_pose)) {
                    obstructed = true;
                    break;
                }
            }
            if (obstructed) {
                continue; // try again
            }

            used_poses.push_back(pose);
            spawner.spawn_block(block, x, y, angle, false, util::gen_bright_color(), false);
            placed = true;
            break;
        }

        if (!placed) {
            ROS_ERROR("Could not find a spot to place block %d", static_cast<int>(block));
        }
    }
}

} // namespace controller::world
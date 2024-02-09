#pragma once
#ifndef _BLOCK_TYPE_HPP_
#define _BLOCK_TYPE_HPP_

namespace planner {

/// Enum representing the types of blocks.
///
enum class Block {
    B_1x1_H = 0,
    B_2x1_T = 1,
    B_2x1_L = 2,
    B_2x1_H = 3,
    B_2x1_U = 4,
    B_2x2_H = 5,
    B_2x2_U = 6,
    B_3x1_H = 7,
    B_3x1_U = 8,
    B_4x1_H = 9,
    B_4x1_L = 10,
};

constexpr const char* get_name(const Block& block_type) {
    switch (block_type) {
        case Block::B_1x1_H:
            return "1x1_H";
        case Block::B_2x1_T:
            return "2x1_T";
        case Block::B_2x1_L:
            return "2x1_L";
        case Block::B_2x1_H:
            return "2x1_H";
        case Block::B_2x1_U:
            return "2x1_U";
        case Block::B_2x2_H:
            return "2x2_H";
        case Block::B_2x2_U:
            return "2x2_U";
        case Block::B_3x1_H:
            return "3x1_H";
        case Block::B_3x1_U:
            return "3x1_U";
        case Block::B_4x1_H:
            return "4x1_H";
        case Block::B_4x1_L:
            return "4x1_L";
    }
    assert(false);
}

constexpr double get_gripping_height(const Block& block_type) {
    switch (block_type) {
        case Block::B_1x1_H:
        case Block::B_2x1_T: case Block::B_2x1_H: case Block::B_2x1_U:
        case Block::B_3x1_H: case Block::B_3x1_U:
        case Block::B_4x1_H:
            return 0.005; // keep a bit away from the table for high blocks
        case Block::B_2x1_L: case Block::B_4x1_L:
            return 0.0; // touch the table for low blocks
        case Block::B_2x2_H: case Block::B_2x2_U:
            return 0.025; // avoid 2x2 blocks from wedging in the gripper's upper part
    }
    assert(false);
}

constexpr double get_open_gripper_pos(const Block& block_type) {
    switch (block_type) {
        // avoid 2x2 blocks from wedging in the gripper's upper part,
        // so open the gripper as much as possible
        case Block::B_2x2_H: case Block::B_2x2_U:
            return 0.9;
        default:
            return 0.5;
    }
}

constexpr double get_closed_gripper_pos(const Block& block_type) {
    switch (block_type) {
        // 2x2 blocks are larger
        case Block::B_2x2_H: case Block::B_2x2_U:
            return 0.2;
        default:
            return -0.2;
    }
    assert(false);
}

/// The bounding box in order to perform safe movements.
/// Circular in order to simplify the checking.
///
constexpr double get_hit_box_radius(const Block& block_type) {
    // TODO fill values
    switch (block_type) {
        case Block::B_1x1_H:
            return 0.0;
        case Block::B_2x1_T:
            return 0.0;
        case Block::B_2x1_L:
            return 0.0;
        case Block::B_2x1_H:
            return 0.0;
        case Block::B_2x1_U:
            return 0.0;
        case Block::B_2x2_H:
            return 0.0;
        case Block::B_2x2_U:
            return 0.0;
        case Block::B_3x1_H:
            return 0.0;
        case Block::B_3x1_U:
            return 0.0;
        case Block::B_4x1_H:
            return 0.0;
        case Block::B_4x1_L:
            return 0.0;
    }
    assert(false);
}

} // namespace controller::world

#endif // _BLOCK_TYPE_HPP_
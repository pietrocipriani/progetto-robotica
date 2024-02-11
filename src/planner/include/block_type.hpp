#pragma once
#ifndef _BLOCK_TYPE_HPP_
#define _BLOCK_TYPE_HPP_

#include <cassert>
#include "utils/math.hpp"

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

constexpr Block all_blocks[] = {
    Block::B_1x1_H,
    Block::B_2x1_T,
    Block::B_2x1_L,
    Block::B_2x1_H,
    Block::B_2x1_U,
    Block::B_2x2_H,
    Block::B_2x2_U,
    Block::B_3x1_H,
    Block::B_3x1_U,
    Block::B_4x1_H,
    Block::B_4x1_L,
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
            return 0.0;
        case Block::B_2x1_L: case Block::B_4x1_L:
            return 0.005; // almost touch the table for low blocks
        case Block::B_2x2_H: case Block::B_2x2_U:
            return 0.025; // avoid 2x2 blocks from wedging in the gripper's upper part
    }
    assert(false);
}

constexpr double get_open_gripper_pos(const Block& block_type) {
    return 0.9; // better to always open the gripper as much as possible
}

constexpr double get_closed_gripper_pos(const Block& block_type) {
    switch (block_type) {
        case Block::B_1x1_H:
        case Block::B_2x1_T: case Block::B_2x1_H: case Block::B_2x1_U:
        case Block::B_3x1_H: case Block::B_3x1_U:
        case Block::B_4x1_H:
            return -0.15;
        case Block::B_2x1_L: case Block::B_4x1_L:
            return -0.2; // low blocks require a bit more closing
        case Block::B_2x2_H: case Block::B_2x2_U:
            return 0.2; // 2x2 blocks are larger
    }
    assert(false);
}

/// The bounding box in order to perform safe movements.
/// Circular in order to simplify the checking.
///
constexpr double get_hit_box_radius(const Block& block_type) {
    constexpr double UNIT_SIZE = 0.031; // all sides of the 1x1 block are this long
    switch (block_type) {
        case Block::B_1x1_H:
            return UNIT_SIZE * rectangle_radius(1, 1);
        case Block::B_2x1_T:
            return UNIT_SIZE * rectangle_radius(2, 1);
        case Block::B_2x1_L:
            return UNIT_SIZE * rectangle_radius(2, 1);
        case Block::B_2x1_H:
            return UNIT_SIZE * rectangle_radius(2, 1);
        case Block::B_2x1_U:
            return UNIT_SIZE * rectangle_radius(2, 1);
        case Block::B_2x2_H:
            return UNIT_SIZE * rectangle_radius(2, 2);
        case Block::B_2x2_U:
            return UNIT_SIZE * rectangle_radius(2, 2);
        case Block::B_3x1_H:
            return UNIT_SIZE * rectangle_radius(3, 1);
        case Block::B_3x1_U:
            return UNIT_SIZE * rectangle_radius(3, 1);
        case Block::B_4x1_H:
            return UNIT_SIZE * rectangle_radius(4, 1);
        case Block::B_4x1_L:
            return UNIT_SIZE * rectangle_radius(4, 1);
    }
    assert(false);
}

/// (x, y, angle)
constexpr std::tuple<double, double, double> get_pad_position(const Block& block_type) {
    switch (block_type) {
        case Block::B_1x1_H:
            return {0.03, 0.50, M_PI};
        case Block::B_2x1_T:
            return {0.92, 0.30, M_PI_2};
        case Block::B_2x1_L:
            return {0.92, 0.38, M_PI_2};
        case Block::B_2x1_H:
            return {0.92, 0.46, M_PI_2};
        case Block::B_2x1_U:
            return {0.92, 0.54, M_PI_2};
        case Block::B_2x2_H:
            return {0.05, 0.73, 3 * M_PI_4};
        case Block::B_2x2_U:
            return {0.03, 0.61, M_PI};
        case Block::B_3x1_H:
            return {0.92, 0.62, M_PI_2};
        case Block::B_3x1_U:
            return {0.92, 0.73, M_PI_4};
        case Block::B_4x1_H:
            return {0.03, 0.35, M_PI};
        case Block::B_4x1_L:
            return {0.25, 0.75, M_PI_2};
    }
    assert(false);
}

} // namespace controller::world

#endif // _BLOCK_TYPE_HPP_

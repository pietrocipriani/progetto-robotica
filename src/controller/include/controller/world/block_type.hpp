#pragma once
#ifndef _BLOCK_TYPE_HPP_
#define _BLOCK_TYPE_HPP_

namespace controller::world {

enum class BlockType {
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

constexpr const char* get_name(const BlockType& block_type) {
    switch (block_type) {
        case BlockType::B_1x1_H:
            return "1x1_H";
        case BlockType::B_2x1_T:
            return "2x1_T";
        case BlockType::B_2x1_L:
            return "2x1_L";
        case BlockType::B_2x1_H:
            return "2x1_H";
        case BlockType::B_2x1_U:
            return "2x1_U";
        case BlockType::B_2x2_H:
            return "2x2_H";
        case BlockType::B_2x2_U:
            return "2x2_U";
        case BlockType::B_3x1_H:
            return "3x1_H";
        case BlockType::B_3x1_U:
            return "3x1_U";
        case BlockType::B_4x1_H:
            return "4x1_H";
        case BlockType::B_4x1_L:
            return "4x1_L";
    }
    return ""; // unreachable
}

} // namespace controller::world

#endif // _BLOCK_TYPE_HPP_
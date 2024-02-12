#pragma once
#ifndef _UTIL_COLOR_HPP_
#define _UTIL_COLOR_HPP_

#include <cstdint>

namespace controller::util {

/**
 * @brief An RGBA color, stored in a 32bit unsigned integer.
 */
union Color {
    struct __attribute__ ((__packed__)) {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t a;
    };
    uint32_t rgba_hex = 0x000000ff; // black
};

/**
 * @brief Creates a 32bit unsigned integer color by converting the 4 floating point components.
 */
Color color_from_double_components(double r, double g, double b, double a);

/**
 * @brief Generates a random color, with a higher probability to be bright rather than dark.
 */
Color gen_bright_color();

} // namespace util

#endif // _UTIL_COLOR_HPP_
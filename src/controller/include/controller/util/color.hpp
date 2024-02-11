#pragma once
#ifndef _UTIL_COLOR_HPP_
#define _UTIL_COLOR_HPP_

#include <cstdint>

namespace controller::util {

union Color {
    struct __attribute__ ((__packed__)) {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t a;
    };
    uint32_t rgba_hex = 0x000000ff; // black
};

Color color_from_double_components(double r, double g, double b, double a);
Color gen_bright_color();

} // namespace util

#endif // _UTIL_COLOR_HPP_
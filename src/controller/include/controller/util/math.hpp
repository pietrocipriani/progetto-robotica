#pragma once
#ifndef _UTIL_MATH_HPP_
#define _UTIL_MATH_HPP_

#include <limits>

namespace controller::util {

constexpr double _sqrtNewtonRaphson(double x, double curr, double prev) {
    return curr == prev
        ? curr
        : _sqrtNewtonRaphson(x, 0.5 * (curr + x / curr), curr);
}

/*
* Constexpr version of the square root
* Return value:
*   - For a finite and non-negative value of "x", returns an approximation for the square root of "x"
*   - Otherwise, returns NaN
* Taken from https://stackoverflow.com/a/34134071
*/
constexpr double constexpr_sqrt(double x) {
    return x >= 0 && x < std::numeric_limits<double>::infinity()
        ? _sqrtNewtonRaphson(x, x, 0)
        : std::numeric_limits<double>::quiet_NaN();
}

constexpr double rectangle_radius(double w, double h) {
    // the collision "radius" of a rectangle is half of its diagonal
    return constexpr_sqrt(w*w + h*h) / 2;
}

} // namespace controller::util

#endif // _UTIL_MATH_HPP_
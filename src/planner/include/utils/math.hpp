#ifndef UTILS_MATH_HPP_INCLUDED
#define UTILS_MATH_HPP_INCLUDED

#include <cmath>
#include "../types.hpp"

/// Exponentiates the given quaternion @p base.
/// @param base The base of the exponentiation.
/// @param exp The exponent.
/// @return The exponentiated quaternion.
/// @note The current implementation by eigen is able to generalize
///       the exponential to @p exp > 1.
/// @note The @p base should be an unit quaternion.
inline Quaternion pow(const Quaternion& base, const Scalar& exp) {
  // Computes `slerp(exp, 1, base) = 1^(1 - exp) + base^exp = base^exp`.
  return Quaternion::Identity().slerp(exp, base);
}

constexpr double _constexpr_sqrt_newton_raphson(double x, double curr, double prev) {
  return curr == prev
    ? curr
    : _constexpr_sqrt_newton_raphson(x, 0.5 * (curr + x / curr), curr);
}

/// Constexpr version of the square root
/// @return
///   - For a finite and non-negative value of "x", returns an approximation for the square root of "x"
///   - Otherwise, returns NaN
/// Taken from https://stackoverflow.com/a/34134071
constexpr double constexpr_sqrt(double x) {
  return x >= 0 && x < std::numeric_limits<double>::infinity()
    ? _constexpr_sqrt_newton_raphson(x, x, 0)
    : std::numeric_limits<double>::quiet_NaN();
}

constexpr double rectangle_radius(double w, double h) {
  // the collision "radius" of a rectangle is half of its diagonal
  return constexpr_sqrt(w*w + h*h) / 2;
}

#endif /* UTILS_MATH_HPP_INCLUDED */

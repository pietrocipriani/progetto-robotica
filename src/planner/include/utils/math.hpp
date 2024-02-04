#ifndef UTILS_MATH_HPP_INCLUDED
#define UTILS_MATH_HPP_INCLUDED

#include <cmath>
#include "../types.hpp"

/**
 * Exponentiates the given quaternion @p base.
 * @param base The base of the exponentiation.
 * @param exp The exponent.
 * @return The exponentiated quaternion.
 * @note The current implementation by eigen is able to generalize
 *       the exponential to @p exp > 1.
 * @note The @p base should be an unit quaternion.
 */
// NOTE: cannot specialize std::pow.
// TODO: compute negative exponentiations with the conjugate.
// TODO: template to keep it in the header without warnings. :)
template<class Scalar>
Quaternion pow(const Quaternion& base, Scalar&& exp) {
  // Computes `slerp(exp, 1, base) = 1^(1 - exp) + base^exp = base^exp`.
  return Quaternion::Identity().slerp(exp, base);
}

template<class Scalar>
Scalar sigmoid(Scalar&& x) {
  if (x > 40) return 1;
  if (x < -40) return -1;
  return (std::exp(2 * x) - 1) / (std::exp(2 * x) + 1);
}



#endif /* UTILS_MATH_HPP_INCLUDED */

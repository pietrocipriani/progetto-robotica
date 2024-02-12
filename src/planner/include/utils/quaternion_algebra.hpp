#ifndef UTILS_QUATERNION_ALGEBRA_INCLUDED
#define UTILS_QUATERNION_ALGEBRA_INCLUDED

#include "../types.hpp"
#include "math.hpp"

/// Namespace containing some (dangerous) functions to uniform the algebra for rotations.
/// @note Supports only quaternions.
/// @note Namespace to avoid the unwanted usage of these functions.
namespace uniformed_rotation_algebra {

inline Quaternion operator-(const Quaternion& q) {
  return q.conjugate();
}
inline Quaternion operator+(const Quaternion& q1, const Quaternion& q2) {
  return (q2 * q1).normalized();
}
inline Quaternion& operator+=(Quaternion& q1, const Quaternion& q2) {
  return q1 = q1 + q2;
}
inline Quaternion operator-(const Quaternion& q1, const Quaternion& q2) {
  return -q2 + q1;
}

/// @p this - @p other = @p result | @p other + @p result = @p this.
/// desired - effective = error | effective + error = result.
/// @param other The starting pose.
/// @return The distance between @p other and @p this.
inline Quaternion& operator-=(Quaternion& q1, const Quaternion& q2) {
  return q1 = q1 - q2;
}

inline Quaternion operator*(const Quaternion& q1, const Scalar& c) {
  return pow(q1, c).normalized();
}
inline Quaternion operator*(const Scalar& c, const Quaternion& q1) {
  return q1 * c;
}
inline Quaternion& operator*=(Quaternion& q1, const Scalar& c) {
  return q1 = q1 * c;
}
inline Quaternion operator/(const Quaternion& q1, const Scalar& c) {
  return q1 * (1 / c);
}
inline Quaternion& operator/=(Quaternion& q1, const Scalar& c) {
  return q1 *= (1 / c);
}

} // namespace uniformed_rotation_algebra



#endif /* UTILS_QUATERNION_ALGEBRA_INCLUDED */

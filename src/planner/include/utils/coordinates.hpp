#ifndef UTILS_COORDINATES_HPP_INCLUDED
#define UTILS_COORDINATES_HPP_INCLUDED

#include "../types.hpp"
#include "../constants.hpp"
#include <cmath>
#include <type_traits>

namespace coordinates {

enum CoordinateSystem {
  Cartesian, Cylindrical
};


template<CoordinateSystem from, CoordinateSystem to>
void convert(Vector<3>& p) {
  static_assert(from == to, "Default implementation is the identity function. Use a specialization.");
  return;
}

template<CoordinateSystem from, CoordinateSystem to>
[[nodiscard]] // Use void convert(Point& p) for in-place conversion.
Vector<3> convert(const Vector<3>& p) {
  static_assert(from == to, "Default implementation is the identity function. Use a specialization.");
  return p;
}

/// Converts a @p cartesian coordinate into cylindrical coordinates.
/// @param cartesian The point in cartesian coordinates.
/// @return The point in cylindrical coordinates.
/// @note Cylindrical coordinates: [rho; theta; h].
template<>
[[nodiscard]] // Use void convert(Point& p) for in-place conversion.
inline Vector<3> convert<Cartesian, Cylindrical>(const Vector<3>& cartesian) {
  const auto rho = cartesian.head<2>().norm();
  const auto theta = std::atan2(cartesian.y(), cartesian.x());
  const auto h = cartesian.z();

  return {rho, theta, h};
}

/// Converts in place a @p cartesian coordinate into cylindrical coordinates.
/// @param cartesian The point in cartesian coordinates.
/// @note Cylindrical coordinates: [rho; theta; h].
template<>
inline void convert<Cartesian, Cylindrical>(Vector<3>& cartesian) {
  const auto& point = cartesian;
  cartesian = convert<Cartesian, Cylindrical>(point);
}



/// Converts a @p cylindrical coordinate into cartesian coordinates.
/// @param cylindrical The point in cartesian coordinates.
/// @return The point @p cylindrical in cartesian coordinates.
/// @note Cylindrical coordinates: [rho; theta; h].
template<>
inline Vector<3> convert<Cylindrical, Cartesian>(const Vector<3>& cylindrical) {
  const auto& rho = cylindrical.x();
  const auto& theta = cylindrical.y();
  const auto& h = cylindrical.z();

  const auto x = std::cos(theta) * rho;
  const auto y = std::sin(theta) * rho;
  const auto z = h;

  return {x, y, z};
}

/// Converts in place a @p cylindrical coordinate into cartesian coordinates.
/// @param cylindrical The point in cartesian coordinates.
/// @note Cylindrical coordinates: [rho; theta; h].
template<>
inline void convert<Cylindrical, Cartesian>(Vector<3>& cylindrical) {
  const auto& point = cylindrical;
  cylindrical = convert<Cylindrical, Cartesian>(point);
}


template<CoordinateSystem from, CoordinateSystem to = Cartesian>
Scalar measure(const Vector<3>& start, const Vector<3>& end) {
  static_assert(from == to, "Default implementation do not admit conversion.");
  return (end - start).norm();
}

template<>
inline Scalar measure<Cylindrical>(const Vector<3>& start, const Vector<3>& end) {
  Vector<3> delta = end - start;
  delta[1] *= end[0];


  Scalar pseudo_norm_sq = std::pow(delta[0], 2) + std::pow(delta[2], 2);

  if (std::abs(delta[1]) < dummy_precision) return delta.norm();
  if (pseudo_norm_sq < dummy_precision) return delta[1];

  return delta.norm() / 2 + std::asinh(delta[1] / std::sqrt(pseudo_norm_sq)) * pseudo_norm_sq / delta[1];
}

}

#endif /* UTILS_COORDINATES_HPP_INCLUDED */

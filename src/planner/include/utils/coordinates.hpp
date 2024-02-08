#ifndef UTILS_COORDINATES_HPP_INCLUDED
#define UTILS_COORDINATES_HPP_INCLUDED

#include "../types.hpp"
#include "../constants.hpp"
#include "../euler.hpp"
#include "unlazy.hpp"
#include "helpers.hpp"
#include "unlazy.hpp"
#include <cmath>
#include <type_traits>

namespace coord {

enum LinearSystem {
  Cartesian, Cylindrical
};
enum AngularSystem {
  Euler, Lie
};


template<LinearSystem linear_system, size_t size>
struct linear_type {
  using type = Vector<size>;
};

template<AngularSystem angular_system, size_t size>
struct angular_type {
  using type = EulerAngles<so<size>>;
  static_assert(angular_system == Euler);
};

template<>
struct angular_type<Lie, 3> {
  using type = Quaternion;
};



template<LinearSystem from, LinearSystem to, size_t size>
typename linear_type<to, size>::type convert(const typename linear_type<from, size>::type& p) {
  static_assert(from == to, "Default implementation is the identity function. Use a specialization.");
  return p;
}

template<AngularSystem from, AngularSystem to, size_t size>
typename angular_type<to, size>::type convert(const typename angular_type<from, size>::type& p) {
  static_assert(from == to, "Default implementation is the identity function. Use a specialization.");
  return p;
}

/// Converts a @p cartesian coordinate into cylindrical coordinates.
/// @param cartesian The point in cartesian coordinates.
/// @return The point in cylindrical coordinates.
/// @note Cylindrical coordinates: [rho; theta; h].
template<>
inline typename linear_type<Cylindrical, 3>::type convert<Cartesian, Cylindrical, 3>(
  const typename linear_type<Cartesian, 3>::type& cartesian
) {
  const auto rho = cartesian.head<2>().norm();
  // TODO: dummy implementation to avoid control panel.
  auto theta = std::atan2(cartesian.y(), cartesian.x());
  if (theta >= M_PI_2) theta -= 2 * M_PI;

  const auto h = cartesian.z();

  return {rho, theta, h};
}

/// Converts a @p cylindrical coordinate into cartesian coordinates.
/// @param cylindrical The point in cartesian coordinates.
/// @return The point @p cylindrical in cartesian coordinates.
/// @note Cylindrical coordinates: [rho; theta; h].
template<>
inline typename linear_type<Cartesian, 3>::type convert<Cylindrical, Cartesian, 3>(
  const typename linear_type<Cylindrical, 3>::type& cylindrical
) {
  const auto& rho = cylindrical.x();
  const auto& theta = cylindrical.y();
  const auto& h = cylindrical.z();

  const auto x = std::cos(theta) * rho;
  const auto y = std::sin(theta) * rho;
  const auto z = h;

  return {x, y, z};
}


/// Converts a @p quaternion into an euler angle.
/// @param cartesian The point in cartesian coordinates.
/// @return The point in cylindrical coordinates.
/// @note Euler representation can be subject to changes, only coherence is asserted.
template<>
inline typename angular_type<Euler, 3>::type convert<Lie, Euler, 3>(
  const typename angular_type<Lie, 3>::type& lie
) {
  // TODO: deprecate euler.
  return euler::from<3>(lie);
}

/// Converts a @p quaternion into an euler angle.
/// @param cartesian The point in cartesian coordinates.
/// @return The point in cylindrical coordinates.
/// @note Euler representation can be subject to changes, only coherence is asserted.
template<>
inline typename angular_type<Lie, 3>::type convert<Euler, Lie, 3>(
  const typename angular_type<Euler, 3>::type& euler
) {
  using Ret = typename angular_type<Lie, 3>::type;
  return Ret(euler::to_rotation<3>(euler));
}

/// Measures the length of a linearly interpolated trajectory between
///     @p start and @p end in the @from system with the @to metric.
template<LinearSystem from, LinearSystem to = Cartesian, size_t size>
Scalar measure(
  const typename linear_type<from, size>::type& start,
  const typename linear_type<from, size>::type& end
) {
  static_assert(from == to, "Default implementation do not allow conversion.");
  return (end - start).norm();
}

/// Measures the length of a linearly interpolated trajectory between
///     @p start and @p end in the @from system with the @to metric.
template<AngularSystem from, AngularSystem to = Lie, size_t size>
Scalar measure(
  const typename angular_type<from, size>::type& start,
  const typename angular_type<from, size>::type& end
) {
  static_assert(from == to, "Default implementation do not allow conversion.");
  return (end - start).norm();
}

/// Measures the length of a linearly interpolated trajectory between
///     @p start and @p end in the @from system with the @to metric.
template<>
inline Scalar measure<Lie, Lie, 3>(const Quaternion& start, const Quaternion& end) {
  return start.angularDistance(end);
}


template<>
inline Scalar measure<Cylindrical, Cartesian, 3>(
  const typename linear_type<Cylindrical, 3>::type& start,
  const typename linear_type<Cylindrical, 3>::type& end
) {
  const auto delta = unlazy(end - start);

  const auto& delta_rho = delta.x();
  const auto& delta_theta = delta.y() * end.x();
  const auto& delta_h = delta.z();


  Scalar pseudo_norm_sq = std::pow(delta_rho, 2) + std::pow(delta_h, 2);

  if (std::abs(delta_theta) < dummy_precision) return delta.norm();
  if (pseudo_norm_sq < dummy_precision) return delta_theta;

  return delta.norm() / 2 + std::asinh(delta_theta / std::sqrt(pseudo_norm_sq)) * pseudo_norm_sq / delta_theta;
}

}

#endif /* UTILS_COORDINATES_HPP_INCLUDED */

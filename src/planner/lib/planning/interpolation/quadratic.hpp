#ifndef QUADRATIC_HPP_INCLUDED
#define QUADRATIC_HPP_INCLUDED


#include "kinematics.hpp"
#include "planner.hpp"
#include "spaces/operational_space.hpp"
#include "types.hpp"
#include "utils.hpp"
#include "utils/coordinates.hpp"
#include <eigen3/Eigen/Core>

namespace planner {

namespace internal {


// TODO: I don't like blacklists.
template<class T>
constexpr bool uses_quaternions = false;

template<>
inline constexpr bool uses_quaternions<Quaternion> = true;

template<>
inline constexpr bool uses_quaternions<kinematics::Pose<coord::Lie, coord::Cylindrical>>
    = is_quasi<kinematics::Pose<coord::Lie, coord::Cylindrical>::Angular, Quaternion>;

template<>
inline constexpr bool uses_quaternions<kinematics::Pose<coord::Lie, coord::Cartesian>>
    = is_quasi<kinematics::Pose<coord::Lie, coord::Cylindrical>::Angular, Quaternion>;

template<class Point, class Velocity>
TimeFunction<Point> _quadratic_interpolation(
  const Point& initial_position,
  const Velocity& initial_velocity,
  const Velocity& final_velocity,
  const Time& start_time,
  const Time& duration
) {
  using namespace uniformed_rotation_algebra;

  // 1/2 a. Due to strict policies in the manipulability of js/os::Position.
  auto acceleration2 = unlazy((final_velocity - initial_velocity) / (2 * duration));

  return [=, acc = std::move(acceleration2)](const Time& time) {
    auto t = time - start_time;
    return initial_position + (initial_velocity + acc * t) * t;
  };
}

}

template<class Point, class Velocity>
TimeFunction<Point> quadratic_acceleration(
  const Point& initial_position,
  const Velocity& final_velocity,
  const Time& start_time,
  const Time& duration
) {
  using namespace uniformed_rotation_algebra;

  auto acc2 = unlazy(final_velocity / (2 * duration));

  return [=, acc = std::move(acc2)](const Time& time) {
    auto t = time - start_time;
    return initial_position + acc * t * t;
  };
}

template<class Point, class Velocity>
TimeFunction<Point> quadratic_deceleration(
  const Point& final_position,
  const Velocity& initial_velocity,
  const Time& final_time,
  const Time& duration
) {
  using namespace uniformed_rotation_algebra;

  auto acc2 = unlazy(-initial_velocity / (2 * duration));

  return [=, acc = std::move(acc2)](const Time& time) {
    auto t = final_time - time;
    return final_position + acc * t * t;
  };
}

template<class Point, class Velocity>
TimeFunction<Point> quadratic_interpolation(
  const Point& initial_position,
  const Velocity& initial_velocity,
  const Velocity& final_velocity,
  const Time& start_time,
  const Time& duration
) {
  static_assert(!internal::uses_quaternions<Point>, "Cannot quadratically interpolate quaternions.");

  return internal::_quadratic_interpolation(
    initial_position,
    initial_velocity, final_velocity,
    start_time, duration
  );
}

}


#endif /* QUADRATIC_HPP_INCLUDED */

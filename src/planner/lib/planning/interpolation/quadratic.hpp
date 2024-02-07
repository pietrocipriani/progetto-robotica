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
TimeFunction<Point> quadratic_interpolation(
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
  static_assert(!internal::uses_quaternions<Point>);

  return internal::quadratic_interpolation(
    initial_position,
    initial_velocity, final_velocity,
    start_time, duration
  );
}

template<>
inline TimeFunction<Quaternion> quadratic_interpolation<Quaternion, Quaternion>(
  const Quaternion& initial_position,
  const Quaternion& initial_velocity,
  const Quaternion& final_velocity,
  const Time& start_time,
  const Time& duration
) {
  using namespace uniformed_rotation_algebra;

  const bool aligned = initial_velocity.vec().cross(final_velocity.vec()).isApprox(Axis::Zero());


  if (aligned) {
    return internal::quadratic_interpolation(
      initial_position,
      initial_velocity, final_velocity,
      start_time, duration
    );
  } else {
    auto mid = initial_position + initial_velocity * (duration / 2);
    auto mid_time = start_time + duration / 2;

    auto dec = quadratic_deceleration(mid, initial_velocity, duration / 2, mid_time);
    auto acc = quadratic_acceleration(mid, final_velocity, duration / 2, mid_time);

    return [=, dec = std::move(dec), acc = std::move(acc)](const Time& time) {
      if (time < mid_time) {
        return dec(time);
      } else {
        return acc(time);
      }
    };
  }
}


template<coord::LinearSystem ls>
TimeFunction<kinematics::Pose<coord::Lie, ls>> quadratic_interpolation(
  const kinematics::Pose<coord::Lie, ls>& initial_position,
  const kinematics::Velocity<coord::Lie, ls>& initial_velocity,
  const kinematics::Velocity<coord::Lie, ls>& final_velocity,
  const Time& start_time,
  const Time& duration
) {
  using namespace uniformed_rotation_algebra;

  auto linear_fun = quadratic_interpolation(
    initial_position.linear(),
    initial_velocity.linear(),
    final_velocity.linear(),
    start_time, duration
  );
  auto angular_fun = quadratic_interpolation(
    initial_position.angular(),
    initial_velocity.angular(),
    final_velocity.angular(),
    start_time, duration
  );

  return [=, l = std::move(linear_fun), a = std::move(angular_fun)](const Time& time) {
    return kinematics::Pose<coord::Lie, ls>(l(time), a(time));
  };
}
}


#endif /* QUADRATIC_HPP_INCLUDED */

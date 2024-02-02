#ifndef INTERPOLATION_HPP_INCLUDED
#define INTERPOLATION_HPP_INCLUDED

#include "planner.hpp"
#include "model.hpp"

namespace planner {

template<class T>
using TimeFunction = std::function<T(const Time&)>;

/**
 * Operational space uniformly accelerated motion with speed saturation.
 * @param robot The robot.
 * @param initial_position The position at time 0 s of the end effector.
 * @param initial_velocity The velocity at time 0 s of the end effector.
 * @param final_velocity The saturation velocity.
 * @param acceleration The magnitude of the acceleration for velocity change.
 * @param time The current time.
 * @return The interpolating function of time that returns the position in a given instant.
 * @note The length is measured in [m].
 * @note The time is measured in [sim].
 * @note This function is applied in the lifting phase of the movement, no rotation needed.
 *        Rotation addition is difficult due to the choice of using quaternions.
 *        Alternatively the same axis of rotation must be imposed.
 *        Otherwise numeric integration can be performed.
 */
[[deprecated("Migrate to quadratic + linear interpolation.")]]
TimeFunction<os::Position> os_uam_interpolation(
  const os::Position& initial_position,
  const os::Velocity& initial_velocity,
  const os::Velocity& final_velocity,
  Scalar acceleration
);

template<class Point>
TimeFunction<Point> linear_interpolation(
  const Point& initial_position, const Point& final_position,
  const Time& start_time,
  const Time& duration
);

template<class Point, class Velocity, bool i_know_exactly_what_i_am_doing_and_i_take_full_responsability = false>
TimeFunction<Point> quadratic_interpolation(
  const Point& initial_position,
  const Velocity& initial_velocity,
  const Velocity& final_velocity,
  const Time& start_time,
  const Time& duration
);

template<class Point, class Velocity>
TimeFunction<Point> quadratic_acceleration(
  const Point& initial_position,
  const Velocity& final_velocity,
  const Time& start_time,
  const Time& duration
);

template<class Point, class Velocity>
TimeFunction<Point> quadratic_deceleration(
  const Point& final_position,
  const Velocity& initial_velocity,
  const Time& start_time,
  const Time& duration
);

template<bool i_know_exactly_what_i_am_doing_and_i_take_full_responsability = false>
TimeFunction<Quaternion> quaternion_interpolation(
  const Quaternion& initial_position,
  const Quaternion& initial_velocity,
  const Quaternion& final_velocity,
  const Time& final_time,
  const Time& duration
);

TimeFunction<js::Position> js_parbolic_interpolation(
  const js::Position& initial_config,
  const js::Position& final_config,
  const js::Velocity& initial_velocity,
  const js::Velocity& final_velocity,
  Scalar acceleration
);



}

#endif /* INTERPOLATION_HPP_INCLUDED */

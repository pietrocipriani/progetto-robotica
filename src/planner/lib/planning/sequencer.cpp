#include "sequencer.hpp"
#include "constants.hpp"
#include "interpolation/parabolic.hpp"
#include "interpolation/stop_and_play.hpp"
#include "kinematics.hpp"
#include "model.hpp"
#include "planner.hpp"
#include "utils/coordinates.hpp"
#include "interpolation.hpp"
#include "utils/helpers.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <tuple>


namespace planner {

using namespace coord;

using LinearParams = Params<os::Position::Linear>;
using AngularParams = Params<os::Position::Angular>;

/// Returns the maximum linear acceleration to avoid to stress the joints.
///
Scalar max_acceleration(const os::Position& pose) {
  // NOTE: dummy implementation for radius awareness.
  return model::UR5::max_joint_accel * pose.linear().norm();
}

/// Returns the maximum angular acceleration to avoid to stress the joints.
///
Scalar max_angular_acceleration() {
  return model::UR5::max_joint_accel;
}


struct SpeedLimits {
  Scalar prev, current, next;

  void shift(Scalar new_limit) {
    prev = current;
    current = next;
    next = new_limit;
  }
};



/// Generates the maximum speed reachable in the given space and the given accelerations.
/// Also the previous and next velocities are considered in order to valutate if it is
///   feasible to accelerate to the given speed.
/// @param distance The distance to travel.
/// @param prev_speed The maximum speed of the previous segment.
void get_maximum_speed(
  Scalar distance,
  SpeedLimits& limits,
  Scalar acc_1, Scalar acc_2
) {
  const Scalar a = acc_1 + acc_2;
  const Scalar b = limits.next * acc_1 + limits.prev * acc_2;
  const Scalar c = -2 * distance * acc_1 * acc_2;

  const Scalar maximum_speed = (-b + std::sqrt(std::pow(b, 2) - 4 * a * c)) / 2 / a;
  
  limits.current = std::min(limits.current, maximum_speed);
}

/// Updates the desired speed and returns the time parameters.
/// @param limits The actual speed limits to be changed.
/// @param distance The distance to travel.
/// @param acc_1 The maximum acceleration for the first velocity change.
/// @param acc_2 The maximum acceleration for the second velocity change.
template<class Point>
typename Params<Point>::Times get_desired_speed(
  SpeedLimits& limits, const Scalar& distance,
  const Scalar& acc_1, const Scalar& acc_2
) {
  assert(limits.prev >= 0 && limits.current >= 0 && limits.next >= 0);
  assert(distance >= 0);
  assert(acc_1 > 0 && acc_2 > 0);

  // Worst case approximation.
  // We are assuming that the two velocities have opposite verse in order to accomodate
  //  the acceleration rating in the worst case.
  // Local optimization. No awareness of the global frame.
  get_maximum_speed(distance, limits, acc_1, acc_2);

  Scalar delta_t_1 = std::max(2.0, (limits.prev + limits.current) / acc_1);
  Scalar delta_t_2 = std::max(2.0, (limits.next + limits.current) / acc_2);

  Time time = limits.current < dummy_precision
    ? delta_t_1 / 2 + delta_t_2 / 2
    : std::max(delta_t_1 / 2 + delta_t_2 / 2, distance / limits.current);

  limits.current = distance / time;

  assert(limits.current >= 0);

  return {time, delta_t_1};
}

/// Generate the timestamps for the via point interpolation.
/// @param point The current point.
/// @param next_point The successive point.
/// @param linear_speed Linear speed limits.
/// @param angular_speed Angular speed limits.
/// @param time The actual time (to be changed).
/// @return The set of time parameters for @p point.
/// @note Local optimization only.
/// @note Do not considers the angular distance.
template<LinearSystem ls, AngularSystem as>
auto generate_parameters(
  const kinematics::Pose<as, ls>& point,
  const kinematics::Pose<as, ls>& next_point,
  SpeedLimits& linear_speed,
  SpeedLimits& angular_speed,
  Time& time
) {
  using TargetPoint = kinematics::Pose<as, ls>;
  assert(angular_speed.prev >= 0 && angular_speed.current >= 0 && angular_speed.next >= 0);

  const Scalar linear_distance = measure<ls, Cartesian, TargetPoint::space_size>(
    point.linear(), next_point.linear()
  );

  const Scalar angular_distance = measure<as, Lie, TargetPoint::space_size>(
    point.angular(), next_point.angular()
  );


  // Approximation:
  // The max acceleration limit is on the single joint and is determined by the inertia of the robot.
  // We are imposing the max acceleration on the end effector movement.
  const Scalar max_lin_acc_1 = max_acceleration(point);
  const Scalar max_lin_acc_2 = max_acceleration(next_point);
  const Scalar max_ang_acc_1 = max_angular_acceleration();
  const Scalar max_ang_acc_2 = max_angular_acceleration();

  auto t_lin = get_desired_speed<TargetPoint>(linear_speed, linear_distance, max_lin_acc_1, max_lin_acc_2);

  [[maybe_unused]]
  auto t_ang = get_desired_speed<TargetPoint>(angular_speed, angular_distance, max_ang_acc_1, max_ang_acc_2);

  const Time current_time = time;

  time += std::max(t_lin.time, 0.0/*t_ang.time*/);

  Scalar delta_t = std::max(t_lin.accel_delta, 0.0/*t_ang.accel_delta*/);

  #ifdef JOINT_SPACE_PLANNING
  model::UR5 robot(model::UR5::Configuration(model::UR5::Configuration::Base::Zero()));
  return Params<model::UR5::Configuration>(kinematics::inverse(robot, point), current_time, delta_t);
  #else
  return Params<TargetPoint>(point, current_time, delta_t);
  #endif
}

/// Generate the timestamps for the last point in via point interpolation.
/// @param point The current point.
/// @param linear_speed Linear speed limits.
/// @param angular_speed Angular speed limits.
/// @param time The actual time (to be changed).
/// @return The set of time parameters for @p point.
/// @note Local optimization only.
/// @note Do not consider the angular distance.
template<LinearSystem linear_system, AngularSystem angular_system>
auto generate_parameters(
  const kinematics::Pose<angular_system, linear_system>& point,
  const SpeedLimits& linear_speed,
  [[maybe_unused]]
  const SpeedLimits& angular_speed,
  Time& time
) {
  using TargetPoint = kinematics::Pose<angular_system, linear_system>;

  const Scalar max_lin_acc = max_acceleration(point);

  [[maybe_unused]]
  const Scalar max_ang_acc = max_angular_acceleration();

  // The current required speed is null, as this is the last point.
  const Scalar delta_t = std::max(linear_speed.prev / max_lin_acc, 0.0/*angular_speed.prev / max_ang_acc*/);

  time += delta_t / 2;

  #ifdef JOINT_SPACE_PLANNING
  model::UR5 robot(model::UR5::Configuration(model::UR5::Configuration::Base::Zero()));
  return Params<model::UR5::Configuration>(kinematics::inverse(robot, point), time, delta_t);
  #else
  return Params<TargetPoint>(point, time, delta_t);
  #endif
}

/// Utility function to manage the interpolation of different @p Point types.
/// @param start The starting point time parameters.
/// @param via_points The via point time parameters.
/// @param end The ending point time parameters.
/// @return The interpolating time function.
template<class Point, template <class T> class Container>
TimeFunction<Point> via_point_interpolation(
  const Params<Point>& start,
  Container<Params<Point>>&& via_points,
  Params<Point>&& end
) {
  constexpr bool is_quat_pose = is_quasi<Point, kinematics::Pose<coord::Lie, coord::Cylindrical>> ||
                                is_quasi<Point, kinematics::Pose<coord::Lie, coord::Cartesian>>;

  if constexpr (is_quasi<Point, Quaternion>) {
    // Quaternion: stop&play interpolation.

    for (auto& p : via_points) p.times.accel_delta /= 2;
    via_points.push_back(std::move(end));
    return stop_and_play_interpolation(start, via_points);
  } else if constexpr (is_quat_pose) {
    // Operational space position using quaternions:
    // - Linear part with parabolic interpolation.
    // - Angular part with stop&play interpolation.
    
    const Params<typename Point::Linear> start_lin(start.point.linear(), start.times.time, start.times.accel_delta);
    const Params<typename Point::Angular> start_ang(start.point.angular(), start.times.time, start.times.accel_delta);
    const Params<typename Point::Linear> end_lin(end.point.linear(), end.times.time, end.times.accel_delta);
    Params<typename Point::Angular> end_ang(end.point.angular(), end.times.time, end.times.accel_delta);
    Container<Params<typename Point::Linear>> vp_lin;
    Container<Params<typename Point::Angular>> vp_ang;
    vp_lin.reserve(via_points.size());
    vp_ang.reserve(via_points.size() + 1);

    for (auto vp : via_points) {
      vp_lin.emplace_back(vp.point.linear(), vp.times.time, vp.times.accel_delta);
      vp_ang.emplace_back(vp.point.angular(), vp.times.time, vp.times.accel_delta / 2);
    }
    vp_ang.push_back(std::move(end_ang));
    
    auto lin = parabolic_interpolation(start_lin, vp_lin, end_lin);
    auto ang = stop_and_play_interpolation(start_ang, vp_ang);

    return [l = std::move(lin), a = std::move(ang)](const Time& time) {
      return Point(l(time), a(time));
    };
  } else {
    return parabolic_interpolation(start, via_points, end);
  }
}


template<LinearSystem linear_system, AngularSystem angular_system>
#ifdef JOINT_SPACE_PLANNING
TimeFunction<model::UR5::Configuration>
#else
TimeFunction<kinematics::Pose<angular_system, linear_system>>
#endif
via_point_sequencer(
  const os::Position& current_pose,
  const ViaPoints& viapoints,
  const os::Position& target_pose,
  Time& finish_time
) {
  using namespace uniformed_rotation_algebra;

  auto vp_it = viapoints.begin();

  auto get_next = [&, finished = false]() mutable -> const os::Position* {
    if (finished) {
      return nullptr;
    } else if (vp_it != viapoints.end()) {
      return &*vp_it++;
    } else {
      finished = true;
      return &target_pose;
    }
  };

  const os::Position *current = &current_pose;
  const os::Position *next = get_next();
  const os::Position *next_next = get_next(); 

  SpeedLimits linear_limits{0, max_linear_speed, viapoints.size() == 0 ? 0 : max_linear_speed};
  SpeedLimits angular_limits{0, max_angular_speed, viapoints.size() == 0 ? 0 : max_angular_speed};

  Scalar time = 0;

  auto start = generate_parameters<linear_system, angular_system>(
    *current, *next, linear_limits, angular_limits, time
  );

  time += start.times.accel_delta / 2;

  std::vector<decltype(start)> via_points;
  via_points.reserve(viapoints.size());

  for ([[maybe_unused]] const auto& vp : viapoints) {
    current = next;
    next = next_next;
    next_next = get_next();

    linear_limits.shift(next_next == nullptr ? 0 : max_linear_speed);
    angular_limits.shift(next_next == nullptr ? 0 : max_angular_speed);

    auto params = generate_parameters<linear_system, angular_system>(
      *current, *next, linear_limits, angular_limits, time
    );

    via_points.push_back(std::move(params));
  }

  linear_limits.shift(0);
  angular_limits.shift(0);

  
  auto end = generate_parameters<linear_system, angular_system>(
    target_pose, linear_limits, angular_limits, time
  );

  finish_time = end.times.time;

  return via_point_interpolation(start, std::move(via_points), std::move(end));
}


#define SPECIALIZATION(ls, as)  template decltype(via_point_sequencer<ls, as>) via_point_sequencer<ls, as>;

SPECIALIZATION(coord::Cylindrical, coord::Lie)
SPECIALIZATION(coord::Cylindrical, coord::Euler)
SPECIALIZATION(coord::Cartesian, coord::Lie)
SPECIALIZATION(coord::Cartesian, coord::Euler)

#undef SPECIALIZATION

}


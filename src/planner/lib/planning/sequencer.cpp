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

Scalar get_max_speed() {
  // NOTE: joint space limits enforced by time dilation.
  return max_linear_speed;
}

// TODO: avoid conversions.
Scalar get_max_acceleration(const os::Position& pose) {
  // TODO: radius awareness.
  return model::UR5::max_joint_accel;
}
Scalar get_max_angular_acceleration(const os::Position& pose) {
  return model::UR5::max_joint_accel;
}

/// Generates the maximum speed reachable in the given space and the given accelerations.
/// Also the previous and next velocities are considered in order to valutate if it is
///   feasible to accelerate to the given speed.
/// @param distance The distance to travel.
/// @param prev_speed The maximum speed of the previous segment.
Scalar get_desired_speed(
  Scalar distance,
  Scalar prev_speed, Scalar next_speed,
  Scalar acc_1, Scalar acc_2
) {
  // TODO: In some cases we can obtain a better result by just checking if we can...
  
  const Scalar a = acc_1 + acc_2;
  const Scalar b = next_speed * acc_1 + prev_speed * acc_2;
  const Scalar c = -2 * distance * acc_1 * acc_2;
  
  return (-b + std::sqrt(std::pow(b, 2) - 4 * a * c)) / 2 / a;
}

// TODO: consider orientation.
// TODO: problems with low accelerations.
template<LinearSystem linear_system, AngularSystem angular_system>
auto generate_parameters(
  const kinematics::Pose<angular_system, linear_system>& point,
  const kinematics::Pose<angular_system, linear_system>& next_point,
  const Scalar prev_speed,
  Scalar& max_speed,
  const Scalar next_speed,
  Time& time
) {
  using TargetPoint = kinematics::Pose<angular_system, linear_system>;
  assert(prev_speed >= 0 && max_speed >= 0 && next_speed >= 0);

  const Scalar linear_distance = measure<linear_system, Cartesian, TargetPoint::space_size>(
    point.linear(), next_point.linear()
  );

  /*const Scalar angular_distance = measure<angular_system, Lie>(
    point.angular(), next_point.angular()
  );*/

  assert(linear_distance >= 0/* && angular_distance >= 0*/);

  // Approximation:
  // The max acceleration limit is on the single joint and is determined by the inertia of the robot.
  // We are imposing the max acceleration on the end effector movement.
  const Scalar max_lin_acc_1 = get_max_acceleration(point);
  const Scalar max_lin_acc_2 = get_max_acceleration(next_point);

  assert(max_lin_acc_1 >= 0 && max_lin_acc_2 >= 0);

  // Worst case approximation.
  // We are assuming that the two velocities have opposite verse in order to accomodate
  //  the acceleration rating in the worst case.
  // Local optimization. No awareness of the global frame.
  const Scalar desired_linear_speed = get_desired_speed(
    linear_distance,
    prev_speed, next_speed,
    max_lin_acc_1, max_lin_acc_2
  );
  max_speed = std::min(desired_linear_speed, max_speed);

  assert(max_speed >= 0);

  const Time current_time = time;

  time += std::max(linear_distance / max_speed, 0.0/*angular_distance / max_speed*/);
  Scalar delta_t = std::max(
    (prev_speed + max_speed) / max_lin_acc_1,
    0.0
  );
  
  #ifdef JOINT_SPACE_PLANNING
  return Params<model::UR5::Configuration>(kinematics::inverse(model::UR5(), point), current_time, delta_t);
  #else
  return Params<TargetPoint>(point, current_time, delta_t);
  #endif
}

template<LinearSystem linear_system, AngularSystem angular_system>
auto generate_parameters(
  const kinematics::Pose<angular_system, linear_system>& point,
  const Scalar prev_linear_speed,
  const Scalar prev_angular_speed,
  Time& time
) {
  using TargetPoint = kinematics::Pose<angular_system, linear_system>;

  // Approximation:
  // The max acceleration limit is on the single joint and is determined by the inertia of the robot.
  // We are imposing the max acceleration on the end effector movement.
  const Scalar max_lin_acc = get_max_acceleration(point);

  const Scalar max_ang_acc = get_max_angular_acceleration(point);

  const Scalar delta_t = std::max(
    prev_linear_speed / max_lin_acc,
    prev_angular_speed / max_ang_acc
  );

  time += delta_t / 2;

  #ifdef JOINT_SPACE_PLANNING
  return Params<model::UR5::Configuration>(kinematics::inverse(model::UR5(), point), time, delta_t);
  #else
  return Params<TargetPoint>(point, time, delta_t);
  #endif
}

template<class Point, template <class T> class Container>
TimeFunction<Point> via_point_interpolation(
  const Params<Point>& start,
  Container<Params<Point>>&& via_points,
  Params<Point>&& end
) {
  constexpr bool is_quat_pose = is_quasi<Point, kinematics::Pose<coord::Lie, coord::Cylindrical>> ||
                                is_quasi<Point, kinematics::Pose<coord::Lie, coord::Cartesian>>;

  if constexpr (is_quasi<Point, Quaternion>) {
    for (auto& p : via_points) p.accel_delta /= 2;
    via_points.push_back(std::move(end));
    return stop_and_play_interpolation(start, via_points);
  } else if constexpr (is_quat_pose) {
    const Params<typename Point::Linear> start_lin(start.point.linear(), start.time, start.accel_delta);
    const Params<typename Point::Angular> start_ang(start.point.angular(), start.time, start.accel_delta);
    const Params<typename Point::Linear> end_lin(end.point.linear(), end.time, end.accel_delta);
    Params<typename Point::Angular> end_ang(end.point.angular(), end.time, end.accel_delta);
    Container<Params<typename Point::Linear>> vp_lin;
    Container<Params<typename Point::Angular>> vp_ang;
    vp_lin.reserve(via_points.size());
    vp_ang.reserve(via_points.size() + 1);

    for (auto vp : via_points) {
      vp_lin.emplace_back(vp.point.linear(), vp.time, vp.accel_delta);
      vp_ang.emplace_back(vp.point.angular(), vp.time, vp.accel_delta / 2);
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
  // TODO: too many special cases.

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

  Scalar prev_speed = 0;
  Scalar max_speed = get_max_speed();
  Scalar next_speed = next_next == nullptr ? 0 : get_max_speed();

  Scalar prev_time = 0;

  auto start = generate_parameters<linear_system, angular_system>(
    *current, *next, prev_speed, max_speed, next_speed, prev_time
  );

  prev_time += start.accel_delta / 2;

  std::vector<decltype(start)> via_points;
  via_points.reserve(viapoints.size());

  for (size_t i = 0; i < viapoints.size(); ++i) {
    current = next;
    next = next_next;
    next_next = get_next();

    prev_speed = max_speed;
    max_speed = next_speed;
    next_speed = next_next == nullptr ? 0 : get_max_speed();

    auto params = generate_parameters<linear_system, angular_system>(
      *current, *next, prev_speed, max_speed, next_speed, prev_time
    );

    via_points.push_back(std::move(params));
  }

  
  auto end = generate_parameters<linear_system, angular_system>(
    target_pose, max_speed, max_speed, prev_time
  );

  finish_time = end.time;

  return via_point_interpolation(start, std::move(via_points), std::move(end));
}


/*template<LinearSystem system>
TimeFunction<os::Position> via_point_sequencer(
  const os::Position& current_pose,
  const ViaPoints& viapoints,
  const os::Position& target_pose,
  Time& finish_time
) {
  // TODO: too many special cases.

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

  Scalar prev_speed = 0;
  Scalar max_speed = get_max_speed(*current, *next);
  Scalar next_speed = next_next == nullptr ? 0 : get_max_speed(*next, *next_next);

  Scalar prev_time = 0;

  auto [start_lp, start_ap] = generate_parameters<Cartesian, system>(
    *current, *next, prev_speed, max_speed, next_speed, prev_time
  );

  prev_time += start_lp.accel_delta / 2;

  std::vector<LinearParams> vlp;
  std::vector<AngularParams> vap;
  vlp.reserve(viapoints.size());
  vap.reserve(viapoints.size() + 1);

  for (size_t i = 0; i < viapoints.size(); ++i) {
    current = next;
    next = next_next;
    next_next = get_next();

    prev_speed = max_speed;
    max_speed = next_speed;
    next_speed = next_next == nullptr ? 0 : get_max_speed(*next, *next_next);

    auto [lp, ap] = generate_parameters<Cartesian, system>(
      *current, *next, prev_speed, max_speed, next_speed, prev_time
    );

    vlp.push_back(std::move(lp));
    vap.push_back(std::move(ap));
  }

  
  auto [end_lp, end_ap] = generate_parameters<Cartesian, system>(
    target_pose, max_speed, prev_time
  );

  finish_time = end_lp.time;
  vap.push_back(end_ap);

  auto linear_fun = parabolic_interpolation(start_lp, vlp, end_lp);
  auto angular_fun = stop_and_play_interpolation(start_ap, vap);
  
  return [lf = std::move(linear_fun), af = std::move(angular_fun)](const Time& t) {
    return os::Position(convert<system, Cartesian>(lf(t)), af(t));
  };
}*/


#define SPECIALIZATION(ls, as)  template decltype(via_point_sequencer<ls, as>) via_point_sequencer<ls, as>;

SPECIALIZATION(coord::Cylindrical, coord::Lie)
SPECIALIZATION(coord::Cylindrical, coord::Euler)
SPECIALIZATION(coord::Cartesian, coord::Lie)
SPECIALIZATION(coord::Cartesian, coord::Euler)

#undef SPECIALIZATION

}


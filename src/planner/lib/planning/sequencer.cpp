#include "sequencer.hpp"
#include "constants.hpp"
#include "interpolation/parabolic.hpp"
#include "interpolation/stop_and_play.hpp"
#include "model.hpp"
#include "planner.hpp"
#include "utils/coordinates.hpp"
#include "interpolation.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <tuple>


namespace planner {

using LinearParams = Params<os::Position::Linear>;
using AngularParams = Params<os::Position::Angular>;

Scalar get_max_speed(const os::Position& start, const os::Position& end) {
  // TODO: consider joint space limits.
  return max_linear_speed;
}

Scalar get_max_acceleration(const os::Position& pose) {
  // TODO: radius awareness.
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
  
  return -b + std::sqrt(std::pow(b, 2) - 4 * a * c) / 2 / a;
}

// TODO: consider orientation.
// TODO: problems with low accelerations.
template<coordinates::CoordinateSystem current, coordinates::CoordinateSystem interpolation, bool first = false>
std::tuple<LinearParams, AngularParams> generate_parameters(
  const os::Position& point,
  const os::Position& next_point,
  const Scalar prev_speed,
  Scalar& max_speed,
  const Scalar next_speed,
  Time& time
) {
  using namespace coordinates;

  const Scalar distance = measure<interpolation>(
    convert<current, interpolation>(point.linear()),
    convert<current, interpolation>(next_point.linear())
  );

  // Approximation:
  // The max acceleration limit is on the single joint and is determined by the inertia of the robot.
  // We are imposing the max acceleration on the end effector movement.
  const Scalar max_acceleration = get_max_acceleration(point);
  const Scalar max_acceleration_next = get_max_acceleration(next_point);

  // Worst case approximation.
  // We are assuming that the two velocities have opposite verse in order to accomodate
  //  the acceleration rating in the worst case.
  // Local optimization. No awareness of the global frame.
  const Scalar desired_speed = get_desired_speed(
    distance,
    prev_speed, next_speed,
    max_acceleration, max_acceleration_next
  );
  max_speed = std::min(desired_speed, max_speed);


  const Time current_time = time;
  time += distance / max_speed;
  Scalar delta_t_1 = (prev_speed + max_speed) / max_acceleration / 2;

  if constexpr (first) {
    time += delta_t_1;
  }

  return std::make_tuple(
    LinearParams(convert<current, interpolation>(point.linear()), current_time, 2 * delta_t_1),
    // TODO: first should have full delta.
    AngularParams(point.angular(), current_time, delta_t_1)
  );
}

// TODO: consider orientation.
template<coordinates::CoordinateSystem current, coordinates::CoordinateSystem interpolation>
std::tuple<LinearParams, AngularParams> generate_parameters(
  const os::Position& point,
  const Scalar prev_speed,
  const Time& time
) {
  using namespace coordinates;

  // Approximation:
  // The max acceleration limit is on the single joint and is determined by the inertia of the robot.
  // We are imposing the max acceleration on the end effector movement.
  const Scalar max_acceleration = get_max_acceleration(point);

  const Scalar delta_t_1 = prev_speed / max_acceleration;

  return std::make_tuple(
    LinearParams(convert<current, interpolation>(point.linear()), time + delta_t_1 / 2, delta_t_1),
    AngularParams(point.angular(), time + delta_t_1 / 2, delta_t_1)
  );
}

template<coordinates::CoordinateSystem system>
TimeFunction<os::Position> via_point_sequencer(
  const os::Position& current_pose,
  const ViaPoints& viapoints,
  const os::Position& target_pose,
  Time& finish_time
) {
  using namespace coordinates;
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
}

template decltype(via_point_sequencer<coordinates::Cylindrical>) via_point_sequencer<coordinates::Cylindrical>;
template decltype(via_point_sequencer<coordinates::Cartesian>) via_point_sequencer<coordinates::Cartesian>;

}


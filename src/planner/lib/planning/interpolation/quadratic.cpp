#include "../interpolation.hpp"
#include "planner.hpp"
#include "types.hpp"
#include "utils.hpp"

namespace planner {

// TODO: I don't like blacklists.
template<class T>
constexpr bool uses_quaternions = false;

template<>
constexpr bool uses_quaternions<Quaternion> = true;

template<>
constexpr bool uses_quaternions<os::Position> = is_quasi<os::Position::Angular, Quaternion>;

template<bool i_know_exactly_what_i_am_doing_and_i_take_full_responsability>
TimeFunction<Quaternion> quaternion_interpolation(
  const Quaternion& initial_position,
  const Quaternion& initial_velocity,
  const Quaternion& final_velocity,
  const Time& start_time,
  const Time& duration
) {
  static_assert(i_know_exactly_what_i_am_doing_and_i_take_full_responsability, "Cannot use this function without taking responsability :).");
  assert(
    initial_velocity.isApprox(Quaternion::Identity()) ||
    final_velocity.isApprox(Quaternion::Identity()) ||
    initial_velocity.vec().normalized().isApprox(final_velocity.vec().normalized()) ||
    initial_velocity.vec().normalized().isApprox(-final_velocity.vec().normalized())
  );

  auto acceleration2 = pow(final_velocity * initial_velocity.conjugate(), 0.5 / duration);

  return [=, acc = std::move(acceleration2)](const Time& time) {
    auto t = time - start_time;
    return pow(pow(acc, t) * initial_velocity, t) * initial_position;
  };
}

TimeFunction<Quaternion> quaternion_acceleration_interpolation(
  const Quaternion& initial_position,
  const Quaternion& final_velocity,
  const Time& start_time,
  const Time& duration
) {
  auto acceleration2 = pow(final_velocity, 0.5 / duration);

  return [=, acc = std::move(acceleration2)](const Time& time) {
    auto t = time - start_time;
    return pow(acc, std::pow(t, 2)) * initial_position;
  };
}

TimeFunction<Quaternion> quaternion_deceleration_interpolation(
  const Quaternion& final_position,
  const Quaternion& initial_velocity,
  const Time& start_time,
  const Time& duration
) {
  auto acceleration2 = pow(initial_velocity, -0.5 / duration);

  auto initial_position = pow(initial_velocity, -0.5 * duration) * final_position;

  return [=, acc = std::move(acceleration2)](const Time& time) {
    auto t = time - start_time;
    return pow(pow(acc, t) * initial_velocity, t) * initial_position;
  };
}

template<class Point, class Velocity, bool i_know_exactly_what_i_am_doing_and_i_take_full_responsability>
TimeFunction<Point> quadratic_interpolation(
  const Point& initial_position,
  const Velocity& initial_velocity,
  const Velocity& final_velocity,
  const Time& start_time,
  const Time& duration
) {
  static_assert(i_know_exactly_what_i_am_doing_and_i_take_full_responsability || !uses_quaternions<Point>, "Cannot quadratically interpolate quaternions.");

  // 1/2 a. Due to strict policies in the manipulability of js/os::Position.
  auto acceleration2 = (final_velocity - initial_velocity) / (2 * duration);

  return [=, acc = std::move(acceleration2)](const Time& time) {
    auto t = time - start_time;
    return initial_position + (initial_velocity + acc * t) * t;
  };
}




#ifdef USE_EULER_ANGLES
template decltype(quadratic_interpolation<os::Position, os::Velocity>) quadratic_interpolation<os::Position, os::Velocity>;
template decltype(quadratic_interpolation<os::Position, os::Velocity, true>) quadratic_interpolation<os::Position, os::Velocity, true>;
#else
template decltype(quadratic_interpolation<os::Position, os::Velocity, true>) quadratic_interpolation<os::Position, os::Velocity, true>;
#endif
template decltype(quadratic_interpolation<os::Position::Linear, os::Velocity::Linear>) quadratic_interpolation<os::Position::Linear, os::Velocity::Linear>;
template decltype(quadratic_interpolation<os::Position::Linear, os::Velocity::Linear, true>) quadratic_interpolation<os::Position::Linear, os::Velocity::Linear, true>;
template decltype(quadratic_interpolation<js::Position, js::Velocity>) quadratic_interpolation<js::Position, js::Velocity>;
template decltype(quadratic_interpolation<js::Position, js::Velocity, true>) quadratic_interpolation<js::Position, js::Velocity, true>;
template decltype(quaternion_interpolation<true>) quaternion_interpolation<true>;

}

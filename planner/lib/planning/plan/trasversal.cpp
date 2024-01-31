#include "../plan.hpp"
#include "planner.hpp"

namespace planner {

/**
 * Converts a @p cartesian coordinate into cylindrical coordinates.
 * @param cartesian The point in cartesian coordinates.
 * @return The point @p cartesian in cylindrical coordinates.
 * @note Cylindrical coordinates: [rho; theta; h].
 */
os::Position::Position cartesian_to_cylindrical(const os::Position::Position& cartesian) {
  const auto rho = cartesian.head<2>().norm();
  const auto theta = std::atan2(cartesian.y(), cartesian.x());
  const auto h = cartesian.z();

  return {rho, theta, h};
}

/**
 * Converts a @p cylindrical coordinate into cartesian coordinates.
 * @param cylindrical The point in cartesian coordinates.
 * @return The point @p cylindrical in cartesian coordinates.
 * @note Cylindrical coordinates: [rho; theta; h].
 */
os::Position::Position cylindrical_to_cartesian(const os::Position::Position& cylindrical) {
  const auto& rho = cylindrical.x();
  const auto& theta = cylindrical.y();
  const auto& h = cylindrical.z();

  const auto x = std::cos(theta) * rho;
  const auto y = std::sin(theta) * rho;
  const auto z = h;

  return {x, y, z};
}


os::Velocity trasversal_movement(
  model::UR5& robot,
  MovementSequence::ConfigSequence& seq,
  model::Scalar dt,
  const js::Position& final_position,
  const js::Velocity& initial_velocity
) {
  
}


}

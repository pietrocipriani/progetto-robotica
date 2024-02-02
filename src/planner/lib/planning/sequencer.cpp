#include "sequencer.hpp"
#include "planner.hpp"
#include <algorithm>


namespace planner {


/**
 * Converts a @p cartesian coordinate into cylindrical coordinates.
 * @param cartesian The point in cartesian coordinates.
 * @note Cylindrical coordinates: [rho; theta; h].
 */
void cartesian_to_cylindrical(os::Position::Linear& cartesian) {
  const auto rho = cartesian.head<2>().norm();
  const auto theta = std::atan2(cartesian.y(), cartesian.x());
  const auto h = cartesian.z();

  cartesian = {rho, theta, h};
}

/**
 * Converts a @p cylindrical coordinate into cartesian coordinates.
 * @param cylindrical The point in cartesian coordinates.
 * @return The point @p cylindrical in cartesian coordinates.
 * @note Cylindrical coordinates: [rho; theta; h].
 */
void cylindrical_to_cartesian(os::Position::Linear& cylindrical) {
  const auto& rho = cylindrical.x();
  const auto& theta = cylindrical.y();
  const auto& h = cylindrical.z();

  const auto x = std::cos(theta) * rho;
  const auto y = std::sin(theta) * rho;
  const auto z = h;

  cylindrical = {x, y, z};
}

os::Position via_point_sequencer(
  model::UR5& robot,
  MovementSequence::ConfigSequence& seq,
  const os::Position& current_pose,
  const Time& dt,
  ViaPoints viapoints
) {
  // Conversion of each waypoint to to cylindrical coords.
  for (auto& pose : viapoints) cartesian_to_cylindrical(pose.linear());

  

}


}


#include "../plan.hpp"
#include "../interpolation.hpp"
#include "constants.hpp"
#include "planner.hpp"

namespace planner {

using namespace model;


/**
 * Finds the estimated maximum speed in the operational space according to a speed
 * limit in the joint space. For lifts only.
 * @param robot The robot.
 * @return The magnitude of the max speed in the operational space.
 * @note The estimation is based on the current position jacobian and is reliable only in
 *  a small neighbour of the current pose.
 *  This is not always the case.
 */
template<class Robot>
Scalar get_max_speed(const Robot& robot) {
  // A unit lift velocity in the operational space.
  os::Velocity os_velocity;
  os_velocity.traslation = os::Velocity::Traslation::UnitZ();

  // The joint space velocity.
  // NOTE: a DLS approach is used, this is not optimal as the path could be distorted.
  const auto js_velocity = kinematics::inverse_diff(robot, os_velocity);

  // Cropping of the `js_velocity` in order to have the max top speed smaller than the max possible speed.
  Scalar max_velocity = *std::max_element(js_velocity.vector().begin(), js_velocity.vector().end());

  Scalar crop = UR5::max_joint_speed / std::abs(max_velocity);

  return crop;
}

os::Velocity lift_movement(UR5& robot, MovementSequence::ConfigSequence& seq, Scalar dt) {

  const auto initial_pose = kinematics::direct(robot);

  // The current velocity in the joint space.
  // Initial velocity is assumed to be null.
  os::Velocity current_velocity;

  // If the robot is not in the low-zone, no action is required.
  if (!is_low_zone(initial_pose)) { 
    return current_velocity;
  }

  // TODO: precomputed or better proof of feasibility needed.
  // [m/sim]
  Scalar max_speed = get_max_speed(robot) * dt;

  // NOTE: another approximation.
  // The kinematics jacobian is considered constant.
  // This means that the hessian is null.
  // The second order derivative can be computed as `a(t) = J(q(t))·q''(t)`.
  // Also the lifting movement is rectilinear in the operational space and also in the
  //  joint space if linearly approximated.
  // We can impose the top acceleration with the same cropping as the velocity.
  // It should also be noted that the joints have no maximum acceleration.
  // The limit is imposed on the torque.
  // Limiting the tourque is not unfeasible, hovewer the calculation of the moment of intertia
  //  of the robot (for each joint) is out of the scope for this project.
  // Also considering that we have already approximated the hessian, precise calcolation
  //  of the max angular velocity is kinda useless.
  // Once the moment of intertia `I` is known, the angular acceleration `alpha` can be computed as:
  // `max_torque = I · alpha <=> alpha = max_torque / I`
  // [m/sim²]
  const Scalar max_acc = max_speed * max_joint_accel / max_joint_speed * dt;

  // Also a operational space limit is necessary.
  // Considering that the joint speed limit is "high", this will probably be the effective speed.
  // The joint limit speed is only useful to avoid problems in singularities,
  //  however in these cases there are other problems such as precision loss.
  max_speed = std::min(max_speed, planner::max_speed * dt);

  // We are trying to lift the end effector: decreasing z of the UR5 base frame.
  const auto max_speed_vector = os::Velocity(
    - os::Velocity::Traslation::UnitZ() * max_speed,
    os::Velocity::Rotation()
  );

  // Interpolation function.
  auto interpolation = os_uam_interpolation(
    initial_pose,
    current_velocity,
    max_speed_vector,
    max_acc
  );

  auto desired_pose = initial_pose;

  // TODO: check on the desired pose? Potentially it is much different respect to the effective pose.
  // NOTE: time in [sim] units.
  for (Scalar time = 1.0; is_low_zone(desired_pose); ++time) {
    // Orientation unchanged in the low-orbit to avoid collisions in case of rectangular blocks.
    // Also we could have just positioned a block.
    os::Position next_pose(interpolation(time), initial_pose.orientation);

    // The desired movement from the previous desired pose to the next desired pose.
    const os::Velocity movement = next_pose - desired_pose;

    current_js_velocity = kinematics::dpa_inverse_diff(robot, movement, desired_pose);

    // TODO: decide if we can change the robot or it is better to keep it unmodified.
    robot.config += current_js_velocity;

    // Registeration of the configuration into the sequence.
    seq.push(robot.config);

    desired_pose = std::move(next_pose);
  }
  
  return current_js_velocity;
}



}

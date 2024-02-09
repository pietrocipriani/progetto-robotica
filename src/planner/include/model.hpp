#ifndef MODEL_HPP_INCLUDED
#define MODEL_HPP_INCLUDED

#include <array>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <limits>
#include "types.hpp"
#include "spaces.hpp"

namespace model {

/**
 * Structure representing the DH parameters for a single revolute joint.
 */
struct RevoluteJoint {

  /**
   * The DH parameters as defined by the DH convention.
   * @p theta is a reference to the correponding robot config entry.
   */
  Scalar d = 0, a = 0, alpha = 0;
  const Scalar& theta;

  /**
   * The physical limits of the joint.
   */
  Scalar min_config = -std::numeric_limits<Scalar>::infinity();
  Scalar max_config = std::numeric_limits<Scalar>::infinity();

  constexpr RevoluteJoint(
    Scalar d, const Scalar& theta, Scalar a, Scalar alpha,
    Scalar min_config = -std::numeric_limits<Scalar>::infinity(),
    Scalar max_config = std::numeric_limits<Scalar>::infinity()
  ) noexcept;
};

/**
 * The UR5 manipolator model.
 */
struct UR5 {
  /**
   * The degrees of freedom of the UR5.
   */
  static constexpr size_t dof = 6;

  /**
   * The maximum rotational speed a joint can have. [rad/s]
   */
  static constexpr Scalar max_joint_speed = M_PI;

  /**
   * Very rough estimation of the max acceleration for the base joint with
   * a fully extended arm. [rad/s²]
   * Only half of the available torque is used.
   */
  static constexpr Scalar max_joint_accel = 40 * 0.5 * 0.01;

  /**
   * The type representing a configuration for a @p dof manipulator.
   */
  using Configuration = JointSpace<dof>;

  using Velocity = Configuration::Derivative;

  using Acceleration = Velocity::Derivative;

  using Parameters = std::array<RevoluteJoint, dof>;

  /**
   * The current configuration of the robot.
   */
  Configuration config;

  /**
   * The parameters for the joints.
   * @note The configuration have to be modified via @p config.
   */
  const Parameters joints;

  const RevoluteJoint& base      = joints[0];
  const RevoluteJoint& shoulder  = joints[1];
  const RevoluteJoint& elbow     = joints[2];
  const RevoluteJoint& wrist1    = joints[3];
  const RevoluteJoint& wrist2    = joints[4];
  const RevoluteJoint& wrist3    = joints[5];

  /**
   * Constructs an UR5 with the given initial configuration.
   * @param homing_config The initial configuration.
   */
  explicit UR5(const Configuration& homing_config) noexcept;

  /**
   * Constructs an UR5 with the given initial configuration.
   * @param homing_config The initial configuration.
   */
  explicit UR5(Configuration&& homing_config) noexcept;

};

}

#endif /* MODEL_HPP_INCLUDED */

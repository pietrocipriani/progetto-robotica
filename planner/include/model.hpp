#ifndef MODEL_HPP_INCLUDED
#define MODEL_HPP_INCLUDED

#include <array>
#include <eigen3/Eigen/Dense>

namespace model {

/**
 * The type representing a scalar.
 * @note The project should be dependent on this alias declaration for integrity.
 */
using Scalar = double;

/**
 * The type representing a configuration for a generic robot.
 * `Eigen` implementation in order to obtain efficient manipulation.
 * @param dof The degrees of freedom.
 */
template<size_t dof>
using generic_config = Eigen::Vector<Scalar, dof>;

/**
 * Structure representing the DH parameters for a single revolute joint.
 */
struct RevoluteJoint {

  /**
   * The DH parameters as defined by the DH convention.
   * @p theta is a reference to the correponding robot config entry.
   */
  Scalar d, &theta, a, alpha;

  /**
   * The physical limits of the joint.
   */
  Scalar min_config, max_config;

  RevoluteJoint(
    Scalar&& d, Scalar& theta, Scalar&& a, Scalar&& alpha,
    Scalar&& min_config, Scalar&& max_config
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
   * The type representing a configuration for a @p dof manipulator.
   */
  using Configuration = generic_config<dof>;

  /**
   * The current configuration of the robot.
   */
  Configuration config;

  /**
   * The parameters for the joints.
   * @note The configuration have to be modified via @p config.
   */
  const std::array<RevoluteJoint, dof> joints;

  const RevoluteJoint& world     = joints[0];
  const RevoluteJoint& base      = joints[1];
  const RevoluteJoint& shoulder  = joints[2];
  const RevoluteJoint& elbow     = joints[3];
  const RevoluteJoint& wrist1    = joints[4];
  const RevoluteJoint& wrist2    = joints[5];
  const RevoluteJoint& wrist3    = joints[6];

  /**
   * Constructs an UR5 in the default homing configuration.
   */
  UR5() noexcept;

  /**
   * Constructs an UR5 with the given initial configuration.
   * @param homing_config The initial configuration.
   */
  UR5(const Configuration& homing_config) noexcept;

  /**
   * Constructs an UR5 with the given initial configuration.
   * @param homing_config The initial configuration.
   */
  UR5(Configuration&& homing_config) noexcept;

};

}

#endif /* MODEL_HPP_INCLUDED */

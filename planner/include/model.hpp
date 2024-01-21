#ifndef MODEL_HPP_INCLUDED
#define MODEL_HPP_INCLUDED

#include <array>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <type_traits>

namespace model {

/**
 * The type representing a scalar.
 * @note The project should be dependent on this alias declaration for integrity.
 */
using scalar = double;

/**
 * The type representing a configuration for a generic robot.
 * `Eigen` implementation in order to obtain efficient manipulation.
 * @param dof The degrees of freedom.
 */
template<size_t dof>
using generic_config = Eigen::Vector<scalar, dof>;

/**
 * Structure representing the DH parameters for a single revolute joint.
 */
struct revolute_joint {

  /**
   * The DH parameters as defined by the DH convention.
   * @p theta is a reference to the correponding robot config entry.
   */
  scalar d, &theta, a, alpha;

  /**
   * The physical limits of the joint.
   */
  scalar min_config, max_config;

  revolute_joint(
    scalar&& d, scalar& theta, scalar&& a, scalar&& alpha,
    scalar&& min_config, scalar&& max_config
  ) noexcept;
};

/**
 * The UR5 manipolator robot for the session.
 */
extern struct ur5 {
  /**
   * The degrees of freedom of the UR5.
   */
  static constexpr size_t dof = 6;

  /**
   * The type representing a configuration for a @p dof manipulator.
   */
  using configuration = generic_config<dof>;

  /**
   * The current configuration of the robot.
   */
  configuration config;

  /**
   * The parameters for the joints.
   * @note The configuration have to be modified via @p config.
   */
  const std::array<revolute_joint, dof> joints;

  const revolute_joint& world     = joints[0];
  const revolute_joint& base      = joints[1];
  const revolute_joint& shoulder  = joints[2];
  const revolute_joint& elbow     = joints[3];
  const revolute_joint& wrist1    = joints[4];
  const revolute_joint& wrist2    = joints[5];
  const revolute_joint& wrist3    = joints[6];

  /**
   * Constructs an UR5 in the default homing configuration.
   */
  ur5() noexcept;

  /**
   * Constructs an UR5 with the given initial configuration.
   * @param homing_config The initial configuration.
   */
  ur5(const configuration& homing_config) noexcept;

  /**
   * Constructs an UR5 with the given initial configuration.
   * @param homing_config The initial configuration.
   */
  ur5(configuration&& homing_config) noexcept;

} robot;

}

#endif /* MODEL_HPP_INCLUDED */

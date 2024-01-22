#ifndef MODEL_HPP_INCLUDED
#define MODEL_HPP_INCLUDED

#include <array>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <limits>

namespace model {

/**
 * The type representing a scalar.
 * @note The project should be dependent on this alias declaration for integrity.
 */
using Scalar = double;

/**
 * Generic matrix in `Scalar` field.
 */
template<size_t rows, size_t cols = rows>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;

/**
 * Generic vector in `Scalar` field.
 */
template<size_t n>
using Vector = Eigen::Vector<Scalar, n>;

/**
 * Quaternion in `Scalar` field.
 */
using Quaternion = Eigen::Quaternion<Scalar>;

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

  constexpr RevoluteJoint(
    Scalar d, Scalar& theta, Scalar a, Scalar alpha,
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
   * The type representing a configuration for a @p dof manipulator.
   */
  using Configuration = Vector<dof>;

  /**
   * The current configuration of the robot.
   */
  Configuration config;

  /**
   * The parameters for the joints.
   * @note The configuration have to be modified via @p config.
   */
  const std::array<RevoluteJoint, dof> joints;

  const RevoluteJoint& base      = joints[0];
  const RevoluteJoint& shoulder  = joints[1];
  const RevoluteJoint& elbow     = joints[2];
  const RevoluteJoint& wrist1    = joints[3];
  const RevoluteJoint& wrist2    = joints[4];
  const RevoluteJoint& wrist3    = joints[5];

  /**
   * Constructs an UR5 in the default homing configuration.
   */
  UR5() noexcept;

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

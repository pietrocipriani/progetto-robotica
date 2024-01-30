#ifndef KINEMATICS_HPP_INCLUDED
#define KINEMATICS_HPP_INCLUDED

#include "model.hpp"

namespace kinematics {
  
/**
 * The size of the operational space.
 */
static constexpr size_t os_size = 3;

/**
 * The size of the orientation space.
 * @note Binomial(os_size 2).
 */
static constexpr size_t orientation_size = os_size * (os_size - 1) / 2;

class Pose_2;

struct Movement {
  /**
   * The underlying container type.
   */
  using Container = model::Vector<os_size + orientation_size>;

  using Traslation = model::Vector<os_size>;
  using Rotation = model::Vector<orientation_size>;
  using TraslationReference = decltype(std::declval<Container>().head<os_size>());
  using RotationReference = decltype(std::declval<Container>().tail<orientation_size>());
  
  /**
   * Movement as x, phi vector.
   */
  Container vector;
  
  /**
   * Traslation view of the vector.
   */
  TraslationReference traslation = vector.head<os_size>();
  
  /**
   * Rotation view of the vector.
   */
  RotationReference rotation = vector.tail<orientation_size>();
  
  Movement operator +(const Movement& movement) const;
  Movement& operator +=(const Movement& movement);
  
  /**
   * Multiplies this movement by the given coefficient.
   */
  Movement operator *(model::Scalar coefficient) const;

  /**
   * Multiplies this movement by the given coefficient.
   */
  Movement& operator *=(model::Scalar coefficient);

private:
  Movement() noexcept = default;
  explicit Movement(Container&& position) noexcept;
  explicit Movement(const Container& position) noexcept;

  friend Pose_2;
};

class Pose_2 {
private:
  using Container = Movement::Container;
  /**
   * Pose represented as movement from the origin.
   */
  Movement from_origin;

  explicit Pose_2(Container&& position) noexcept;
  explicit Pose_2(const Container& position) noexcept;
public:
  using Position = Movement::Traslation;
  using Orientation = Movement::Rotation;
  using PositionReference = Movement::TraslationReference;
  using OrientationReference = Movement::RotationReference;

  PositionReference& position = from_origin.traslation;
  OrientationReference& orientation = from_origin.rotation;

  Pose_2() noexcept = default;
  Pose_2(Position&& position, Orientation&& orientation) noexcept;
  Pose_2(const Position& position, Orientation&& orientation) noexcept;
  Pose_2(Position&& position, const Orientation& orientation) noexcept;
  Pose_2(const Position& position, const Orientation& orientation) noexcept;

  /**
   * Returns the distance between @p other and @p this.
   * @param other The starting pose.
   * @return The distance as movement.
   */
  Movement operator -(const Pose_2& other) const;

  Pose_2 operator +(const Movement& movement) const;
  Pose_2& operator +=(const Movement& movement);
};


/**
 * The pose of the end effector (position + orientation).
 * Position in cartesian coordinates.
 * Orientation as quaternions.
 * @note About the usage of quaternions:
 *  - Euler angles have representational singularities and need the analytical jacobian.
 *  - Angular velocities are much more intuitive that euler angles variations.
 *    - Interpolation of euler angles can lead to "unexpected" movements.
 *  - Quaternions consent numeric integration of the result (a problem of angular velocities).
 *  However:
 *  - Quaternions cannot be analytically integrated unless the rotation axis remains constant. // TODO: verify statement.
 *    - Actually the project require rotations only around the Z axis. (loss of generality).
 *    - Also in case of multiple rotations it is possible to segment the movement: cannot grant continuity in orientation variation.
 *  - The project require rotations only around the Z axis (this is not a good point, loss of generality).
 *  - The planner has full control on the orientation interpolation
 *
 */
struct [[deprecated("Migration to euler angles.")]] Pose {
  /**
   * The size of the operational space.
   */
  static constexpr size_t os_size = 3;

  /**
   * The size of the orientation space.
   */
  static constexpr size_t orientation_size = os_size * (os_size - 1) / 2;

  using Position = model::Vector<os_size>;

  // Quaternion to avoid representation singularities.
  using Orientation = model::Quaternion;

  Position position;
  Orientation orientation;

  Pose() noexcept;
  Pose(Position&& position, Orientation&& orientation) noexcept;
  Pose(const Position& position, const Orientation& orientation) noexcept;

  /**
   * Moves `this` Pose by the given @p movement.
   * @param movement The movement.
   * @return `this` modified pose.
   */
  Pose& move(const Pose& movement);

  /**
   * Returns a pose given by `this` pose moved by @p movement.
   * @param movement The movement.
   * @return The modified pose.
   */
  Pose moved(const Pose& movement) const;

  /**
   * Returns the movement necessary to move from `this` position to the @p desired pose.
   * @param desired The desired pose.
   * @return The movement.
   */
  Pose error(const Pose& desired) const;

  /**
   * Returns the distance between @p other and @p this.
   * This method is substantially a wrapper for Pose::error.
   * It is provided only for clarity in certain contexts.
   * @param other The starting pose.
   * @return The distance as movement.
   */
  Pose operator -(const Pose& other) const;

  /**
   * Inverts `this` movement.
   */
  Pose inverse() const;

  /**
   * Computes the norm of the pose.
   * Useful to compute the norm of `error`.
   */
  model::Scalar norm() const;


  /**
   * Multiplies this movement by the given coefficient.
   */
  Pose operator *(model::Scalar coefficient) const;

  /**
   * Multiplies this movement by the given coefficient.
   */
  Pose& operator *=(model::Scalar coefficient);

};

/**
 * Evaluates the direct kinematics of @p robot.
 * @param robot The robot configuration.
 * @return The pose of the end effector.
 */
Pose_2 direct(const model::UR5& robot) noexcept;

/**
 * Evaluates the direct kinematics of @p robot.
 * @param robot The robot configuration.
 * @param config The target robot configuration.
 * @return The pose of the end effector.
 */
Pose_2 direct(const model::UR5& robot, const model::UR5::Configuration& config) noexcept;

/**
 * Performs the inverse kinematics for @p robot.
 * @param robot The robot parameters.
 * @param pose The desired pose.
 * @return The "nearest" configuration to the current configuration to obtain the given @p pose.
 * @throws @p std::domain_error if the desired @p pose in not in the operational space.
 * @note UR5 is not redundant, however multiple (finite) solutions are allowed.
 */
model::UR5::Configuration inverse(const model::UR5& robot, const Pose_2& pose);

/**
 * Finds the configuration variation in order to obtain the desired velocity in the operational space.
 * @param robot The robot configuration.
 * @param movement The desired velocity in the operational space.
 * @return The configuration variation.
 * @throws @p std::domain_error when in singularity.
 * @note Damped least-squares. This choice could be subject to change.
 */
model::UR5::Configuration inverse_diff(const model::UR5& robot, const Movement& movement);

/**
 * Desired-pose-aware inverse differential kinematics implementation.
 * @param robot The robot configuration.
 * @param movement The desired velocity in the operational space.
 * @param desired_pose The desired current pose.
 * @return The configuration variation.
 * @throws @p std::domain_error when in singularity.
 * @see kinematics::inverse_diff
 */
model::UR5::Configuration dpa_inverse_diff(
  const model::UR5& robot,
  const Movement& movement,
  const Pose_2& desired_pose
);

}

#endif /* KINEMATICS_HPP_INCLUDED */

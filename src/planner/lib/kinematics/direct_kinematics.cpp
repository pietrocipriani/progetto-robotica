#include "kinematics.hpp"
#include "direct_kinematics.hpp"
#include "model.hpp"
#include "utils.hpp"
#include "euler.hpp"
#include "utils/coordinates.hpp"

namespace kinematics {

using namespace model;


/**
 * Utility function to return a translation + rotation transformation around the same axis.
 * @param axis The axis for the translation and the rotation.
 * @param translation The magnitude of the translation.
 * @param rotation The magnitude of the rotation.
 * @return The corresponding transformation.
 */
JointTransformation translation_rotation(const Axis& axis, Scalar translation, Scalar rotation) {
  return Translation(axis * translation) * Rotation(rotation, axis);
}

JointTransformation joint_transformation_matrix(const RevoluteJoint& joint) {
  return joint_transformation_matrix(joint, joint.theta);
}

JointTransformation joint_transformation_matrix(const model::RevoluteJoint& joint, Scalar theta) {
  // d and theta transformation.
  const auto first_transform = translation_rotation(Axis::UnitZ(), joint.d, theta);
  // a and alpha transformation.
  const auto second_transform = translation_rotation(Axis::UnitX(), joint.a, joint.alpha);

  // Complete transformation.
  return first_transform * second_transform;
}

template<
  class Robot, coord::AngularSystem angular_system,
  coord::LinearSystem linear_system
> Pose<angular_system, linear_system> direct(const Robot& robot) noexcept {
  return direct<Robot, angular_system, linear_system>(robot, robot.config);
}

template<
  class Robot, coord::AngularSystem angular_system,
  coord::LinearSystem linear_system
> Pose<angular_system, linear_system> direct(
  const Robot& robot,
  const typename Robot::Configuration& config
) noexcept {
  using DefaultPose = Pose<coord::Lie, coord::Cartesian>;

  JointTransformation transformation = JointTransformation::Identity();

  for (const auto& joint_theta : zip(robot.joints, config.vector())) {
    const auto& [joint, theta] = joint_theta;

    // Apply transformation to the relative frame (post-multiplication).
    transformation = transformation * joint_transformation_matrix(joint, theta);
  }

  DefaultPose::Linear translation = transformation.translation();
  DefaultPose::Angular rotation(transformation.linear());

  const DefaultPose pose(std::move(translation), std::move(rotation));

  return pose;
}

#define SPECIALIZE(m, a, l) template Pose<a, l> direct<m, a, l>(const m&) noexcept;\
                            template Pose<a, l> direct<m, a, l>(const m&, const m::Configuration&) noexcept;


SPECIALIZE(model::UR5, coord::Lie, coord::Cartesian)
SPECIALIZE(model::UR5, coord::Lie, coord::Cylindrical)
SPECIALIZE(model::UR5, coord::Euler, coord::Cartesian)
SPECIALIZE(model::UR5, coord::Euler, coord::Cylindrical)

#undef SPECIALIZE

}

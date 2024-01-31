#include "kinematics.hpp"
#include "direct_kinematics.hpp"
#include "model.hpp"
#include "utils.hpp"
#include "euler.hpp"

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

template<class Robot>
Pose direct(const Robot& robot) noexcept {
  return direct(robot, robot.config);
}

template<class Robot>
Pose direct(const Robot& robot, const typename Robot::Configuration& config) noexcept {
  JointTransformation transformation = JointTransformation::Identity();

  for (const auto& joint_theta : zip(robot.joints, config.vector())) {
    const auto& [joint, theta] = joint_theta;

    // Apply transformation to the relative frame (post-multiplication).
    transformation = transformation * joint_transformation_matrix(joint, theta);
  }

  Pose::Linear translation = transformation.translation();

  // ZYX Euler angles from rotation matrix.
  #ifndef USE_EULER_ANGLES
    Pose::Angular rotation(transformation.linear());
  #else
    Pose::Angular rotation = euler::from<dimension<JointTransformation::LinearPart>>(transformation.linear());
  #endif

  // Construct the pose as position + orientation.
  return Pose(std::move(translation), std::move(rotation));
}

template Pose direct<model::UR5>(const model::UR5&) noexcept;
template Pose direct<model::UR5>(const model::UR5&, const model::UR5::Configuration&) noexcept;

}

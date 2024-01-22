#include "kinematics.hpp"
#include "direct_kinematics.hpp"
#include "model.hpp"

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
  using Translation = Eigen::Translation<Scalar, Pose::os_size>;
  using Rotation = Eigen::AngleAxis<Scalar>;
  
  return Translation(axis * translation) * Rotation(rotation, axis);
} 

JointTransformation joint_transformation_matrix(const RevoluteJoint& joint) {
  // d and theta transformation.
  const auto first_transform = translation_rotation(Axis::UnitZ(), joint.d, joint.theta);
  // a and alpha transformation.
  const auto second_transform = translation_rotation(Axis::UnitX(), joint.a, joint.alpha);

  // Complete transformation.
  return first_transform * second_transform;
}

Pose direct_kinematics(const UR5& robot) noexcept {
  JointTransformation transformation = JointTransformation::Identity();

  for (const auto& joint : robot.joints) {
    // Apply transformation to the relative frame (post-multiplication).
    transformation = transformation * joint_transformation_matrix(joint);
  }

  // Quaternion from rotation matrix.
  Pose::Orientation rotation(transformation.linear());

  // Construct the pose as position + orientation.
  return Pose(transformation.translation(), std::move(rotation));
}

}

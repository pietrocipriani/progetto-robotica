#include "kinematics.hpp"
#include "model.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/AngleAxis.h>
#include <eigen3/Eigen/src/Geometry/Translation.h>

namespace kinematics {

/**
 * Type representing the geometric or analytical jacobian.
 * @note Specific for UR5.
 */
using Jacobian = Eigen::Matrix<model::Scalar, os_size, model::UR5::dof>;

/**
 * Type representing a transformation for direct kineamtics.
 * @note Eigen::TransformTraits::Isometry as Mode in order to speed up some operations (no scaling permitted).
 */
using JointTransformation = Eigen::Transform<model::Scalar, 3, Eigen::TransformTraits::Isometry>;

/**
 * Compute the direct kinematics transformation matrix for a signle joint.
 * @param joint The joint.
 * @return The corresponding transformation matrix to the joint frame.
 */
JointTransformation joint_transformation_matrix(const model::RevoluteJoint& joint) {
  using Translation = Eigen::Translation<model::Scalar, 3>;
  using Rotation = Eigen::AngleAxis<model::Scalar>;
  using Axis = Eigen::Vector<model::Scalar, 3>;

  // TODO: check if it is possible to combine into a single transformation to avoid multiplication.
  // d and theta transformation.
  const auto first_transform = Translation(0, 0, joint.d) * Rotation(joint.theta, Axis::UnitZ());
  // a and alpha transformation.
  const auto second_transform = Translation(joint.a, 0, 0) * Rotation(joint.alpha, Axis::UnitX());

  // Complete transformation.
  return first_transform * second_transform;
}

Pose direct_kinematics(const model::UR5& robot) noexcept {
  JointTransformation transformation = JointTransformation::Identity(); // TODO: world_to_base_transform_matrix;

  for (const auto& joint : robot.joints) {
    transformation = transformation * joint_transformation_matrix(joint);
  }

  // TODO: change `Pose` definition.
  return transformation.translation();
  transformation.linear();
}

}

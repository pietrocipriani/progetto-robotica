#ifndef DIRECT_KINEMATICS_HPP_INCLUDED
#define DIRECT_KINEMATICS_HPP_INCLUDED

#include "model.hpp"
#include "kinematics.hpp"
#include "constants.hpp"
#include "types.hpp"

namespace kinematics {

using Translation = Translation<os_size>;

/// Type representing a transformation for direct kinematics.
/// @note Eigen::TransformTraits::Isometry as Mode in order to speed up some operations (no scaling permitted).
using JointTransformation = Eigen::Transform<Scalar, os_size, Eigen::TransformTraits::Isometry>;

/// Compute the direct kinematics transformation matrix for a signle joint.
/// @param joint The joint.
/// @return The corresponding transformation matrix to the joint frame.
JointTransformation joint_transformation_matrix(const model::RevoluteJoint& joint);

/// Compute the direct kinematics transformation matrix for a signle joint.
/// @param joint The joint.
/// @param theta A custom value theta to avoid config modification.
/// @return The corresponding transformation matrix to the joint frame.
JointTransformation joint_transformation_matrix(const model::RevoluteJoint& joint, Scalar theta);

}

#endif /* DIRECT_KINEMATICS_HPP_INCLUDED */

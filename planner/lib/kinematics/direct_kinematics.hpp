#ifndef DIRECT_KINEMATICS_HPP_INCLUDED
#define DIRECT_KINEMATICS_HPP_INCLUDED

#include "model.hpp"
#include "kinematics.hpp"

namespace kinematics {

/**
 * Type representing a frame axis.
 */
using Axis = model::Vector<Pose::os_size>;

/**
 * Type representing a transformation for direct kinematics.
 * @note Eigen::TransformTraits::Isometry as Mode in order to speed up some operations (no scaling permitted).
 */
using JointTransformation = Eigen::Transform<model::Scalar, Pose::os_size, Eigen::TransformTraits::Isometry>;

/**
 * Compute the direct kinematics transformation matrix for a signle joint.
 * @param joint The joint.
 * @return The corresponding transformation matrix to the joint frame.
 */
JointTransformation joint_transformation_matrix(const model::RevoluteJoint& joint);

}

#endif /* DIRECT_KINEMATICS_HPP_INCLUDED */

#include "kinematics.hpp"

namespace kinematics {

Pose::Pose(Position&& position, Orientation&& orientation)
  : position(std::move(position)), orientation(std::move(orientation)) {}

Pose::Pose(const Position& position, const Orientation& orientation)
  : position(position), orientation(orientation) {}

Pose& Pose::move(const Pose& movement) {
  // Linear composition.
  position += movement.position;

  // Quaternion composition.
  orientation = movement.orientation * orientation;

  return *this;
}

Pose Pose::moved(const Pose& movement) const {
  return Pose(*this).move(movement);
}

Pose Pose::error(const Pose& desired) const {
  return inverse().move(desired);
}

Pose Pose::inverse() const {
  return Pose(-position, orientation.conjugate());
}

model::Scalar Pose::norm() const {
  return std::sqrt(position.squaredNorm() + orientation.vec().squaredNorm());
}


}

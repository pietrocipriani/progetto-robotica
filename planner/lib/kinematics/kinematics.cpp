#include "kinematics.hpp"
#include "utils.hpp"

namespace kinematics {


Pose_2::Pose_2(Container&& pose) noexcept : from_origin(std::move(pose)) {}

Pose_2::Pose_2(const Container& pose) noexcept : from_origin(pose) {}


Pose::Pose(Position&& position, Orientation&& orientation) noexcept
  : position(std::move(position)), orientation(std::move(orientation)) {}

Pose::Pose(const Position& position, const Orientation& orientation) noexcept
  : position(position), orientation(orientation) {}

Pose& Pose::move(const Pose& movement) {
  // Linear composition.
  position += movement.position;

  // Quaternion composition.
  orientation = movement.orientation * orientation;

  // NOTE: Norm has been multiplied. Small errors are exponentially propagated: mitigation.
  orientation.normalize();

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

Pose Pose::operator *(model::Scalar coefficient) const {
  return Pose(position * coefficient, pow(orientation, coefficient));
}

Pose& Pose::operator *=(model::Scalar coefficient) {
  position *= coefficient;
  orientation = pow(orientation, coefficient);
  return *this;
}

}

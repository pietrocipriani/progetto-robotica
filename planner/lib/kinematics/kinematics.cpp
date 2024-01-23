#include "kinematics.hpp"

namespace kinematics {

Pose::Pose(Position&& position, Orientation&& orientation) : position(std::move(position)), orientation(std::move(orientation)) {}
Pose::Pose(const Position& position, const Orientation& orientation) : position(position), orientation(orientation) {}

Pose& Pose::operator +=(const Pose& other) {
  position += other.position;
  orientation = other.orientation * orientation;
  return *this;
}

Pose& Pose::operator -=(const Pose& other) {
  position -= other.position;
  orientation = other.orientation.conjugate() * orientation;
  return *this;
}

Pose Pose::operator +(const Pose& other) const {
  return Pose(position + other.position, other.orientation * orientation);
}

Pose Pose::operator -(const Pose& other) const {
  return Pose(position - other.position, other.orientation.conjugate() * orientation);
}

}

#ifndef JOINT_SPACE_HPP_INCLUDED
#define JOINT_SPACE_HPP_INCLUDED

#include "../types.hpp"



template<size_t size, ssize_t order = 0>
class JointSpace {
public:
  using Base = Vector<size>;
  using Derivative = JointSpace<size, order + 1>;
  using Primitive = JointSpace<size, order - 1>;

private:

  Base _vector;

public:

  JointSpace() noexcept : _vector(Base::Zero()) {}

  explicit JointSpace(const Base& base)
    : _vector(base) {}
  
  explicit JointSpace(Base&& base)
    : _vector(std::move(base)) {}
  

  JointSpace operator-() const {
    return JointSpace(-_vector);
  }

  /**
   * Returns the distance between @p other and @p this.
   * This method is substantially a wrapper for Pose::error.
   * It is provided only for clarity in certain contexts.
   * @param other The starting pose.
   * @return The distance as movement.
   */
  JointSpace operator-(const JointSpace& other) const {
    return JointSpace(*this) -= other;
  }

  JointSpace& operator-=(const JointSpace& variation) {
    return operator+=(-variation);
  }

  JointSpace operator+(const JointSpace& variation) const {
    return JointSpace(*this) += variation;
  }

  JointSpace& operator+=(const JointSpace& variation) {
    _vector += variation._vector;
    return *this;
  }

  Primitive operator*(const Time& dt) const {
    return Primitive(_vector * dt);
  }

  Derivative operator/(const Time& dt) const {
    return Derivative(_vector / dt);
  }

  Scalar norm() const {
    return _vector.norm();
  }

  const Base& vector() const {
    return _vector;
  }

  const auto& operator[](size_t index) const {
    // NOTE: assert check.
    return _vector[index];
  }

  auto& operator[](size_t index) {
    // NOTE: assert check.
    return _vector[index];
  }

  // For uniformity with Eigen only.
  JointSpace& eval() { return *this; }

  // For uniformity with Eigen only.
  const JointSpace& eval() const { return *this; }

};





#endif /* JOINT_SPACE_HPP_INCLUDED */

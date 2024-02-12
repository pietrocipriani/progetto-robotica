#ifndef OPERATIONAL_SPACE_HPP_INCLUDED
#define OPERATIONAL_SPACE_HPP_INCLUDED

#include "../types.hpp"
#include "../constants.hpp"
#include "../utils.hpp"


/// The differentiable operational space.
/// @param linear_system The linear system of coordinates.
/// @param angular_system The angula system of coordinates.
/// @param size The dimension of the space.
/// @param order The time derivative order.
template<coord::LinearSystem linear_system, coord::AngularSystem angular_system, size_t size = 3, ssize_t order = 0>
class OperationalSpace {
public:
  /// The jacobian friendly vector type.
  /// Given by the size of the space + the size of the rotational space.
  using Base = Vector<size + so<size>>;

  using Derivative = OperationalSpace<linear_system, angular_system, size, order + 1>;
  using Primitive = OperationalSpace<linear_system, angular_system, size, order - 1>;

  /// The linear part type.
  ///
  using Linear = typename coord::linear_type<linear_system, size>::type;

  /// The linear part type.
  ///
  using Angular = typename coord::angular_type<angular_system, size>::type;
  static_assert(dimension<Angular> == so<size>, "The angular dimension cannot be represented fully.");

  /// Publicyc accessible size of this `OperationalSpace`.
  ///
  static constexpr size_t space_size = size;

private:

  Linear _linear;
  Angular _angular;

public:

  OperationalSpace() : OperationalSpace(Linear::Zero(), Angular::Identity()) {}

  /// Constructor for implicit functional-style conversions.
  /// @param other A operational space point in arbitrary coordinates.
  /// @return @p other in the given coordinates.
  template<coord::LinearSystem ls, coord::AngularSystem as>
  OperationalSpace(const OperationalSpace<ls, as, size, order>& other)
    : _linear(coord::convert<ls, linear_system, size>(other.linear()))
    , _angular(coord::convert<as, angular_system, size>(other.angular())) {
    // NOTE: velocity conversion can be added.
    static_assert(order == 0, "Only positions can be converted.");
  }
  
  OperationalSpace(const Linear& linear, const Angular& angular)
    : _linear(linear), _angular(angular) {}

  OperationalSpace(const Linear& linear, Angular&& angular)
    : _linear(linear), _angular(std::move(angular)) {}

  OperationalSpace(Linear&& linear, const Angular& angular)
    : _linear(std::move(linear)), _angular(angular) {}

  OperationalSpace(Linear&& linear, Angular&& angular)
    : _linear(std::move(linear)), _angular(std::move(angular)) {}
  

  OperationalSpace operator-() const {
    using namespace uniformed_rotation_algebra;
    return OperationalSpace(-_linear, -_angular);
  }

  // NOTE: this function is not so user friendly. Use with care.
  OperationalSpace operator-(const OperationalSpace& other) const {
    return OperationalSpace(*this) -= other;
  }

  // NOTE: this function is not so user friendly. Use with care.
  OperationalSpace& operator-=(const OperationalSpace& variation) {
    using namespace uniformed_rotation_algebra;

    _linear -= variation._linear;
    _angular -= variation._angular;
    return *this;
  }

  OperationalSpace operator+(const OperationalSpace& variation) const {
    return OperationalSpace(*this) += variation;
  }

  OperationalSpace& operator+=(const OperationalSpace& variation) {
    using namespace uniformed_rotation_algebra;

    _linear += variation._linear;
    _angular += variation._angular;
    return *this;
  }

  Primitive operator*(const Time& dt) const {
    using namespace uniformed_rotation_algebra;

    return Primitive(_linear * dt, _angular * dt);
  }

  Derivative operator/(const Time& dt) const {
    using namespace uniformed_rotation_algebra;

    return Derivative(_linear / dt, _angular / dt);
  }

  Scalar norm() const {
    return vector().norm();
  }

  auto& linear() {
    return _linear;
  }
  
  const auto& linear() const {
    return _linear;
  }

  auto& angular() {
    return _angular;
  }
  
  const auto& angular() const {
    return _angular;
  }

  /// Represents this point as a linear + angular vector.
  /// @note In Lie algebra only use with quaternions.
  const Base vector() const {
    Base result;
    
    // The linear part is just the linear part.
    auto builder = result << _linear;

    if constexpr (angular_system == coord::Lie) {
      // For the angular part in lie algebra we consider the angular pseudo-vector.
      // NOTE: this implementation is in reality only valid for quaternions.

      auto sin_theta_2 = _angular.vec().norm();
      if (sin_theta_2 < dummy_precision) {
        // To avoid numeric approximation bad conditioning.
        builder , _angular.vec() * 2;
      } else {
        // Extraction of the theta angle.
        auto omega = 2 * std::asin(sin_theta_2);
        builder , _angular.vec().normalized() * omega;
      }
    } else {
      // In euler algebra the angular part is already represented as a vector.
      builder , _linear;
    }

    builder.finished();
    return result;
  }

  // For uniformity with Eigen only.
  OperationalSpace& eval() { return *this; }

  // For uniformity with Eigen only.
  const OperationalSpace& eval() const { return *this; }
  
};


template<coord::LinearSystem linear_system, coord::AngularSystem angular_system, size_t size, ssize_t order>
auto operator*(const Time& time, const OperationalSpace<linear_system, angular_system, size, order>& mov) {
  return mov * time;
}

#endif /* OPERATIONAL_SPACE_HPP_INCLUDED */

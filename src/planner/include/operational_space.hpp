#ifndef OPERATIONAL_SPACE_HPP_INCLUDED
#define OPERATIONAL_SPACE_HPP_INCLUDED

#include "types.hpp"
#include "constants.hpp"
#include "utils.hpp"

template<size_t size = 3, ssize_t order = 0>
class OperationalSpace {
public:
  using Base = Vector<size + so<size>>;
  using Derivative = OperationalSpace<size, order + 1>;
  using Primitive = OperationalSpace<size, order - 1>;

  using Linear = Vector<size>;
#ifndef USE_EULER_ANGLES
  // FIXME: complex for size == 2... unfortunately eigen does not provide a complex type and std::complex has not an uniformed interface.
  using Angular = std::conditional_t<size == 2, Complex, Quaternion>;
#else
  using Angular = EulerAngles<size>;
  using LinearView = decltype(std::declval<Base>().template head<size>());
  using AngularView = decltype(std::declval<Base>().template tail<so<size>>());
#endif
  static_assert(dimension<Angular> == so<size>, "The angular dimension cannot be represented fully.");

private:

#ifndef USE_EULER_ANGLES
  Linear _linear;
  Angular _angular;
#else
  Base _vector;
  LinearView _linear = _vector.template head<size>();
  AngularView _angular = _vector.template tail<so<size>>();
#endif

public:

#ifndef USE_EULER_ANGLES
  
  OperationalSpace(const Linear& linear, const Angular& angular)
    : _linear(linear), _angular(angular) {}

  OperationalSpace(const Linear& linear, Angular&& angular)
    : _linear(linear), _angular(std::move(angular)) {}

  OperationalSpace(Linear&& linear, const Angular& angular)
    : _linear(std::move(linear)), _angular(angular) {}

  OperationalSpace(Linear&& linear, Angular&& angular)
    : _linear(std::move(linear)), _angular(std::move(angular)) {}
  
#else

  explicit OperationalSpace(const Base& base)
    : _vector(base) {}
  
  explicit OperationalSpace(Base&& base)
    : _vector(std::move(base)) {}
  
  OperationalSpace(const Linear& linear, const Angular& angular)
    : OperationalSpace((Base() << linear, angular).finished()) {}

  OperationalSpace(const Linear& linear, Angular&& angular)
    : OperationalSpace((Base() << linear, std::move(angular)).finished()) {}

  OperationalSpace(Linear&& linear, const Angular& angular)
    : OperationalSpace((Base() << std::move(linear), angular).finished()) {}

  OperationalSpace(Linear&& linear, Angular&& angular)
    : OperationalSpace((Base() << std::move(linear), std::move(angular)).finished()) {}

#endif
  

  OperationalSpace operator-() const {
    #ifndef USE_EULER_ANGLES
      return OperationalSpace(-_linear, _angular.conjugate());
    #else
      return OperationalSpace(-_vector);
    #endif
  }

  /**
   * @p this - @p other = @p result | @p other + @p result = @p this.
   * desired - effective = error | effective + error = result.
   *
   * Returns the distance between @p other and @p this.
   * This method is substantially a wrapper for Pose::error.
   * It is provided only for clarity in certain contexts.
   * @param other The starting pose.
   * @return The distance as movement.
   */
  [[deprecated("Unintuitive.")]]
  OperationalSpace operator-(const OperationalSpace& other) const {
    return OperationalSpace(*this) -= other;
  }

  [[deprecated("Unintuitive.")]]
  OperationalSpace& operator-=(const OperationalSpace& variation) {
    // TODO: un-intuitive
    #ifndef USE_EULER_ANGLES
      _linear -= variation._linear;
      _angular = _angular * variation._angular.conjugate();
    #else
      _vector += variation._vector;
    #endif
    return *this;
  }

  [[deprecated("Non commutative.")]]
  OperationalSpace operator+(const OperationalSpace& variation) const {
    return OperationalSpace(*this) += variation;
  }

  [[deprecated("Non commutative.")]]
  OperationalSpace& operator+=(const OperationalSpace& variation) {
    #ifndef USE_EULER_ANGLES
      _linear += variation._linear;
      _angular = (variation._angular * _angular).normalized();
    #else
      _vector += variation._vector;
    #endif
    return *this;
  }

  Primitive operator*(const Time& dt) const {
    #ifndef USE_EULER_ANGLES
      return Primitive(_linear * dt, pow(_angular, dt));
    #else
      return Primitive(_vector * dt);
    #endif
  }

  Derivative operator/(const Time& dt) const {
    #ifndef USE_EULER_ANGLES
      return Derivative(_linear / dt, pow(_angular, 1.0 / dt));
    #else
      return Derivative(_vector / dt);
    #endif
  }

  /*OperationalSpace normalized() const;
  OperationalSpace& normalize() {

  }*/

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

  #ifndef USE_EULER_ANGLES
  const Base vector() const {
    Base result;
    auto builder = result << _linear;

    auto sin_theta_2 = _angular.vec().norm();
    if (sin_theta_2 < dummy_precision) {
      builder , _angular.vec() * 2;
    } else {
      auto omega = 2 * std::asin(sin_theta_2);
      builder , _angular.vec().normalized() * omega;
    }

    builder.finished();
    return result;
  }
  #else
  const Base& vector() const {
    return _vector;
  }
  #endif
};

template<size_t size, ssize_t order>
typename OperationalSpace<size, order>::Primitive operator*(const Time& time, const OperationalSpace<size, order>& mov) {
  return mov * time;
}

#endif /* OPERATIONAL_SPACE_HPP_INCLUDED */

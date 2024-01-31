#ifndef TYPES_HPP_INCLUDED
#define TYPES_HPP_INCLUDED

using Scalar = double;

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "constants.hpp"
#include "utils.hpp"

template<size_t rows, size_t cols = rows>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;

template<size_t N>
using Vector = Eigen::Vector<Scalar, N>;

using Rotation = Eigen::AngleAxis<Scalar>;

using RotationMatrix = Rotation::RotationMatrixType;

using Quaternion = Eigen::Quaternion<Scalar>;

using Axis = Rotation::VectorType;

using Time = Scalar;


template<ssize_t order, size_t size = 3>
class OperationalSpace {
public:
  using Linear = Vector<size>;
  using Angular = Quaternion;
  static_assert(Angular::Dim == size * (size - 1) / 2, "The angular dimension cannot be represented fully.");

private:
  using Derivative = OperationalSpace<order + 1, size>;
  using Primitive = OperationalSpace<order - 1, size>;

  Linear _linear;
  Angular _angular;

public:
  
  OperationalSpace(const Linear& linear, const Angular& angular)
    : _linear(linear), _angular(angular) {}

  OperationalSpace(const Linear& linear, Angular&& angular)
    : _linear(linear), _angular(std::move(angular)) {}

  OperationalSpace(Linear&& linear, const Angular& angular)
    : _linear(std::move(linear)), _angular(angular) {}

  OperationalSpace(Linear&& linear, Angular&& angular)
    : _linear(std::move(linear)), _angular(std::move(angular)) {}
  
  /**
   * Returns the distance between @p other and @p this.
   * This method is substantially a wrapper for Pose::error.
   * It is provided only for clarity in certain contexts.
   * @param other The starting pose.
   * @return The distance as movement.
   */
  OperationalSpace operator-(const OperationalSpace& other) const;
  OperationalSpace& operator-=(const OperationalSpace& variation);

  OperationalSpace operator+(const OperationalSpace& variation) const;

  OperationalSpace& operator+=(const OperationalSpace& variation);

  Primitive operator*(const Time& dt) const {
    return Primitive(_linear * dt, pow(_angular, dt));
  }

  Derivative operator/(const Time& dt) const {
    return Derivative(_linear / dt, pow(_angular, 1.0 / dt));
  }

  OperationalSpace normalized() const;
  OperationalSpace& normalize();

  Scalar norm() const;

  const Linear& linear() const {
    return _linear;
  }

  const Angular& angular() const {
    return _angular;
  }
};


template<size_t time_derivative>
class JsPoint {
public:
  /**
   * The underlying container type.
   */
  using Base = model::Vector<os_size + orientation_size>;

  using Linear = model::Vector<os_size>;
  using Angular = euler::EulerAngles;
  using LinearView = decltype(std::declval<Base>().head<os_size>());
  using AngularView = decltype(std::declval<Base>().tail<orientation_size>());
  using Time = model::Scalar;

private:
  Base container;

public:
  
  LinearView linear = container.head<os_size>();
  AngularView angular = container.tail<orientation_size>();

  LengthOrientation(const Base& vector);
  LengthOrientation(Base&& vector);
  LengthOrientation(const Linear& linear, const Angular& angular);
  LengthOrientation(const Linear& linear, Angular&& angular);
  LengthOrientation(Linear&& linear, const Angular& angular);
  LengthOrientation(Linear&& linear, Angular&& angular);

  /*template< class L, class A, std::enable_if_t<is_linear<L> && is_angular<A>, bool> enabled = true>
  LengthOrientation(L&& linear, A&& angular) noexcept;*/
  
  /**
   * Returns the distance between @p other and @p this.
   * This method is substantially a wrapper for Pose::error.
   * It is provided only for clarity in certain contexts.
   * @param other The starting pose.
   * @return The distance as movement.
   */
  LengthOrientation operator-(const LengthOrientation& other) const;
  LengthOrientation& operator-=(const LengthOrientation& variation);

  LengthOrientation operator+(const LengthOrientation& variation) const;

  LengthOrientation& operator+=(const LengthOrientation& variation);

  LengthOrientation<time_derivative - 1> operator*(model::Scalar time) const;
  LengthOrientation<time_derivative + 1> operator/(model::Scalar time) const;

  LengthOrientation normalized() const;
  LengthOrientation& normalize();

  model::Scalar norm() const;

  explicit operator const Base& () const;
};

#endif /* TYPES_HPP_INCLUDED */

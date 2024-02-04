#ifndef TYPES_HPP_INCLUDED
#define TYPES_HPP_INCLUDED

#include <complex>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

/**
 * The type representing a scalar.
 * @note The project should be dependent on this alias declaration for integrity.
 */
using Scalar = double;

template<size_t rows, size_t cols = rows>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;

template<size_t N>
using Vector = Eigen::Vector<Scalar, N>;

template<size_t size>
using EulerAngles = Vector<size>;

/**
 * Type representing a rotation in the operational space.
 */
using Rotation = Eigen::AngleAxis<Scalar>;

/**
 * Type representing a translation in the operational space.
 */
template<size_t size>
using Translation = Eigen::Translation<Scalar, size>;

using RotationMatrix = Rotation::RotationMatrixType;

/**
 * Quaternion in `Scalar` field.
 */
using Quaternion = Eigen::Quaternion<Scalar>;

/// Complex class to provide uniformity with Quaternion.
/// Unit length complex for 2D rotation.
class Complex : public std::complex<Scalar> {
public:
  static constexpr size_t Dim = 1;

  constexpr Complex() : std::complex<Scalar>(1, 0) {}
  Complex(const Scalar& angle) : std::complex<Scalar>(std::polar<Scalar>(1, angle)) {}

  static constexpr Complex Identity() {
    return Complex();
  }
};

using Axis = Rotation::VectorType;

using Time = Scalar;

template<class T>
using TimeFunction = std::function<T(const Time&)>;


#endif /* TYPES_HPP_INCLUDED */

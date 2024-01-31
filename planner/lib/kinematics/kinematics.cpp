#include "kinematics.hpp"
#include "euler.hpp"
#include "model.hpp"
#include "utils.hpp"

namespace kinematics {
  
template<size_t time_derivative>
LengthOrientation<time_derivative>::LengthOrientation(Base&& vector)
  : container(std::move(vector)) {}

template<size_t time_derivative>
LengthOrientation<time_derivative>::LengthOrientation(const Base& vector)
  : container(vector) {}

template<size_t time_derivative>
LengthOrientation<time_derivative>::LengthOrientation(const Linear& linear, const Angular& angular)
  : container((Base() << linear, angular).finished()) {}

template<size_t time_derivative>
LengthOrientation<time_derivative>::LengthOrientation(const Linear& linear, Angular&& angular)
  : container((Base() << linear, std::move(angular)).finished()) {}

template<size_t time_derivative>
LengthOrientation<time_derivative>::LengthOrientation(Linear&& linear, const Angular& angular)
  : container((Base() << std::move(linear), angular).finished()) {}

template<size_t time_derivative>
LengthOrientation<time_derivative>::LengthOrientation(Linear&& linear, Angular&& angular)
  : container((Base() << std::move(linear), std::move(angular)).finished()) {}




template<size_t time_derivative>
LengthOrientation<time_derivative> LengthOrientation<time_derivative>::operator-(const LengthOrientation& other) const {
  return LengthOrientation(container) -= other;
}

template<size_t time_derivative>
LengthOrientation<time_derivative>& LengthOrientation<time_derivative>::operator-=(const LengthOrientation& other) {
  container -= other.container;
  angular = euler::from(model::Quaternion(euler::to_rotation(angular)));
  return *this;
}

template<size_t time_derivative>
LengthOrientation<time_derivative> LengthOrientation<time_derivative>::operator+(const LengthOrientation& variation) const {
  return LengthOrientation(container) += variation.container;
}

template<size_t time_derivative>
LengthOrientation<time_derivative>& LengthOrientation<time_derivative>::operator+=(const LengthOrientation& variation) {
  container += variation.container;
  angular = euler::from(model::Quaternion(euler::to_rotation(angular)));
  return *this;
}

template<size_t time_derivative>
LengthOrientation<time_derivative - 1> LengthOrientation<time_derivative>::operator*(model::Scalar time) const {
  return LengthOrientation<time_derivative - 1>(container * time);
}

template<size_t time_derivative>
LengthOrientation<time_derivative + 1> LengthOrientation<time_derivative>::operator/(model::Scalar time) const {
  return LengthOrientation<time_derivative + 1>(container / time);
}

template<size_t time_derivative>
LengthOrientation<time_derivative> LengthOrientation<time_derivative>::normalized() const {
  return LengthOrientation(container.normalized());
}

template<size_t time_derivative>
LengthOrientation<time_derivative>& LengthOrientation<time_derivative>::normalize() {
  container.normalize();
  return *this;
}

template<size_t time_derivative>
model::Scalar LengthOrientation<time_derivative>::norm() const {
  return container.norm();
}

template<size_t time_derivative>
LengthOrientation<time_derivative>::operator const Base& () const {
  return container;
}


template class LengthOrientation<0>;
template class LengthOrientation<1>;
template class LengthOrientation<2>;

}

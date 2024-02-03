#ifndef UTILS_UNLAZY_HPP_INCLUDED
#define UTILS_UNLAZY_HPP_INCLUDED

#include "../types.hpp"

/// Takes a lazy expression and evaluates it.
template<class T>
constexpr auto unlazy(T&& lazy_expression) {
  return lazy_expression.eval();
}

template<>
inline auto unlazy<const Quaternion&>(const Quaternion& lazy_expression) {
  return lazy_expression;
}

template<>
inline auto unlazy<Quaternion&>(Quaternion& lazy_expression) {
  return lazy_expression;
}

template<>
inline auto unlazy<Quaternion>(Quaternion&& lazy_expression) {
  return lazy_expression;
}



#endif /* UTILS_UNLAZY_HPP_INCLUDED */

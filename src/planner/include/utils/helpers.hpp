#ifndef UTILS_HELPERS_HPP_INCLUDED
#define UTILS_HELPERS_HPP_INCLUDED

#include <cstddef>
#include <type_traits>
#include "../types.hpp"

/// Checs if a template type @p T is the same as @p Target.
/// Removes references and const from @p T.
template<class T, class Target>
constexpr bool is_quasi = std::is_same_v<std::remove_cv_t<std::remove_reference_t<T>>, Target>;

/// The number of sizes of the SO rotational group.
/// @note This is Binomial(size, 2).
template<size_t size>
constexpr size_t so = size * (size - 1) / 2;

/// The dimension of a certain type representing a space.
/// @note Used to uniform the interface.
template<class T>
constexpr size_t dimension = T::RowsAtCompileTime;

template<>
inline constexpr size_t dimension<Quaternion> = Quaternion::Dim;

template<>
inline constexpr size_t dimension<Complex> = 1;


#endif /* UTILS_HELPERS_HPP_INCLUDED */

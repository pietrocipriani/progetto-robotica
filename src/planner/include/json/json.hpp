#ifndef JSON_JSON_HPP_INCLUDED
#define JSON_JSON_HPP_INCLUDED

#include <string>
#include <variant>
#include <unordered_map>
#include <vector>

namespace json {

class Json {
public:
  /// The `null` value type.
  /// ```null```
  using Null = std::monostate;

  /// The string value type.
  /// ```"Hello World"```
  using String = std::string;

  /// The integer value type.
  /// ```42```
  using Int = int64_t;

  /// The decimal value type.
  /// ```42.0```
  using Dec = double;

  /// The object value type.
  /// ```{ ... }```
  using Object = std::unordered_map<String, Json>;

  /// The list value type.
  /// ```[ ... ]```
  using List = std::vector<Json>;

  /// The boolean value type.
  /// ```true```
  using Bool = bool;



private:
  #define TYPES Null, String, Int, Dec, Object, List, Bool

  using Variant = std::variant<TYPES>;

  /// The union data container.
  ///
  Variant data;

  template<class T>
  using base_type = std::remove_cv_t<std::remove_reference_t<T>>;

  template<class T, class U>
  static constexpr bool is_same_enough = std::is_same_v<base_type<T>, U>;

  template<class T, class... S>
  static constexpr bool is_same_variadic = (std::is_same_v<T, S> || ...);

  template<class T>
  static constexpr bool is_json_type = is_same_variadic<T, TYPES>;

  template<class T>
  using enable = typename std::enable_if_t<is_json_type<T>, T>;

  template<class T>
  using enable_e = typename std::enable_if_t<is_json_type<T> || is_same_enough<T, Json>, T>;
  
  template<class T>
  static constexpr const bool& assignable = std::is_assignable_v<Variant, T>;
  template<class T>
  static constexpr const bool& constructible = std::is_constructible_v<Variant, T>;
  template<class T>
  static constexpr const bool& nt_assignable = std::is_assignable_v<Variant, T>;
  template<class T>
  static constexpr const bool& nt_constructible = std::is_nothrow_constructible_v<Variant, T>;

  friend std::ostream& operator <<(std::ostream&, const Json&);

public:
  Json() = default;

  template<class T, std::enable_if_t<constructible<T>, bool> = true>
  Json(T&& data) noexcept(nt_constructible<T>) : data(std::forward<T&&>(data)) {}
  
  template<class T>
  typename std::enable_if_t<assignable<T>, Json>& operator =(T&& value) noexcept(nt_assignable<T>) {
    data = value;
  }

  /// Implicit cast to base type.
  ///
  template<class T>
  operator const enable<T>& () const {
    return as<T>();
  }

  /// Implicit cast to base type.
  ///
  template<class T>
  operator enable<T>& () {
    return as<T>();
  }

  /// Assumes the json is an object.
  /// Extracts the value corresponding to the @p key.
  const Json& operator[] (const Object::key_type& key) const {
    return get(key);
  }

  /// Assumes the json is an object.
  /// Extracts the value corresponding to the @p key.
  Json& operator[] (const Object::key_type& key) {
    return get(key);
  }

  /// Assumes the json is an object.
  /// Extracts the value corresponding to the @p key.
  template<class T = Json>
  const enable_e<T>& get(const Object::key_type& key) const {
    return as<Object>().at(key).as<T>();
  }

  /// Assumes the json is an object.
  /// Extracts the value corresponding to the @p key.
  template<class T = Json>
  enable_e<T>& get(const Object::key_type& key) {
    return as<Object>().at(key).as<T>();
  }

  /// Assumes the json is a list.
  /// Extracts the value at the given @p index.
  const Json& operator[] (const List::size_type& index) const {
    return get(index);
  }
  
  /// Assumes the json is a list.
  /// Extracts the value at the given @p index.
  Json& operator[] (const List::size_type& index) {
    return get(index);
  }

  /// Assumes the json is a list.
  /// Extracts the value at the given @p index.
  template<class T = Json>
  const enable_e<T>& get(const List::size_type& index) const {
    return as<List>().at(index).as<T>();
  }
  
  /// Assumes the json is a list.
  /// Extracts the value at the given @p index.
  template<class T = Json>
  enable_e<T>& get(const List::size_type& index) {
    return as<List>().at(index).as<T>();
  }

  /// Converts to the given underlying type.
  ///
  template<class T>
  enable_e<T>& as() {
    if constexpr (is_json_type<T>) {
      return std::get<T>(data);
    } else {
      return *this;
    }
  }

  /// Converts to the given underlying type.
  ///
  template<class T>
  const enable_e<T>& as() const {
    if constexpr (is_json_type<T>) {
      return std::get<T>(data);
    } else {
      return *this;
    }
  }

  /// Checks if this json represents a given underlying type.
  ///
  template<class T>
  std::enable_if_t<is_json_type<T>, bool> is() const {
    return std::holds_alternative<T>(data);
  }

  #define SPEC(Type)  bool is##Type() const { return is<Type>(); }\
                      const Type& as##Type() const { return as<Type>(); }\
                      Type& as##Type() { return as<Type>(); }\
                      const Type& get##Type(const Object::key_type& key) const { return get<Type>(key); }\
                      Type& get##Type(const Object::key_type& key) { return get<Type>(key); }\
                      const Type& get##Type(const List::size_type& index) const { return get<Type>(index); }\
                      Type& get##Type(const List::size_type& index) { return get<Type>(index); }
                      
  
  SPEC(Null)
  SPEC(String)
  SPEC(Int)
  SPEC(Dec)
  SPEC(Object)
  SPEC(List)
  SPEC(Bool)

  #undef SPEC
  
};

}

#endif /* JSON_JSON_HPP_INCLUDED */

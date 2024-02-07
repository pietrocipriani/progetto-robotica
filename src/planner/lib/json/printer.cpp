/*#include "json.hpp"
#include <iostream>


namespace json {

std::ostream& operator <<(std::ostream& out, const Json::Null&) {
  return out << "null";
}

std::ostream& operator <<(std::ostream& out, const Json::String& str) {
  std::string replaced = "";
  replaced.reserve(2 * str.size());

  for (char c : str) {
    if (c == '\\') {
      replaced += "\\\\";
    } else if (c == '"') {
      replaced += "\\\"";
    } else {
      replaced += c;
    }
  }
  return std::operator<<(out << '"', str) << '"';
}

std::ostream& operator <<(std::ostream& out, const Json::List& list) {
  out << '[';
  auto it = list.begin();
  if (it != list.end()) out << *it++;
  for (; it != list.end(); ++it) out << ',' << *it;
  return out << ']';
}

std::ostream& operator <<(std::ostream& out, const std::pair<Json::String, Json>& entry) {
  return out << entry.first << ':' << entry.second;
}

std::ostream& operator <<(std::ostream& out, const Json::Object& object) {
  out << '{';
  auto it = object.begin();
  if (it != object.end()) out << *it++;
  for (; it != object.end(); ++it) out << ',' << *it;
  return out << '}';
}



std::ostream& operator <<(std::ostream& out, const Json& json) {
  switch (json.data.index()) {
    case 0: return out << json.as<std::variant_alternative_t<0, Json::Variant>>();
    case 1: return out << json.as<std::variant_alternative_t<1, Json::Variant>>();
    case 2: return out << json.as<std::variant_alternative_t<2, Json::Variant>>();
    case 3: return out << json.as<std::variant_alternative_t<3, Json::Variant>>();
    case 4: return out << json.as<std::variant_alternative_t<4, Json::Variant>>();
    case 5: return out << json.as<std::variant_alternative_t<5, Json::Variant>>();
    case 6: return out << std::boolalpha << json.as<std::variant_alternative_t<6, Json::Variant>>();
  }
  throw std::runtime_error("Unexpected value: " + std::to_string(json.data.index()));
}

std::istream& operator >>(std::istream& in, Json& json) {
  using Iterator = std::istreambuf_iterator<char>;
  json = parse<Iterator, false>(Iterator(in), Iterator());
  return in;
}


}
*/
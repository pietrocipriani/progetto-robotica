#ifndef JSON_PARSERS_HPP_INCLUDED
#define JSON_PARSERS_HPP_INCLUDED

#include "json.hpp"
#include <cmath>
#include <stdexcept>

namespace json {

namespace internal {

template<typename Iterator>
struct Result {
  Json result;
  Iterator end;
};

template<typename Iterator>
Result<Iterator> parseGeneric(Iterator begin, Iterator end);

template<typename Iterator>
Iterator skip_whitespaces(Iterator begin, Iterator end) {
  while (begin != end && std::isspace(*begin)) ++begin;
  return begin;
}

template<typename Iterator>
Result<Iterator> parseString(Iterator begin, Iterator end) {
  if (begin == end || *begin != '"') throw std::invalid_argument("A JSON string has to start with '\"'.");

  std::string str = "";

  ++begin;

  bool escape = false;

  // TODO: unescape
  while (begin != end && (escape || *begin != '"')) {
    if (*begin == '\\') escape = true;
    else escape = false;
    str += *begin;
    ++begin;
  }
  if (begin == end) throw std::invalid_argument("Expected '\"'.");

  return {std::move(str), ++begin};
}
template<typename Iterator>
Result<Iterator> parseObject(Iterator begin, Iterator end) {
  if (begin == end || *begin != '{') throw std::invalid_argument("A JSON object has to start with '{'.");
  
  Json::Object result;

  begin = skip_whitespaces(++begin, end);
  bool expected = true;
  while (begin != end && *begin != '}') {
    if (!expected) throw std::invalid_argument("Expected ',' as separator.");
    Result<Iterator>&& key = parseString(begin, end);
    begin = skip_whitespaces(key.end, end);

    if (begin == end || *begin != ':') throw std::invalid_argument("Expected ':'.");

    Result<Iterator>&& value = parseGeneric(++begin, end);
    begin = skip_whitespaces(value.end, end);

    result.emplace(std::move(key.result.template as<Json::String>()), std::move(value.result));

    expected = false;
    while (begin != end && *begin == ',') {
      begin = skip_whitespaces(++begin, end);
      expected = true;
    }
  }
  if (begin == end) throw std::invalid_argument("Expected '}'.");

  return {std::move(result), ++begin};
}
template<typename Iterator>
Result<Iterator> parseList(Iterator begin, Iterator end) {
  if (begin == end || *begin != '[') throw std::invalid_argument("A JSON list has to start with '['.");
  
  Json::List result;

  begin = skip_whitespaces(++begin, end);
  bool expected = true;
  while (begin != end && *begin != ']') {
    if (!expected) throw std::invalid_argument("Expected ',' as separator.");

    Result<Iterator>&& value = parseGeneric(begin, end);
    begin = skip_whitespaces(value.end, end);

    result.emplace_back(std::move(value.result));

    expected = false;
    while (begin != end && *begin == ',') {
      begin = skip_whitespaces(++begin, end);
      expected = true;
    }
  }
  if (begin == end) throw std::invalid_argument("Expected ']'.");

  return {std::move(result), ++begin};
}
template<typename Iterator>
Result<Iterator> parseNumber(Iterator begin, Iterator end) {
  if (begin == end || (!std::isdigit(*begin) && *begin != '-')) throw std::invalid_argument("Expected a number.");

  Json::Int number = 0;
  bool negative = *begin == '-';
  if (negative) ++begin;

  if (begin == end || !std::isdigit(*begin)) throw std::invalid_argument("Expected a number.");

  while (begin != end && std::isdigit(*begin)) {
    number = number * 10 + (*begin - '0');
    ++begin;
  }

  if (begin == end || (*begin != '.' && *begin != 'e' && *begin != 'E')) return {number, begin};

  Json::Dec decimal = number;

  if (*begin == '.') {
    ++begin;
    Json::Dec factor = 1;
    if (begin == end || !std::isdigit(*begin)) throw std::invalid_argument("Expected a digit.");
    while (begin != end && std::isdigit(*begin)) {
      factor /= 10;
      decimal += factor * (*begin -'0');
      ++begin;
    }
  }

  if (begin != end && (*begin == 'e' || *begin == 'E')) {
    auto r = parseNumber(++begin, end);
    number *= std::pow(10, r.result.template as<Json::Int>());
    begin = r.end;
  }

  return {decimal, begin};
}

template<typename Iterator>
Result<Iterator> parseBool(Iterator begin, Iterator end) {
  if (begin == end || (*begin != 't' && *begin != 'f')) throw std::invalid_argument("Expected 'true' or 'false'.");
  static const std::string truel = "true";
  static const std::string falsel = "false";

  bool result = *begin == 't';
  const std::string& expected = result ? truel : falsel;

  for (const char c : expected) {
    if (begin == end || *begin++ != c) throw std::invalid_argument("Expected '" + expected + "'.");
  }

  return {result, begin};
}
template<typename Iterator>
Result<Iterator> parseNull(Iterator begin, Iterator end) {
  if (begin == end) throw std::invalid_argument("Expected 'null'.");
  static const std::string expected = "null";

  for (const char c : expected) {
    if (begin == end || *begin++ != c) throw std::invalid_argument("Expected 'null'.");
  }

  return {Json::Null(), begin};
}

template<typename Iterator>
Result<Iterator> parseGeneric(Iterator begin, Iterator end) {
  begin = skip_whitespaces(begin, end);

  if (begin == end) throw std::invalid_argument("Cannot parse an empty string as a Json.");

  Result<Iterator> result;
  if (*begin == '{') {
    result = parseObject(begin, end);
  } else if (*begin == '[') {
    result = parseList(begin, end);
  } else if (*begin == '"') {
    result = parseString(begin, end);
  } else if ((*begin >= '0' && *begin <= '9') || *begin == '-') {
    result = parseNumber(begin, end);
  } else if (*begin == 't' || *begin == 'f') {
    result = parseBool(begin, end);
  } else if (*begin == 'n') {
    result = parseNull(begin, end);
  } else {
    throw std::invalid_argument("Unexpected character '" + std::to_string(*begin) + "'.");
  }
  return { std::move(result.result) , skip_whitespaces(result.end, end) };
}

}





template<class Iterator, bool check_end = true>
Json parse(Iterator begin, Iterator end) {
  using namespace internal;

  Result<Iterator> result = parseGeneric(begin, end);

  begin = result.end;
  if (check_end && begin != end) throw std::invalid_argument("Unexpected character at the end of input.");

  return std::move(result.result);
}

/// Parses the given (char) iterable and returns the corresponding json.
///
template<class Iterable, bool check_end = true>
Json parse(Iterable&& raw) {
  return parse(raw.begin(), raw.end());
}

}


#endif /* JSON_PARSERS_HPP_INCLUDED */

#pragma once
#ifndef _UTIL_STRING_HPP_
#define _UTIL_STRING_HPP_

#include <string>
#include <memory>
#include "model.hpp"

namespace controller::util {

template<typename... Args>
std::string string_format(const std::string& format, Args... args) {
    int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1; // Extra space for '\0'
    if(size_s <= 0) { throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>(size_s);
    std::unique_ptr<char[]> buf(new char[size]);
    std::snprintf(buf.get(), size, format.c_str(), args...);
    return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

std::string config_to_string(const model::UR5::Configuration& config, const double& gripper_pos);

} // namespace util

#endif // _UTIL_STRING_HPP_
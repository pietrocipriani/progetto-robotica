#include "controller/util/string.hpp"

namespace controller::util {

std::string config_to_string(const model::UR5::Configuration& config, const double& gripper_pos) {
    const auto& v = config.vector();
    return string_format("%.4f %.4f %.4f %.4f %.4f %.4f %.2f",
        v[0], v[1], v[2], v[3], v[4], v[5], gripper_pos);
}

} // namespace controller::util
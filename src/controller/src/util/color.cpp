#include "controller/util/color.hpp"

#include <random>
#include <cassert>

namespace controller::util {

inline uint8_t double_to_uint8(double v) {
    assert(v >= 0.0 && v <= 1.0);
    return uint8_t(v * 255 + 0.5);
}

Color color_from_double_components(double r, double g, double b, double a) {
    return Color{
        double_to_uint8(r),
        double_to_uint8(g),
        double_to_uint8(b),
        double_to_uint8(a)
    };
}

Color gen_bright_color() {
    static std::mt19937 rng = std::mt19937(std::random_device()());
    static std::uniform_real_distribution gen_component(0.0, 1.0);

    // use sqrt to make it more likely to generate brighter colors
    return color_from_double_components(
        std::sqrt(gen_component(rng)),
        std::sqrt(gen_component(rng)),
        std::sqrt(gen_component(rng)),
        1.0
    );
}

} // namespace controller::util
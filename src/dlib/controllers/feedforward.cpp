#include "dlib/controllers/feedforward.hpp"
#include "api.h"
#include "au/au.hpp"
#include <cmath>

namespace dlib {

Feedforward::Feedforward(FeedforwardGains gains) : gains(gains) {

}

au::Quantity<au::Volts, double> Feedforward::calculate(
    au::Quantity<au::MetersPerSecond, double> target_velocity, 
    au::Quantity<au::MetersPerSecondSquared, double> target_acceleration
) {
    auto s = std::copysign(gains.ks, target_velocity.in(au::meters_per_second));
    auto v = gains.kv * target_velocity.in(au::meters_per_second);
    auto a = gains.ka * target_acceleration.in(au::meters_per_second_squared);

    return au::volts(s + v + a);
}

}

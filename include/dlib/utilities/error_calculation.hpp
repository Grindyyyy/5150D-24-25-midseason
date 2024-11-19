#pragma once
#include "au/au.hpp"

namespace dlib {

au::Quantity<au::Meters, double> linear_error(
    au::Quantity<au::Meters, double> target, 
    au::Quantity<au::Meters, double> reading
);

au::Quantity<au::Degrees, double> sanitize_angle(
    au::Quantity<au::Degrees, double> angle
);

au::Quantity<au::Degrees, double> angular_error(
    au::Quantity<au::Degrees, double> target, 
    au::Quantity<au::Degrees, double> reading
);

template<typename U1, typename U2>
au::Quantity<U1, double> relative_target(
    au::Quantity<U1, double> start,
    au::Quantity<U2, double> target
) {
    return start + target;
}

}
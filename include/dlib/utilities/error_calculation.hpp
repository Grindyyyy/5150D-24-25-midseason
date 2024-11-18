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

template<typename Units>
au::Quantity<Units, double> relative_target(
    au::Quantity<Units, double> start,
    au::Quantity<Units, double> target
) {
    return start + target;
}

}
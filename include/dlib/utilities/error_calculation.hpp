#pragma once
#include "au/au.hpp"

namespace dlib {

au::Quantity<au::Meters, double> linear_error(
    const au::Quantity<au::Meters, double> target, 
    const au::Quantity<au::Meters, double> reading
);

au::Quantity<au::Degrees, double> sanitize_angle(
    const au::Quantity<au::Degrees, double> angle
);

au::Quantity<au::Degrees, double> angular_error(
    const au::Quantity<au::Degrees, double> target, 
    const au::Quantity<au::Degrees, double> reading
);

template<typename Units>
au::Quantity<Units, double> relative_target(
    const au::Quantity<Units, double> start,
    const au::Quantity<Units, double> target
) {
    return start + target;
}

}
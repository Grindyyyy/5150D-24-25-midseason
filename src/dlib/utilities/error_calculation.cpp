#include "dlib/utilities/error_calculation.hpp"
#include "au/au.hpp"

namespace dlib {

au::Quantity<au::Meters, double> linear_error(
    const au::Quantity<au::Meters, double> target, 
    const au::Quantity<au::Meters, double> reading
) {
    return target - reading;
}

au::Quantity<au::Degrees, double> sanitize_angle(
    const au::Quantity<au::Degrees, double> angle
) {
    return au::fmod(
        au::fmod(angle, au::degrees(360)) + au::degrees(360), 
        au::degrees(360)
    );
}

au::Quantity<au::Degrees, double> angular_error(
    const au::Quantity<au::Degrees, double> target, 
    const au::Quantity<au::Degrees, double> reading
) {
    return au::remainder(sanitize_angle(target) - reading, au::degrees(360));
}



}
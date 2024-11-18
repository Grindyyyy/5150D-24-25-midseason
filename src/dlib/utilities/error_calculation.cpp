#include "dlib/utilities/error_calculation.hpp"
#include "au/au.hpp"

namespace dlib {

au::Quantity<au::Meters, double> linear_error(
    au::Quantity<au::Meters, double> target, 
    au::Quantity<au::Meters, double> reading
) {
    return target - reading;
}

au::Quantity<au::Degrees, double> sanitize_angle(
    au::Quantity<au::Degrees, double> angle
) {
    return au::fmod(
        au::fmod(angle, au::degrees(360)) + au::degrees(360), 
        au::degrees(360)
    );
}

au::Quantity<au::Degrees, double> angular_error(
    au::Quantity<au::Degrees, double> target, 
    au::Quantity<au::Degrees, double> reading
) {
    return au::remainder(sanitize_angle(target) - reading, au::degrees(360));
}



}
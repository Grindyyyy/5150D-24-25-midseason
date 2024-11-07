#pragma once
#include "au/au.hpp"

namespace dlib {

struct ProfileSetpoint {
    au::Quantity<au::Meters, double> position;
    au::Quantity<au::MetersPerSecond, double> velocity;

    ProfileSetpoint(
        au::Quantity<au::Meters, double> position, 
        au::Quantity<au::MetersPerSecond, double> velocity
    );
};


}
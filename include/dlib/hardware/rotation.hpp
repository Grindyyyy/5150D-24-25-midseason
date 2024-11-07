#pragma once

#include "pros/rotation.hpp"
#include "au/au.hpp"

namespace dlib {

struct RotationConfig {
    int8_t port;
    au::Quantity<au::Meters, double> wheel_diameter;
    double gear_ratio;

    RotationConfig(int8_t port, au::Quantity<au::Meters, double> wheel_diameter, double gear_ratio);
};

class Rotation {
public:
    void initialize();
    
    au::Quantity<au::Revolutions, double> get_position();
    au::Quantity<au::Rpm, double> get_velocity();

    Rotation(RotationConfig config);
    
    pros::Rotation raw;
};

}
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
    
    au::Quantity<au::Meters, double> get_linear_displacement();
    au::Quantity<au::MetersPerSecond, double> get_linear_velocity();

    Rotation(RotationConfig config);
    
    au::Quantity<au::Meters, double> wheel_diameter;
    double gear_ratio;

    pros::Rotation raw;
};

}
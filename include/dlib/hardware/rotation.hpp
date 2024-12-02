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
    /**
     * @brief Initialize the Rotation sensor
     * 
     */
    void initialize();
    
    /**
     * @brief Get the position of the rotation sensor in revolutions
     * 
     * @return revolutions
     */
    au::Quantity<au::Revolutions, double> get_position();

    /**
     * @brief Get the velocity of the rotation sensor in rpm
     * 
     * @return rpm
     */
    au::Quantity<au::Rpm, double> get_velocity();
    
    /**
     * @brief Get the linear displacement of the rotation sensor
     * 
     * @return linear displacement in meters
     */
    au::Quantity<au::Meters, double> get_linear_displacement();

    /**
     * @brief Get the linear velocity of the rotation sensor
     * 
     * @return linear velocity in meters per second
     */
    au::Quantity<au::MetersPerSecond, double> get_linear_velocity();

    Rotation(RotationConfig config);
    
    const au::Quantity<au::Meters, double> wheel_diameter;
    const double gear_ratio;

    pros::Rotation raw;
};

}
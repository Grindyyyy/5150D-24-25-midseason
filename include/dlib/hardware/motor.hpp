#pragma once
#include "pros/motors.hpp"
#include "au/au.hpp"

// TODO: definitely missing some methods here...

namespace dlib {

struct MotorConfig {
    int8_t port;

    MotorConfig(int8_t port);
};

class Motor {
public:
    /**
     * @brief Get the position of the Motor in revolutions
     * 
     * @return revolutions
     */
    au::Quantity<au::Revolutions, double> get_position();

    /**
     * @brief Get the velocity of the Motor in rpm
     * 
     * @return rpm
     */
    au::Quantity<au::Rpm, double> get_velocity();

    Motor(MotorConfig config);

    pros::Motor raw;
};

}
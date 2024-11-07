#pragma once
#include "pros/motors.hpp"
#include "au/au.hpp"

namespace dlib {

struct MotorConfig {
    int8_t port;

    MotorConfig(int8_t port);
};

class Motor {
public:
    au::Quantity<au::Revolutions, double> get_position();
    au::Quantity<au::Rpm, double> get_velocity();

    Motor(MotorConfig config);

    pros::Motor raw;
};

}
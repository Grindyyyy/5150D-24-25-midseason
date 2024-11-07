#pragma once
#include "pros/motor_group.hpp"
#include "au/au.hpp"
#include <initializer_list>

namespace dlib {

struct MotorGroupConfig {
    std::vector<int8_t> ports;

    MotorGroupConfig(std::initializer_list<int8_t> ports);
    MotorGroupConfig(std::vector<int8_t>& ports);
};

class MotorGroup {
public:
    void send(double power);
    void send_voltage(au::Quantity<au::Volts, double> voltage);
    au::Quantity<au::Revolutions, double> get_position();
    au::Quantity<au::Rpm, double> get_velocity();

    MotorGroup(MotorGroupConfig config);
    
    pros::MotorGroup raw;
};

}
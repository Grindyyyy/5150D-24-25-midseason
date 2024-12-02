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
    /**
     * @brief Move the motorgroup with a joystick input
     * 
     * @param power the joystick input value
     */
    void move(const double power);

    /**
     * @brief Move the MotorGroup with a given voltage
     * 
     * @param voltage the voltage to send to the motors
     */
    void move_voltage(const au::Quantity<au::Volts, double> voltage);

    /**
     * @brief Get the MotorGroup average position in revolutions
     * 
     * @return revolutions
     */
    au::Quantity<au::Revolutions, double> get_position();

    /**
     * @brief Get the MotorGroup average velocity in rpm
     * 
     * @return rpm
     */
    au::Quantity<au::Rpm, double> get_velocity();

    MotorGroup(MotorGroupConfig config);
    
    pros::MotorGroup raw;
};

}
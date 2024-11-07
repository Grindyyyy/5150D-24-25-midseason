#include "dlib/hardware/motor_group.hpp"
#include "au/au.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include <initializer_list>


namespace dlib {

// TODO: Handle motor initialization with gearsets (currently only handled by Chassis)

MotorGroupConfig::MotorGroupConfig(
    std::initializer_list<int8_t> ports
) : 
    ports(ports)
{

}

MotorGroup::MotorGroup(MotorGroupConfig config) : raw(config.ports) {

}

void MotorGroup::send(double power) {
    this->raw.move(power);
}

void MotorGroup::send_voltage(au::Quantity<au::Volts, double> voltage) {
    auto millivolts = voltage.in(au::milli(au::volts));
    this->raw.move_voltage(millivolts);
}

au::Quantity<au::Revolutions, double> MotorGroup::get_position() {
    auto positions = this->raw.get_position_all();
    double average = 0;

    for (auto position : positions) {
        average += position;
    }

    average /= positions.size();

    auto revolutions = au::revolutions(average);
    return revolutions;
}

au::Quantity<au::Rpm, double> MotorGroup::get_velocity() {
    auto rpms = this->raw.get_actual_velocity_all();
    double average = 0;

    for (auto rpm : rpms) {
        average += rpm;
    }

    average /= rpms.size();

    auto rpm = au::rpm(average);
    return rpm;
}

}
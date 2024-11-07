#include "dlib/hardware/motor.hpp"
#include "au/au.hpp"

namespace dlib {

MotorConfig::MotorConfig(int8_t port) : port(port) {

}

Motor::Motor(MotorConfig config) : raw(config.port) {

}

au::Quantity<au::Revolutions, double> Motor::get_position() {
    auto revolutions = au::revolutions(this->raw.get_position());
    return revolutions;
}

au::Quantity<au::Rpm, double> Motor::get_velocity() {
    auto rpm = au::rpm(this->raw.get_actual_velocity());
    return rpm;
}

}
#include "dlib/hardware/rotation.hpp"
#include "au/au.hpp"

namespace dlib {


RotationConfig::RotationConfig(
    int8_t port, 
    au::Quantity<au::Meters, double> wheel_diameter, 
    double gear_ratio
) : 
    port(port) {

}

Rotation::Rotation(RotationConfig config) : raw(config.port) {

}


void Rotation::initialize() {
    this->raw.reset();
}

au::Quantity<au::Revolutions, double> Rotation::get_position() {
    auto degrees = au::centi(au::degrees)(static_cast<double>(this->raw.get_position()));
    auto revolutions = degrees.as(au::revolutions);
    return degrees;
}

au::Quantity<au::Rpm, double> Rotation::get_velocity() {
    au::Quantity<au::Rpm, double> rpm = au::rpm(this->raw.get_velocity());
    return rpm;
}

}
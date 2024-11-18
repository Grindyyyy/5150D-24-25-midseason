#include "dlib/hardware/rotation.hpp"
#include "au/au.hpp"

namespace dlib {


RotationConfig::RotationConfig(
    int8_t port, 
    au::Quantity<au::Meters, double> wheel_diameter, 
    double gear_ratio = 1
) : 
    port(port),
    wheel_diameter(wheel_diameter),
    gear_ratio(gear_ratio) {

}

Rotation::Rotation(RotationConfig config) : raw(config.port), 
    wheel_diameter(config.wheel_diameter), 
    gear_ratio(config.gear_ratio) {

}


void Rotation::initialize() {
    this->raw.reset_position();
}

au::Quantity<au::Revolutions, double> Rotation::get_position() {
    auto position = au::centi(au::degrees)(static_cast<double>(this->raw.get_position()));
    return position;
}

au::Quantity<au::Rpm, double> Rotation::get_velocity() {
    auto rpm = (au::degrees_per_second)(static_cast<double>(this->raw.get_velocity()));
    return rpm;
}

au::Quantity<au::Meters, double> Rotation::get_linear_displacement() {
    auto position = this->get_position();
    auto linear_displacement = 
        au::meters(position.in(au::revolutions) * this->gear_ratio * wheel_diameter.in(au::meters) * M_PI);

    return linear_displacement;
}

au::Quantity<au::MetersPerSecond, double> Rotation::get_linear_velocity() {
    auto velocity = this->get_velocity();
    auto linear_velocity = 
        au::meters_per_second(velocity.in(au::rps) * this->gear_ratio * wheel_diameter.in(au::meters) * M_PI);

    return linear_velocity;
}

}
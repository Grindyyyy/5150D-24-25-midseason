#include "dlib/hardware/chassis.hpp"
#include "au/au.hpp"
#include "motor_group.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"

namespace dlib {

ChassisConfig::ChassisConfig(
    MotorGroupConfig left_motors_config,
    MotorGroupConfig right_motors_config,
    pros::MotorGearset drive_gearset,
    au::Quantity<au::Rpm, double> total_rpm,
    au::Quantity<au::Meters, double> wheel_diameter
)  :
    left_motors(left_motors_config),
    right_motors(right_motors_config),
    total_rpm(total_rpm),
    wheel_diameter(wheel_diameter) {

    switch (drive_gearset) {
        case pros::MotorGearset::red: this->base_rpm = au::rpm(100); break;
        case pros::MotorGearset::green: this->base_rpm = au::rpm(200); break;
        case pros::MotorGearset::blue: this->base_rpm = au::rpm(600); break;
        case pros::MotorGearset::invalid: this->base_rpm = au::rpm(0); break;
    }
};

Chassis::Chassis(
    ChassisConfig& config
) : 
    left_motors(config.left_motors), 
    right_motors(config.right_motors), 
    base_rpm(config.base_rpm),
    total_rpm(config.total_rpm), 
    wheel_diameter(config.wheel_diameter) {

}

void Chassis::initialize() {
    left_motors.raw.tare_position_all();
    right_motors.raw.tare_position_all();

    left_motors.raw.set_encoder_units_all(pros::MotorUnits::rotations);
    right_motors.raw.set_encoder_units_all(pros::MotorUnits::rotations);
}

void Chassis::move(double power) {
    this->left_motors.send(power);
    this->right_motors.send(power);
}

void Chassis::move_voltage(au::Quantity<au::Volts, double> voltage) {
    this->left_motors.send_voltage(voltage);
    this->right_motors.send_voltage(voltage);
}

void Chassis::turn(double power) {
    this->left_motors.send(-power);
    this->right_motors.send(power);
}

void Chassis::turn_voltage(au::Quantity<au::Volts, double> voltage) {
    this->left_motors.send_voltage(-voltage);
    this->right_motors.send_voltage(voltage);
}

void Chassis::arcade(double power, double turn) {
    this->left_motors.send(power - turn);
    this->right_motors.send(power + turn);
}

au::Quantity<au::Meters, double> Chassis::revolutions_to_displacement(au::Quantity<au::Revolutions, double> revolutions) {
    // not sure if there's a unit-safe way to do this
    auto wheel_circumference = this->wheel_diameter.in(au::meters) * M_PI;
    auto linear_distance = 
        revolutions.in(au::revolutions) 
        * wheel_circumference 
        * (this->total_rpm / this->base_rpm);
    
    return au::meters(linear_distance);
}

au::Quantity<au::MetersPerSecond, double> Chassis::rpm_to_velocity(au::Quantity<au::Rpm, double> rpm) {
    // not sure if there's a unit-safe way to do this
    auto wheel_circumference = this->wheel_diameter.in(au::meters) * M_PI;
    auto linear_velocity = 
        (rpm.in(au::rpm) 
        * wheel_circumference 
        * (this->total_rpm / this->base_rpm)) / 60.0;
    
    return au::meters_per_second(linear_velocity);
}

au::Quantity<au::Meters, double> Chassis::left_motors_displacement() {
    auto revolutions = this->left_motors.get_position();
    auto displacement = 
        this->revolutions_to_displacement(revolutions);
    
    return displacement;
}

au::Quantity<au::Meters, double> Chassis::right_motors_displacement() {
    auto revolutions = this->right_motors.get_position();
    auto displacement = 
        this->revolutions_to_displacement(revolutions);

    return displacement;
}

au::Quantity<au::Meters, double> Chassis::average_motor_displacement() {
    return (this->left_motors_displacement() + this->right_motors_displacement()) / 2.0;
}

}
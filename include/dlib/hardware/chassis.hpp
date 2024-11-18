#pragma once
#include "motor_group.hpp"
#include "pros/abstract_motor.hpp"
#include <initializer_list>
namespace dlib {

struct ChassisConfig {
    MotorGroupConfig left_motors;
    MotorGroupConfig right_motors;

    pros::MotorGearset drive_gearset;

    au::Quantity<au::Rpm, double> base_rpm;
    au::Quantity<au::Rpm, double> total_rpm;
    au::Quantity<au::Meters, double> wheel_diameter;

    ChassisConfig(
        MotorGroupConfig left_motors_config,
        MotorGroupConfig right_motors_config,
        pros::MotorGearset drive_gearset,
        au::Quantity<au::Rpm, double> total_rpm,
        au::Quantity<au::Meters, double> wheel_diameter
    );
};

class Chassis {
public:
    /**
     * @brief Initialize the chassis
     * 
     * @b Example
     * @code {.cpp}
     * void opcontrol(){
     * // Construct a chassis config
     * dlib::ChassisConfig chassis_config({
	 *  {1, 2, 3},	// left motor ports
	 *  {4, 5, 6},	// right motor ports
	 *  rpm(450),	// the drivebase rpm
	 *  inches(3.25)	// the drivebase wheel diameter
	 * });
     * 
     * // Construct a chassis
	 * dlib::Chassis chassis(chassis_config);
     * 
     * // Initialize the chassis
     * chassis.initialize();
     * @endcode
    */
    void initialize();

    /**
     * @brief Move the chassis
     * 
     * @b Example
     * @code {.cpp}
     * void opcontrol(){
     * // Construct a chassis config
     * dlib::ChassisConfig chassis_config({
	 *  {1, 2, 3},	// left motor ports
	 *  {4, 5, 6},	// right motor ports
	 *  rpm(450),	// the drivebase rpm
	 *  inches(3.25)	// the drivebase wheel diameter
	 * });
     * 
     * // Construct a chassis
	 * dlib::Chassis chassis(chassis_config);
     * 
     * // Move the chassis at max forward (127)
     * chassis.move(127);
     * @endcode
    */
    void move(double power);

    void move_voltage(au::Quantity<au::Volts, double> voltage);
    
    /**
     * @brief Turn the chassis
     * 
     * @b Example
     * @code {.cpp}
     * void opcontrol(){
     * // Construct a chassis config
     * dlib::ChassisConfig chassis_config({
	 *  {1, 2, 3},	// left motor ports
	 *  {4, 5, 6},	// right motor ports
	 *  rpm(450),	// the drivebase rpm
	 *  inches(3.25)	// the drivebase wheel diameter
	 * });
     * 
     * // Construct a chassis
	 * dlib::Chassis chassis(chassis_config);
     * 
     * // Turn the chassis at max forward (127)
     * chassis.turn(127);
     * @endcode
    */
    void turn(double power);

    void turn_voltage(au::Quantity<au::Volts, double> voltage);
    /**
     * @brief Turn the chassis
     * 
     * @b Example
     * @code {.cpp}
     * void opcontrol(){
     * // Construct a chassis config
     * dlib::ChassisConfig chassis_config({
	 *  {1, 2, 3},	// left motor ports
	 *  {4, 5, 6},	// right motor ports
	 *  rpm(450),	// the drivebase rpm
	 *  inches(3.25)	// the drivebase wheel diameter
	 * });
     * 
     * // Construct a chassis
	 * dlib::Chassis chassis(chassis_config);
     * 
     * // Create a controller
     * pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
     * 
     * // Instantiate power & turn
     * double power = master.get_analog(ANALOG_LEFT_Y);
     * double turn = master.get_analog(ANALOG_RIGHT_X);
     * 
     * // Move the chassis given power & turn
     * chassis.arcade(power, turn);
     * @endcode
    */
    void arcade(double power, double turn);

    void brake();
    
    au::Quantity<au::Meters, double> revolutions_to_displacement(au::Quantity<au::Revolutions, double> revolutions);
    au::Quantity<au::MetersPerSecond, double> rpm_to_velocity(au::Quantity<au::Rpm, double> rpm);
    
    au::Quantity<au::Meters, double> left_motors_displacement();
    au::Quantity<au::Meters, double> right_motors_displacement();
    au::Quantity<au::Meters, double> average_motor_displacement();
    
    au::Quantity<au::MetersPerSecond, double> left_motors_velocity();
    au::Quantity<au::MetersPerSecond, double> right_motors_velocity();
    au::Quantity<au::MetersPerSecond, double> average_motor_velocity();

    Chassis(ChassisConfig config);

    dlib::MotorGroup left_motors;
    dlib::MotorGroup right_motors;
    
    pros::MotorGearset drive_gearset;

    au::Quantity<au::Rpm, double> base_rpm;
    au::Quantity<au::Rpm, double> total_rpm;
    au::Quantity<au::Meters, double> wheel_diameter;
};

}
#pragma once
#include "motor_group.hpp"
#include "pros/abstract_motor.hpp"
#include <initializer_list>
namespace dlib {

// chassis.hpp

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
     * @brief Move the Chassis with a joystick input
     * 
     * @param power the joystick input value
     *
     * @b Example
     * @code {.cpp}
     * // Construct a ChassisConfig
     * dlib::ChassisConfig chassis_config({
	 *  {1, 2, 3},	// left motor ports
	 *  {4, 5, 6},	// right motor ports
	 *  rpm(450),	// the drivebase rpm
	 *  inches(3.25)	// the drivebase wheel diameter
	 * });
     * 
     * // Construct a Chassis
	 * dlib::Chassis chassis(chassis_config);
     * 
     * // Move the chassis at max forward (127)
     * chassis.move(127);
     * @endcode
    */
    void move(const int32_t power);

    /**
     * @brief Move the Chassis with a given voltage
     * 
     * @param voltage the voltage to send to the motors
     */
    void move_voltage(const au::Quantity<au::Volts, double> voltage);
    
    /**
     * @brief Turn the Chassis with a joystick input
     *
     * @param power the joystick input value
     *
     * @b Example
     * @code {.cpp}
     * // Construct a Chassis config
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
    void turn(const int32_t power);

    /**
     * @brief Turn the chassis with a given voltage
     * 
     * @param voltage the voltage to send to the motors
     */
    void turn_voltage(const au::Quantity<au::Volts, double> voltage);
    
    /**
     * @brief Turn the Chassis
     * 
     * @b Example
     * @code {.cpp}
     * // Construct a ChassisConfig
     * dlib::ChassisConfig chassis_config({
	 *  {1, 2, 3},	// left motor ports
	 *  {4, 5, 6},	// right motor ports
	 *  rpm(450),	// the drivebase rpm
	 *  inches(3.25)	// the drivebase wheel diameter
	 * });
     * 
     * // Construct a Chassis
	 * dlib::Chassis Chassis(chassis_config);
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
    void arcade(const int32_t power, const int32_t turn);

    /**
     * @brief Brake the Chassis
     * 
     */
    void brake();
    
    /**
     * @brief Convert from motor revolutions to linear displacement for the Chassis
     * 
     * @param revolutions the number of motor revolutions
     * @return linear displacement
     */
    au::Quantity<au::Meters, double> revolutions_to_displacement(const au::Quantity<au::Revolutions, double> revolutions) const;
    
    /**
     * @brief Convert from motor rpm to linear velocity for the Chassis
     * 
     * @param rpm 
     * @return linear velocity
     */
    au::Quantity<au::MetersPerSecond, double> rpm_to_velocity(const au::Quantity<au::Rpm, double> rpm) const;
    
    /**
     * @brief Get the average linear displacement of the left side of the Chassis
     * 
     * @return the linear displacement of the left side of the Chassis
     */
    au::Quantity<au::Meters, double> left_motors_displacement();

    /**
     * @brief Get the average linear displacement of the right side of the Chassis
     * 
     * @return the linear displacement of the right side of the Chassis
     */
    au::Quantity<au::Meters, double> right_motors_displacement();

    /**
     * @brief Get the forward linear displacment of the Chassis
     * 
     * @return the forward linear displacement of the Chassis
     */
    au::Quantity<au::Meters, double> forward_motor_displacement();
    
    /**
     * @brief Get the linear velocity of the left side of the Chassis
     * 
     * @return the linear velocity of the left side of the Chassis
     */
    au::Quantity<au::MetersPerSecond, double> left_motors_velocity();

    /**
     * @brief Get the linear velocity of the right side of the Chassis
     * 
     * @return the linear velocity of the right side of the Chassis
     */
    au::Quantity<au::MetersPerSecond, double> right_motors_velocity();

    /**
     * @brief Get the forward linear velocity of the Chassis
     * 
     * @return the forward linear velocity of the Chassis
     */
    au::Quantity<au::MetersPerSecond, double> forward_motor_velocity();

    Chassis(ChassisConfig config);

    dlib::MotorGroup left_motors;
    dlib::MotorGroup right_motors;
    
    pros::MotorGearset drive_gearset;

    au::Quantity<au::Rpm, double> base_rpm;
    au::Quantity<au::Rpm, double> total_rpm;
    au::Quantity<au::Meters, double> wheel_diameter;
};

}
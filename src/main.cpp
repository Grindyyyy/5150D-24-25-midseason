#include "main.h"
#include "au/au.hpp"
#include "dlib/controllers/error_derivative_settler.hpp"
#include "dlib/controllers/feedforward.hpp"
#include "dlib/dlib.hpp"
#include "dlib/kinematics/odometry.hpp"
#include "dlib/trajectories/trapezoid_profile.hpp"
#include "dlib/utilities/error_calculation.hpp"
#include "intake.hpp"
#include "misc_pneumatics.hpp"
#include "mogo.hpp"
#include "lift.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include <memory>
#include <mutex>

using namespace au;

enum class Alliance {
    Red,
    Blue
};

bool redirect = false;

// Robot class
class Robot {
public:
	dlib::Chassis chassis;
	dlib::Imu imu;

	std::unique_ptr<Intake> intake;
	Mogo mogo;
	Lift lift;
	MiscPneumatics pneumatics;

	dlib::Pid<Inches> move_pid;
	dlib::ErrorDerivativeSettler<Meters> move_settler;

	dlib::Pid<Inches> left_move_pid;
	dlib::ErrorDerivativeSettler<Meters> left_move_settler;

	dlib::Pid<Inches> right_move_pid;
	dlib::ErrorDerivativeSettler<Meters> right_move_settler;

	dlib::Pid<Degrees> turn_pid;
	dlib::ErrorDerivativeSettler<Degrees> turn_settler;

	dlib::Pid<Inches> boomerang_move_pid;
	dlib::ErrorDerivativeSettler<Inches> boomerang_move_settler;

	dlib::Pid<Degrees> boomerang_turn_pid;
	dlib::ErrorDerivativeSettler<Degrees> boomerang_turn_settler;

	dlib::Feedforward<Meters> feedforward;
	dlib::Odometry odom = dlib::Odometry();
	std::unique_ptr<pros::Task> odom_updater = nullptr;

	// ------------------------------ //
	// Robot Class Methods //
	// ------------------------------ //

	// Run this in initialize function
	// Makes sure that everything is set up inside of robot class for autonomous
	void initialize() {
		chassis.initialize();

		chassis.left_motors.raw.set_gearing_all(pros::MotorGearset::blue);
		chassis.right_motors.raw.set_gearing_all(pros::MotorGearset::blue);
		
		imu.initialize();
	}

	// Only move the left side of the drivebase using PID
	void move_left_with_pid(Quantity<Meters, double> displacement){
		auto start_displacement = chassis.left_motors_displacement();
		auto target_displacement = dlib::relative_target(start_displacement, displacement);

		left_move_pid.reset();
		left_move_settler.reset();

		while(!left_move_settler.is_settled(left_move_pid.get_error(),left_move_pid.get_derivative())){
			auto error = dlib::linear_error(target_displacement,chassis.left_motors_displacement());
			std::cout << "error: " << error.in(inches) << std::endl;
			auto voltage = left_move_pid.update(error, milli(seconds)(20));

			chassis.left_motors.move_voltage(voltage);

			pros::delay(20);
		}
		std::cout << "left movement settled" << "\n";
		std::cout << "left displacement: " << chassis.left_motors_displacement().in(inches) << std::endl;
		chassis.move(0);
	}

	// Only move the right side of the drivetrain using PID
	void move_right_with_pid(Quantity<Meters, double> displacement){
		auto start_displacement = chassis.right_motors_displacement();
		auto target_displacement = dlib::relative_target(start_displacement, displacement);

		right_move_pid.reset();
		right_move_settler.reset();

		while(!right_move_settler.is_settled(right_move_pid.get_error(),right_move_pid.get_derivative())){
			auto error = dlib::linear_error(target_displacement,chassis.right_motors_displacement());
			auto voltage = right_move_pid.update(error, milli(seconds)(20));

			chassis.right_motors.move_voltage(voltage);

			pros::delay(20);
		}
	}

	// Move the chassis using PID and feedforward
	void move_ffwd(double displacement, double max_time = 99999, double max_velocity = 1.6){
		auto start_displacement = chassis.forward_motor_displacement();
		dlib::TrapezoidProfile<Meters> profile {
			meters_per_second_squared(3),
			meters_per_second(max_velocity),
			inches(displacement)
		};

		move_pid.reset();
		move_settler.reset();

		auto elapsed_time = 0;
		auto current_time = pros::millis();
		auto start_time = pros::millis();
	
		while (true) {
			current_time = pros::millis();
			elapsed_time = current_time - start_time;

			auto setpoint = profile.calculate(milli(seconds)(elapsed_time));

			auto current_position = chassis.forward_motor_displacement();
			auto target_position = dlib::relative_target(start_displacement, setpoint.position);

			auto error = dlib::linear_error(target_position, current_position);

			auto pid_voltage = move_pid.update(error, milli(seconds)(20));
			auto ff_voltage = feedforward.calculate(setpoint.velocity, setpoint.acceleration);
			
			if(profile.stage(milli(seconds)(elapsed_time)) == dlib::TrapezoidProfileStage::Decelerating){
				chassis.move_voltage(volts(0));
			}

			if(profile.stage(milli(seconds)(elapsed_time)) == dlib::TrapezoidProfileStage::Done){
				break;
			}

			chassis.move_voltage(ff_voltage + pid_voltage);
			
			pros::delay(20);
		}
		chassis.move(0);
	}

	// Turn the chassis using PID
	void turn_with_pid(double heading, double max_time, int max_voltage) {
		auto target_heading = au::degrees(heading);

		double elapsed_time = 0;	

		turn_pid.reset();
		turn_settler.reset();

		auto voltage = volts(0.0);

		auto last_error = degrees(heading);
		auto reading = imu.get_rotation();

		while (!turn_settler.is_settled(turn_pid.get_error(), turn_pid.get_derivative())) {

			auto error = dlib::angular_error((au::degrees)(heading), imu.get_rotation());

			if (elapsed_time > max_time) {
				chassis.brake();
				std::cout << "timed out" << "\n";
				break;
			}

			voltage = turn_pid.update(error, milli(seconds)(20));
			std::cout << voltage << std::endl;
			
			if(au::abs(voltage) > milli(volts)(max_voltage)){
				if(voltage > volts(0)){
					voltage = milli(volts)(max_voltage);
				}
				else{
					voltage = milli(volts)(-max_voltage);
				}
			}

			chassis.turn_voltage(voltage);

			last_error = error;

			elapsed_time += 20;
			pros::delay(20);
		}

		chassis.left_motors.raw.brake();
		chassis.right_motors.raw.brake();
	}

	// Use arctan formula to turn to a relative point
	void turn_to_point(double x, double y, bool reverse = false, double max_time = 15000, int max_voltage = 12000) {
		auto point = dlib::Vector2d(inches(x), inches(y));
		auto angle = odom.angle_to(point, reverse);

		turn_with_pid(angle.in(degrees), max_time,max_voltage);
	}

	// Use pythagorean formula to move to a relative point
	void move_to_point(double x, double y, bool reverse = false, double max_time = 3000, double move_max_velocity = 1.6, int turn_max_voltage = 12000) {
		auto point = dlib::Vector2d(inches(x), inches(y));
		turn_to_point(x, y, reverse,max_time, turn_max_voltage);
		auto displacement = odom.displacement_to(point);
		if(reverse){
			displacement = -displacement;
		}
		move_ffwd(displacement.in(inches), max_time, move_max_velocity);
	}

	void move_with_boomerang(double x, double y, double theta, double dlead) {
		dlib::Pose2d target(inches(x),inches(y),degrees(theta));

		boomerang_move_pid.reset();
		boomerang_move_settler.reset();
		boomerang_turn_pid.reset();
		boomerang_turn_settler.reset();

		// calc distance from center point
		double d = std::sqrt(std::pow((odom.get_position().x.in(inches)) - x,2) + std::pow((odom.get_position().y.in(inches)) - x,2));

		while(!boomerang_move_settler.is_settled(boomerang_move_pid.get_error(),boomerang_move_pid.get_derivative())
		&& !boomerang_turn_settler.is_settled(boomerang_turn_pid.get_error(),boomerang_turn_pid.get_derivative())){
			double carrot_x = target.x.in(inches) - d * cos(target.theta.in(radians)) * dlead;
			double carrot_y = target.y.in(inches) - d * sin(target.theta.in(radians)) * dlead;

			dlib::Vector2d carrot(inches(carrot_x), inches(carrot_y));

			auto current_position = chassis.forward_motor_displacement();
			auto target_position = dlib::relative_target(current_position,odom.displacement_to(carrot));

			auto linear_error = dlib::linear_error(target_position, current_position);
			auto linear_power = boomerang_move_pid.update(target_position,milli(seconds)(20));

			auto current_angle = imu.get_rotation();
			auto target_angle = target.theta;

			auto angular_error = dlib::angular_error(target_angle,current_angle);
			auto angular_power = boomerang_turn_pid.update(angular_error,milli(seconds)(20));

			auto left_power = linear_power + angular_power;
			auto right_power = linear_power - angular_power;

			chassis.left_motors.move_voltage(left_power);
			chassis.right_motors.move_voltage(right_power);

			pros::delay(20);
		}
	}

	// Start the odom task responsible for updating coordinates
	// Odom task
	void start_odom() {
		odom_updater = std::make_unique<pros::Task>([this]() {
			while (true) {
				odom.update(
					chassis.left_motors_displacement(), 
					chassis.right_motors_displacement(), 
					ZERO,
					imu.get_rotation()
				);

				pros::delay(20);
			}
		});
	}
};

// ------------------------------ //
// Chassis + IMU Configurations //
// ------------------------------ //

// Create a config for everything used in the Robot class
dlib::ChassisConfig chassis_config {
	{{18, 19, 17}},	// left motors
	{{-14, -16, -11}},	// right motors
	pros::MotorGearset::blue,
	rpm(450),	// the drivebase rpm
	inches(3.25)	// the drivebase wheel diameter
};

dlib::ImuConfig imu_config {
	15,	// imu port
	1.0132	// optional imu scaling constant
};

// ------------------------------ //
// Move PID Gains + Settler //
// ------------------------------ //

// Chassis move PID gains
dlib::PidConfig move_pid_gains {
	{50, 	// kp, porportional gain
	0, //0, 	// ki, integral gain
	0},
	volts(12) //11.8	// kd, derivative gain
};

dlib::PidConfig boomerang_move_pid_gains {
	{0,0,0},
	volts(12)
};

dlib::PidConfig boomerang_turn_pid_gains {
	{0,0,0},
	volts(12)
};

// Left DB move PID gains
dlib::PidConfig left_move_pid_gains {
	{500,
	0,
	0},
	volts(12)
};

// Right DB move PID gains
dlib::PidConfig right_move_pid_gains {
	{1000,
	0,
	10},
	volts(12)
};

// Chassis move PID settler
dlib::ErrorDerivativeSettler<Meters> move_pid_settler {
	inches(0.5),		// error threshold, the maximum error the pid can settle at
	meters_per_second(0.005) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

dlib::ErrorDerivativeSettler<Inches> boomerang_move_settler {
	inches(0.5),
	meters_per_second(0.005)
};

dlib::ErrorDerivativeSettler<Degrees> boomerang_turn_settler {
	degrees(1),
	degrees_per_second(0.2)
};

// Left DB move PID settler
dlib::ErrorDerivativeSettler<Meters> left_move_pid_settler {
	inches(0.5),		// error threshold, the maximum error the pid can settle at
	meters_per_second(0.005) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

// Right DB move PID settler
dlib::ErrorDerivativeSettler<Meters> right_move_pid_settler {
	inches(0.5),		// error threshold, the maximum error the pid can settle at
	meters_per_second(0.005) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};


// Chassis move Feedforward gains
dlib::Feedforward<Meters> feedforward {
	{
		0.98304353748,
	5.35,
	1
	}
};

// ------------------------------ //
// Turn PID Gains + Settler //
// ------------------------------ //

// Chassis turn PID gains
dlib::PidConfig turn_pid_gains {
	{
	50,
	0,
	3.1
	},
	volts(12)
};

// Chassis turn PID settler
dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler {
	degrees(1),		// error threshold, the maximum error the pid can settle at
	degrees_per_second(0.2)	// derivative threshold, the maximum instantaneous error over time the pid can settle at
};

// Intake object
auto intake = std::make_unique<Intake>(
	-8,
	3,
	4
);

// Mogo object
Mogo mogo(
	'H',
	false
);

// Lift object
Lift lift(
	-1,
	10
);

// MiscPneumatics object
MiscPneumatics misc_pneumatics(
	'G',
	false,
	'E',
	false,
	'C',
	false
);

// Robot object
// Put everything in this class
Robot robot = {
	chassis_config,
	imu_config,
	std::move(intake),
	mogo,
	lift,
	misc_pneumatics,
	move_pid_gains,
	move_pid_settler,
	left_move_pid_gains,
	left_move_pid_settler,
	right_move_pid_gains,
	right_move_pid_settler,
	turn_pid_gains,
	turn_pid_settler,
	boomerang_move_pid_gains,
	boomerang_move_settler,
	boomerang_turn_pid_gains,
	boomerang_turn_settler,
	feedforward
};

Alliance alliance;

// ------------------------------ //
// Autonomous //
// ------------------------------ //

// 4 rings (1 on alliance stake, 3 on mogo) = 8 points
void blue_awp_ring_side(){
	lift.lift_move(-20);
	robot.move_ffwd(-13.5);
	robot.move_to_point(-13.5, 4.5,true,1500);
	robot.intake->max();
	pros::delay(500);
	robot.move_ffwd(10);
	robot.intake->stop();
	robot.move_to_point(10.5,-35.7,true,3000,.85);
	mogo.set_clamp_state(true);
	pros::delay(200);
	robot.intake->max();
	robot.move_to_point(33.6,-37.5);
	robot.move_ffwd(-15);
	robot.move_to_point(23.9,-48.4);
	robot.move_ffwd(-24);
	pros::delay(1200);
	// lift up
	lift.lift_move(127);
	robot.move_to_point(-1.8,-47,false,800,1.8);
	// -1.8 -38
}

// 4 rings (1 on alliance stake, 3 on mogo) = 8 points
void red_awp_ring_side(){
	alliance = Alliance::Red;
	robot.move_ffwd(-13.5);
	robot.move_to_point(-13.5, -4.5,true,1500,.85);
	robot.intake->max();
	pros::delay(500);
	robot.move_ffwd(10);
	robot.intake->stop();
	robot.move_to_point(8.5,35.7,true,3000,.9);
	mogo.set_clamp_state(true);
	pros::delay(200);
	robot.intake->max();
	robot.move_to_point(31.6,36.5);
	robot.move_ffwd(-13);
	robot.move_to_point(23.9,48.4);
	robot.move_ffwd(-24);
	pros::delay(1200);
	// lift up
	lift.lift_move(127);
	robot.move_to_point(2,42,false,1200,1.8);
	//-.8 34.7
	// doinker
}

void blue_elim_ring_side(){
	lift.lift_move(127);
	robot.move_to_point(-25,9.3,true,3000,.85);
	mogo.set_clamp_state(true);
	pros::delay(300);
	robot.intake->max();
	pros::delay(400);
	robot.move_to_point(-38.8,-3.1);
	pros::delay(400);
	robot.move_to_point(-42.3,-15.3);
	pros::delay(400);
	robot.move_ffwd(-24);
	robot.move_to_point(-27.6,-9.5);
	// -7.7 29.5
	// -12
}

void red_elim_ring_side(){
	lift.lift_move(-20);
	robot.move_to_point(-25,-9.3,true,3000,.85);
	mogo.set_clamp_state(true);
	pros::delay(300);
	robot.intake->max();
	pros::delay(400);
	robot.move_to_point(-38.8,3.1);
	pros::delay(400);
	robot.move_to_point(-42.3,15.3);
	pros::delay(400);
	robot.move_ffwd(-24);
	robot.move_to_point(-27.6,9.5);
	// -7.7 29.5
	// -12
}

// 3 rings (1 on 5th mogo, 2 on safe mogo) = 7 points
void blue_elim_goal_side(){
	// -25.1 -10.2 true
	robot.move_to_point(-25.1, -10.2, true,3000,.85);
	mogo.clamp();
	pros::delay(300);
	robot.intake->max();
	pros::delay(400);
	robot.move_to_point(-28, 11.5);
	// -10 -16.8
	robot.move_to_point(-10, -16.8);
	misc_pneumatics.set_doinker_state(true);
	pros::delay(300);
	robot.move_ffwd(-10);
	misc_pneumatics.set_doinker_state(false);
	robot.move_to_point(-12.7,-22);

	robot.move_to_point(-4,-31.1);
	robot.move_to_point(4.9,-.5);
	misc_pneumatics.set_doinker_state(true);
	robot.move_to_point(2,18.6);
	robot.chassis.turn_voltage(volts(-12));
	pros::delay(1250);
	robot.chassis.brake();
	// -4 -31.1
	// 4.9 -.5
	// doinker
	// 2 18.6
	// turn for 500ms
}

void blue_awp_goal_side(){
	robot.move_to_point(-25.1, -10.2, true,3000,.85);
	mogo.clamp();
	pros::delay(300);
	robot.intake->max();
	pros::delay(400);
	robot.move_to_point(-27, 5.5);
	pros::delay(2000);
	robot.intake->stop();
	robot.move_to_point(-32.5,-19.5);
}

void evolution_auton(){
	robot.move_ffwd(-13.5);
	robot.move_to_point(-13.5, -4.5,true,1500,.85);
	robot.intake->max();
	pros::delay(500);
	robot.move_ffwd(10);
	robot.intake->stop();
	robot.move_to_point(8.5,35.7,true,3000,.9);
	mogo.set_clamp_state(true);
	pros::delay(200);
	robot.intake->max();
	robot.move_to_point(31.6,38.5);
	robot.move_ffwd(-13);
	pros::delay(2000);
	// lift up
	lift.lift_move(127);
	robot.move_to_point(2,42,false,1200,1.8);
}

// 3 rings (1 on 5th mogo, 2 on safe mogo) = 7 points
void red_elim_goal_side(){
	robot.move_to_point(-25.1, 10.2, true,3000,.85);
	mogo.clamp();
	pros::delay(300);
	robot.intake->max();
	pros::delay(400);
	robot.move_to_point(-28, -11.5);
	pros::delay(1000);
	robot.intake->stop();
	robot.move_to_point(-16.5, 21.7);
	misc_pneumatics.set_doinker_state(true);
	pros::delay(300);
	robot.move_ffwd(-10);
	misc_pneumatics.set_doinker_state(false);
	robot.move_to_point(-12.7,22);

	robot.move_to_point(-25.5,-16.4);
	robot.move_to_point(-8.2,-23.1);
	misc_pneumatics.set_doinker_state(true);
	robot.move_to_point(.5,-21);
	robot.chassis.turn_voltage(volts(-12));
	pros::delay(1250);
	robot.chassis.brake();
}

void red_awp_goal_side(){
	robot.move_to_point(-25.1, 10.2, true,3000,.85);
	mogo.clamp();
	pros::delay(300);
	robot.intake->max();
	pros::delay(400);
	robot.move_to_point(-27, -5.5);
	pros::delay(1000);

	robot.intake->stop();
	// lift
	lift.lift_move(127);
	robot.move_to_point(-32.5,22.5);
	// -32.5, 19.5
}

// Skills
// 19 rings (1 on alliance stake, 6 on mogo 1, 6 on mogo 2, 6 on mogo 3, all in corner + 1 more) = 47 pts
void skills(){
	robot.intake->max();
	pros::delay(750);
	robot.move_ffwd(16.5);
	// grab mogo
	robot.move_to_point(16.5, 22.1, true, 10000, 0.85);
	mogo.set_clamp_state(true);
	// ring 1
	robot.move_to_point(40, 22.7);
	pros::delay(500);
	// ring 2
	robot.move_to_point(58,52.6);
	pros::delay(500);
	// ring 3
	robot.move_to_point(42.2, 47.8);
	pros::delay(500);
	// rings 4,5
	robot.move_to_point(5.2, 44.9,false,10000,0.3);
	pros::delay(1500);
	// back up
	robot.move_ffwd(-20);
	// ring 6
	robot.move_to_point(20.4, 53.7);
	pros::delay(500);
	// drop mogo
	robot.move_to_point(6.8, 58.9,true,10000,0.3);
	mogo.set_clamp_state(false);
	robot.intake->stop();
	// move to next mogo
	robot.move_to_point(19,50.9,false,1000);
	robot.move_to_point(14.5,22.9,true);
	robot.move_to_point(20.6,-25.4,true,10000,0.85);
	mogo.set_clamp_state(true);
	pros::delay(300);
	robot.intake->max();
	//ring 1
	robot.move_to_point(38.5,-23.9);
	pros::delay(500);
	// ring 2
	robot.move_to_point(58.7,-54);
	pros::delay(500);
	// ring 3
	robot.move_to_point(37.4,-48.6);
	pros::delay(500);
	// rings 4,5
	robot.move_to_point(5.1, -45.4,false,10000,0.3);
	pros::delay(1500);
	// back up
	robot.move_ffwd(-23);
	// ring 6
	robot.move_to_point(20.1, -58.7);
	pros::delay(500);
	// drop mogo
	robot.move_to_point(7.2, -60.2, true, 10000, 0.3);
	mogo.set_clamp_state(false);
	robot.intake->stop();
	// third quad
	robot.move_to_point(60.5,-43.4);
	robot.intake->max();
}

void left_pid_test(){
	robot.move_left_with_pid(inches(-24));
}

void test(){
	pros::delay(5000);
	mogo.set_clamp_state(true);
	robot.intake->max();
	pros::delay(1000);
	mogo.set_clamp_state(false);
	robot.intake->stop();
	pros::delay(10000);
	mogo.set_clamp_state(true);
	pros::delay(300);
	robot.intake->max();
}

// This serves to test our tasks in autonomous
void test_tasks(){
	robot.intake->auton_stick = true;
	while(true){
		if(!robot.intake->intake_filter()){
			robot.intake->move(127);
		}
		pros::delay(20);
	}
}

// Selector
// We use the Robodash library to control selection of our autons.
rd::Selector selector({
    {"Red Ring AWP", red_awp_ring_side},
	{"Red Goal AWP", red_awp_goal_side},
	{"Blue Ring AWP", blue_awp_ring_side},
	{"Blue Goal AWP", blue_awp_goal_side},
	{"Evolution AWP", evolution_auton},
	{"Skills", skills},
	{"Red Elim 4 Ring", red_elim_ring_side},
	{"Blue Elim 4 Ring", blue_elim_ring_side},
	{"Red Elim Goal", red_elim_goal_side},
	{"Blue Elim Goal", blue_elim_goal_side},
});

rd::Console console;

pros::Optical intake_sensor(3);
pros::Motor intake_motor(-8);

bool detected = false;
pros::Mutex mutex;

void initialize() {
	robot.initialize();
	robot.start_odom();
	selector.focus();

	robot.chassis.left_motors.raw.tare_position_all();
	robot.chassis.right_motors.raw.tare_position_all();


	// Intake Filter Task //
	// ------------------ //
	// when the sensor detects a ring, enable one of two macros:
	// redirect: redirect a alliance color ring onto wall stake mech
	// fling: fling non-alliance rings so they dont get scored
	// shouldn't interfere with auton

	
	

	// Screen Task //
	// ----------- //
	// This task intends to print the odometry coordinates to the brain screen.
	pros::Task screen_task([&]() {
        while (true) {
			dlib::Pose2d pose = robot.odom.get_position();

			console.clear();
            // print robot location to the brain screen
            console.printf("X: %f", pose.x.in(inches)); // x
            console.printf("Y: %f", pose.y.in(inches)); // y
            console.printf("Theta: %f",pose.theta.in(degrees));

			//std::cout << pose.theta.in(degrees) << "\n"; // heading
			// 0.05
            // delay to save resources
            pros::delay(20);
        }
	});
	// potential bad bad (no mutex)
	
}

void disabled() {}

void competition_initialize() {}

// Add selected autons in here.
// Prints the completion time of autons.
void autonomous() {
	console.focus();

	auto start_time = pros::millis();
	//evolution_auton();
	selector.run_auton();
	robot.chassis.brake();
	auto elapsed_time = pros::millis() - start_time;

	std::cout << "settled in " << elapsed_time << " milliseconds." << "\n";
	dlib::Pose2d position = robot.odom.get_position();
	std::cout << position.x.in(inches) << ", " << position.y.in(inches) << ", " << position.theta.in(degrees) << "\n";
}	

void opcontrol() {
	lift.lift_move(-40);
	intake_sensor.set_led_pwm(20);
	// Try arcade drive control!
	pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
	alliance = Alliance::Red;
	robot.chassis.left_motors.raw.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	robot.chassis.right_motors.raw.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	
	while(true){

		// ------------------- //
		// Chassis Controls
		// ------------------- //
		// get power and turn
		double power = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X);

		robot.chassis.arcade(power,turn);

		// ------------------- //
		// Intake Controls
		// ------------------- //

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake_motor.move(127);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			intake_motor.move(-127);
		}
		else{
			intake_motor.move(0);
		}

		// ------------------- //
		// Mogo Controls
		// ------------------- //
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
			mogo.reverse_clamp_state();
		}

		// ------------------- //
		// Lift Controls
		// ------------------- //
		
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
			lift.lift_move(127);
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
			lift.lift_move(-20);
		}

		// ------------------- //
		// Doinker Controls
		// ------------------- //
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			misc_pneumatics.doinker_toggle = !misc_pneumatics.doinker_toggle;
			misc_pneumatics.set_doinker_state(misc_pneumatics.doinker_toggle);
		}

		// ------------------- //
		// Indexer Controls
		// ------------------- //
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			misc_pneumatics.set_indexer_state(true);
		}
		else{
			misc_pneumatics.set_indexer_state(false);
		}

		pros::delay(20);
	}
	
	
}
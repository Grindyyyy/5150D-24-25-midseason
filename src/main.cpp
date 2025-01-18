#include "main.h"
#include "au/au.hpp"
#include "dlib/controllers/error_derivative_settler.hpp"
#include "dlib/controllers/feedforward.hpp"
#include "dlib/dlib.hpp"
#include "dlib/hardware/rotation.hpp"
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

// Robot class
class Robot {
public:
	dlib::Chassis chassis;
	dlib::Imu imu;
	dlib::Rotation left_odom;
	dlib::Rotation right_odom;

	Intake intake;
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

	dlib::Feedforward<Meters> move_feedforward;
	dlib::Feedforward<Degrees> turn_feedforward;
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

		left_odom.initialize();
		right_odom.initialize();
		
		imu.initialize();
	}

	// Move the chassis using PID and feedforward
	void move_ffwd(double displacement, double max_time = 99999, double max_velocity = 1.6, double early_exit_error = -100){
		auto start_displacement = chassis.forward_motor_displacement();
		dlib::TrapezoidProfile<Meters> profile {
			meters_per_second_squared(3),
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
			std::cout << error << std::endl;
			/*if(early_exit_error > au::abs(error).in(meters)){
				std::cout << "early exited at " << error << " error." << std::endl;
				std::cout << "exited in " << elapsed_time << "ms" << std::endl;
				early_exit = true;
			}*/

			auto pid_voltage = move_pid.update(error, milli(seconds)(20));
			auto ff_voltage = move_feedforward.calculate(setpoint.velocity, setpoint.acceleration);
			
			if(profile.stage(milli(seconds)(elapsed_time)) == dlib::TrapezoidProfileStage::Decelerating){
				chassis.move_voltage(volts(0));
			}

			if(profile.stage(milli(seconds)(elapsed_time)) == dlib::TrapezoidProfileStage::Done){
				break;
			}

			chassis.move_voltage(ff_voltage + pid_voltage);
			
			pros::delay(20);
		}
		chassis.move_voltage(volts(0));
	}

	void turn_with_pid(Quantity<Degrees, double> heading) {

		turn_pid.reset();
		turn_settler.reset();

		while (!turn_settler.is_settled(turn_pid.get_error(), turn_pid.get_derivative())) {
			auto error = dlib::angular_error(heading, imu.get_rotation());
			auto voltage = turn_pid.update(error, milli(seconds)(20));
			//std::cout << "error: " << error << std::endl;
			//std::cout << "voltage: " << voltage << std::endl;
			chassis.turn_voltage(voltage);

			pros::delay(20);
		}
		chassis.brake();
	}

	void turn_with_pid(double heading) {
		turn_with_pid(degrees(heading));
	}

	// Use arctan formula to turn to a relative point
	void turn_to_point(double x, double y, bool reverse = false, double max_time = 15000, int max_voltage = 5500) {
		auto point = dlib::Vector2d(inches(x), inches(y));
		auto angle = odom.angle_to(point, reverse);

		turn_with_pid(angle.in(degrees));
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
			std::cout << left_power.in(volts) << std::endl;
			std::cout << right_power.in(volts) << std::endl;
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
					//chassis.left_motors_displacement(), 
					//chassis.right_motors_displacement(), 
					left_odom.get_linear_displacement(),
					right_odom.get_linear_displacement(),
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
	{{-17,-16,-15}},
	{{14,13,11}},
	
	//{{18, 19, 17}},	// left motors
	//{{-14, -16, -11}},	// right motors
	pros::MotorGearset::blue,
	rpm(450),	// the drivebase rpm
	inches(3.25)	// the drivebase wheel diameter
};

dlib::ImuConfig imu_config {
	19,	// imu port
	1.011	// optional imu scaling constant
};

dlib::RotationConfig left_odom_config {
	1,
inches(2),
	1
};

dlib::RotationConfig right_odom_config {
	-2,
	inches(2),
	1
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
	{10,0,0},
	volts(12)
};

dlib::PidConfig boomerang_turn_pid_gains {
	{10,0,0},
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
dlib::Feedforward<Meters> move_feedforward {
	{
		0.98304353748,
	5.35,
	1
	}
};

dlib::Feedforward<Degrees> turn_feedforward {
	{
		0,0,0
	}
};

// ------------------------------ //
// Turn PID Gains + Settler //
// ------------------------------ //

// Chassis turn PID gains
dlib::PidConfig turn_pid_gains {
	{
	48,0,2.5
	},
	volts(8)
};

// Chassis turn PID settler
dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler {
	degrees(1),		// error threshold, the maximum error the pid can settle at
	degrees_per_second(1.5)	// derivative threshold, the maximum instantaneous error over time the pid can settle at
};

Intake intake(
	-20,
	12
);

// Mogo object
Mogo mogo(
	'H',
	false
);

// Lift object
Lift lift(
	-1,
	10,
	'C'
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
	left_odom_config,
	right_odom_config,
	intake,
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
	move_feedforward,
	turn_feedforward
};

Alliance alliance;

// ------------------------------ //
// Autonomous //
// ------------------------------ //

// 4 rings (1 on alliance stake, 3 on mogo) = 8 points
void blue_awp_ring_side(){
	lift.lift_move(-20);
	alliance = Alliance::Red;
	robot.move_ffwd(-13.5);
	robot.move_to_point(-13.5, 4.5,true,1500,.85);
	intake.max();
	pros::delay(500);
	robot.move_ffwd(10);
	intake.stop();
	robot.move_to_point(8.5,-35.7,true,3000,.9);
	mogo.set_clamp_state(true);
	pros::delay(200);
	intake.max();
	robot.move_to_point(23.9,-48.4);
	robot.move_ffwd(-24);
	robot.move_to_point(31.6,-36.5);
	robot.move_ffwd(-13);
	pros::delay(1200);
	// lift up
	lift.lift_move(127);
	robot.move_to_point(2,-42,false,1200,1.8);
	lift.lift_move(-20);

	// -1.8 -38
}

// 4 rings (1 on alliance stake, 3 on mogo) = 8 points
void red_awp_ring_side(){
	lift.lift_move(-20);
	alliance = Alliance::Red;
	robot.move_ffwd(-13.5);
	robot.move_to_point(-13.5, -4.5,true,1500,.85);
	intake.max();
	pros::delay(500);
	robot.move_ffwd(10);
	intake.stop();
	robot.move_to_point(8.5,35.7,true,3000,.9);
	mogo.set_clamp_state(true);
	pros::delay(200);
	intake.max();
	robot.move_to_point(21.5,49.4);
	robot.move_ffwd(-24);
	robot.move_to_point(31.6,36.5);
	robot.move_ffwd(-13);
	pros::delay(1200);
	// lift up
	lift.lift_move(127);
	robot.move_to_point(2,42,false,1200,1.8);
	lift.lift_move(-20);
	//-.8 34.7
	// doinker
}

void blue_elim_ring_side(){
	robot.move_to_point(-26.5,9.3,true,3000,.78);
	mogo.set_clamp_state(true);
	pros::delay(300);
	intake.max();
	pros::delay(400);
	robot.move_to_point(-38.8,-3.1);
	robot.move_to_point(-42,-18.3);
	robot.move_ffwd(-24);
	robot.move_to_point(-27.6,-13.5);
	robot.move_ffwd(-12);
	robot.move_to_point(0.6,-24.5);
	misc_pneumatics.set_doinker_state(true);
	pros::delay(200);
	intake.stop();
	robot.turn_to_point(-19.53,-17);
	misc_pneumatics.set_doinker_state(false);
	robot.turn_to_point(0.6,-24.5);
}

void red_elim_ring_side(){
	robot.move_to_point(-26.5,-9.3,true,3000,.78);
	mogo.set_clamp_state(true);
	pros::delay(300);
	intake.max();
	pros::delay(400);
	robot.move_to_point(-38.8,3.1);
	robot.move_to_point(-42,18.3);
	robot.move_ffwd(-24);
	robot.move_to_point(-27.6,13.5);
	robot.move_ffwd(-12);
	/*robot.move_to_point(0.6,24.5);
	misc_pneumatics.set_doinker_state(true);
	pros::delay(200);
	robot.intake->stop();
	robot.turn_to_point(-19.53,17);
	misc_pneumatics.set_doinker_state(false);
	robot.turn_to_point(0.6,24.5);*/
	// 2.1 24.5
	// 19.53 14
}

void red_solo_awp(){
	robot.move_ffwd(-27.2,2000,.85);
	mogo.clamp();
	robot.turn_to_point(-18.13, 21.56);
	intake.max();
	pros::delay(500);
	robot.move_ffwd(24);
	robot.move_to_point(-17.2,-53.7,3000,true);
	intake.stop();
	mogo.unclamp();
	robot.move_to_point(-44.1,-46.4,true,3000,0.85);
	mogo.clamp();
	robot.turn_to_point(-48.9,-69.5);
	intake.max();
	robot.move_to_point(-48.9,-69.5);
	pros::delay(250);
	robot.move_to_point(-44.5,-34.5,false,3000,1);
	lift.set_state(LiftState::Touch);
}

void blue_solo_awp(){
	robot.move_ffwd(-27.2,2000,.85);
	mogo.clamp();
	robot.turn_to_point(-18.13, -21.56);
	intake.max();
	pros::delay(500);
	robot.move_ffwd(24);
	robot.move_to_point(-17.2,53.7,3000,true);
	intake.stop();
	mogo.unclamp();
	robot.move_to_point(-44.1,46.4,true,3000,0.85);
	mogo.clamp();
	robot.turn_to_point(-48.9,69.5);
	intake.max();
	robot.move_to_point(-48.9,69.5);
	pros::delay(250);
	robot.move_to_point(-44.5,34.5,false,3000,1);
	lift.set_state(LiftState::Touch);
}

// 3 rings (1 on 5th mogo, 2 on safe mogo) = 7 points
void blue_elim_goal_side(){
	// -25.1 -10.2 true
	robot.move_to_point(-25.1, -10.2, true,3000,.85);
	mogo.clamp();
	pros::delay(300);
	intake.max();
	pros::delay(400);
	robot.move_to_point(-28, 11.5);
	// -10 -16.8
	robot.move_to_point(-10, -16.8);
	
	/*misc_pneumatics.set_doinker_state(true);
	pros::delay(300);
	robot.move_ffwd(-10);
	misc_pneumatics.set_doinker_state(false);
	robot.move_to_point(-12.7,-22);
	pros::delay(1250);*/

	// doinker
	// 1.2 20.5
	// move left 60
}
void blue_awp_goal_side(){
	robot.move_to_point(-25.1, -10.2, true,3000,.85);
	mogo.clamp();
	pros::delay(300);
	intake.max();
	pros::delay(400);
	robot.move_to_point(-27, 5.5);
	pros::delay(2000);
	intake.stop();
	lift.lift_move(127);
	robot.move_to_point(-34.5,-19.5);
	lift.lift_move(-20);
}

void evolution_auton(){
	robot.move_ffwd(-13.5);
	robot.move_to_point(-13.5, -4.5,true,1500,.85);
	intake.max();
	pros::delay(500);
	robot.move_ffwd(10);
	intake.stop();
	robot.move_to_point(8.5,35.7,true,3000,.9);
	mogo.set_clamp_state(true);
	pros::delay(200);
	intake.max();
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
	intake.max();
	pros::delay(400);
	robot.move_to_point(-28, -11.5);
	pros::delay(1000);
	intake.stop();
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
	intake.max();
	pros::delay(400);
	robot.move_to_point(-27, -5.5);
	// -32.5, 19.5
}

void risky_elim_ring_side(){
	robot.chassis.move_voltage(volts(-12));
	pros::delay(300);
	robot.chassis.move_voltage(volts(-4));
	pros::delay(350);
	mogo.clamp();
	robot.chassis.brake();
	robot.chassis.move_voltage(volts(0));
	robot.chassis.move_voltage(volts(1));
	pros::delay(50);
	robot.chassis.move_voltage(volts(0));
	robot.turn_to_point(-33.7,15.83);
	intake.max();
	robot.move_to_point(-33.7,15.83);
	robot.turn_to_point(-31.75, 28.33);
	robot.move_ffwd(16);
	robot.chassis.right_motors.move_voltage(volts(-6));
	robot.chassis.left_motors.move_voltage(volts(-9));
	pros::delay(350);
	robot.chassis.brake();
	robot.chassis.move_voltage(volts(0));
	robot.chassis.move_voltage(volts(1));
	pros::delay(50);
	robot.chassis.move_voltage(volts(0));
	robot.move_to_point(-21.3,24.5);
	robot.move_ffwd(-6);
	
	// 17.7 13.6
	// indexer
	// -18.8 20.9
	// -15.2 -16.2
}

void risky_elim_ring_blue(){
	robot.chassis.move_voltage(volts(-12));
	pros::delay(300);
	robot.chassis.move_voltage(volts(-4));
	pros::delay(350);
	mogo.clamp();
	robot.chassis.brake();
	robot.chassis.move_voltage(volts(0));
	robot.chassis.move_voltage(volts(1));
	pros::delay(50);
	robot.chassis.move_voltage(volts(0));
	robot.turn_to_point(-33.7,-15.83);
	intake.max();
	robot.move_to_point(-33.7,-15.83);
	robot.turn_to_point(-31.75, -28.33);
	robot.move_ffwd(16);
	robot.chassis.right_motors.move_voltage(volts(-9));
	robot.chassis.left_motors.move_voltage(volts(-6));
	pros::delay(350);
	robot.chassis.brake();
	robot.chassis.move_voltage(volts(0));
	robot.chassis.move_voltage(volts(1));
	pros::delay(50);
	robot.chassis.move_voltage(volts(0));
	robot.move_to_point(-21.3,-24.5);
	robot.move_ffwd(-6);
	// 19.6 -12.9
}

// Skills
// 19 rings (1 on alliance stake, 6 on mogo 1, 6 on mogo 2, 6 on mogo 3, all in corner + 1 more) = 47 pts
void skills(){
	lift.lift_move(-20);
	intake.max();
	pros::delay(750);
	robot.move_ffwd(16.5);
	intake.stop();
	// grab mogo
	// skills mogo clamp manual
	robot.turn_to_point(17, 22.1, true);
	robot.move_ffwd(-24,3000,.85);
	mogo.clamp();
	pros::delay(150);
	intake.max();
	// ring 1
	robot.move_to_point(37.9, 24.9);
	// ring 2
	robot.move_to_point(62.9, 52.2,false,3000,1);
	pros::delay(500);
	// ring 3
	robot.move_to_point(42.6,47.5);
	// rings 4,5
	robot.move_to_point(9.4,45.8,false,5000,0.7);
	pros::delay(1000);
	robot.move_ffwd(-18);
	// ring 6
	robot.move_to_point(20,53.9);
	robot.move_to_point(6.9,57.4,true,3000,0.7);
	intake.stop();
	mogo.unclamp();
	robot.move_ffwd(24);
	// move to next mogo
	robot.move_to_point(17.5,-9,true);
	robot.move_to_point(15,-24.6,true,3000,0.85);
	mogo.clamp();
	pros::delay(300);
	intake.max();
	robot.move_to_point(39.6,-24.2);
	robot.move_to_point(62,-53.2,false,3000,1);
	pros::delay(500);
	robot.move_to_point(42.1,-49.2);
	robot.move_to_point(9.5,-47.3,false,3000,0.7);
	robot.move_ffwd(-18);
	robot.move_to_point(16.8, -57.2);
	robot.move_to_point(4.5,-59.5,true,3000,0.7);
	pros::delay(750);
	intake.stop();
	mogo.unclamp();
	pros::delay(300);
	robot.move_ffwd(50);
	intake.max();
	// grab ring and hold in intake
	robot.move_to_point(86.17,-20.85);
	pros::delay(500);
	intake.stop();
	robot.move_to_point(108,1,true,3000,.85);
	mogo.clamp();
	pros::delay(300);
	intake.max();
	robot.move_to_point(82.96,26.22,false,3000,.85);
	robot.move_to_point(85,48.62,false,3000,.85);
	robot.move_to_point(101.85,48.42,false,3000,.85);
	robot.move_ffwd(-6);
	pros::delay(1000);
	robot.move_to_point(119.6,62.6,true);
	robot.move_ffwd(24);
	robot.move_to_point(128.5,-58,true);
	// back 6 in
	// 105.7 59.1
	// 119.6 62.6 true
	// fwd 36
	// 128.5 -58true


	
}

void test(){
	lift.set_state(LiftState::Up);
}

// Selector
// We use the Robodash library to control selection of our autons.
rd::Selector selector({
	{"Skills", skills},
	{"Red Elim 4 Ring", risky_elim_ring_side},
	{"Blue Elim 4 Ring", risky_elim_ring_blue}, 
	{"Red WP", red_solo_awp},
	{"Blue WP", blue_solo_awp},
	{"red goal wp", red_awp_goal_side}
});

rd::Console console;

//bool detected = false;

pros::Motor rot(3);

void initialize() {
	robot.initialize();
	robot.start_odom();
	selector.focus();

	robot.chassis.left_motors.raw.tare_position_all();
	robot.chassis.right_motors.raw.tare_position_all();

	pros::Task screen_task([&]() {
        while (true) {
			dlib::Pose2d pose = robot.odom.get_position();

			console.clear();
            // print robot location to the brain screen
            console.printf("X: %f", pose.x.in(inches)); // x
            console.printf("Y: %f", pose.y.in(inches)); // y
            console.printf("Theta: %f",pose.theta.in(degrees));

            // delay to save resources
            pros::delay(20);
        }
	});


	// Intake Filter Task //
	// ------------------ //
	// when the sensor detects a ring, enable one of two macros:
	// redirect: redirect a alliance color ring onto wall stake mech
	// fling: fling non-alliance rings so they dont get scored
	// shouldn't interfere with auton

	pros::Task lift_task([&]() {
		while(true){
			auto kP = 1.5;
			auto error = lift.target_position - rot.get_position();

			auto p = error * kP;

			lift.move_voltage(p);

			if(rot.get_position() > 10000 && lift.lift_state != LiftState::Touch){
				lift.piston.set_value(true);
			}
			else{
				lift.piston.set_value(false);
			}
			pros::delay(20);
		}
	});
}

void disabled() {}

void competition_initialize() {}

// Add selected autons in here.
// Prints the completion time of autons.
void autonomous() {
	console.focus();

	auto start_time = pros::millis();
	//lift.set_state(LiftState::Touch);
	selector.run_auton();
	//bad_team();
	//red_solo_awp();
	//blue_solo_awp();
	//robot.turn_with_pid(45);
	//red_elim_ring_side();
	//skills();
	auto elapsed_time = pros::millis() - start_time;

	std::cout << "settled in " << elapsed_time << " milliseconds." << "\n";
	dlib::Pose2d position = robot.odom.get_position();
	std::cout << position.x.in(inches) << ", " << position.y.in(inches) << ", " << position.theta.in(degrees) << "\n";
}	

void opcontrol() {
	pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
	intake.set_alliance(Alliance::Red);
	
	robot.chassis.left_motors.raw.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	robot.chassis.right_motors.raw.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	std::int32_t last_error = rot.get_position();
	
	while(true){
		// ------------------- //
		// Chassis Controls
		// ------------------- //
		// get power and turn
		double power = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X);

		robot.chassis.arcade(power,-turn);

		// ------------------- //
		// Intake Controls
		// ------------------- //

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake.max();
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			intake.rev();
		}
		else{
			intake.stop();
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
			lift.set_state(LiftState::Up);
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
			lift.set_state(LiftState::Idle);
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
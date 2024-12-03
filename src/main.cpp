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
#include <mutex>

using namespace au;

// Robot class
class Robot {
public:
	dlib::Chassis chassis;
	dlib::Imu imu;

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

		while(!left_move_settler.is_settled(right_move_pid.get_error(),left_move_pid.get_derivative())){
			auto error = dlib::linear_error(target_displacement,chassis.right_motors_displacement());
			auto voltage = right_move_pid.update(error, milli(seconds)(20));

			chassis.right_motors.move_voltage(voltage);

			pros::delay(20);
		}
	}

	// Move the chassis using PID and feedforward
	void move_ffwd(double displacement, double max_time = 99999, int max_voltage = 12000){
		auto start_displacement = chassis.forward_motor_displacement();
		dlib::TrapezoidProfile<Meters> profile {
			meters_per_second_squared(3),
			meters_per_second(1.6),
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

			std::cout << elapsed_time << "," << setpoint.velocity.in(meters_per_second) << "," << chassis.forward_motor_velocity().in(meters_per_second) << "\n";
			
			if(profile.stage(milli(seconds)(elapsed_time)) == dlib::TrapezoidProfileStage::Decelerating){
				chassis.move_voltage(volts(0));
			}

			if(profile.stage(milli(seconds)(elapsed_time)) == dlib::TrapezoidProfileStage::Done){
				break;
			}

			chassis.move_voltage(ff_voltage + pid_voltage);
			
			pros::delay(20);
		}
		std::cout << "settled" << "\n";
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

			std::cout << "target:" << heading << ", " << "current:" << imu.get_heading() << ", " << "error: " << error << ", voltage:" << voltage << std::endl;
			chassis.turn_voltage(voltage);

			last_error = error;

			elapsed_time += 20;
			pros::delay(20);
		}

		std::cout << "settled with voltage: " << voltage << "\n";
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
	void move_to_point(double x, double y, bool reverse = false, double max_time = 3000, int move_max_voltage = 12000, int turn_max_voltage = 12000) {
		auto point = dlib::Vector2d(inches(x), inches(y));
		turn_to_point(x, y, reverse,max_time, turn_max_voltage);
		auto displacement = odom.displacement_to(point);
		if(reverse){
			displacement = -displacement;
		}
		move_ffwd(displacement.in(inches), max_time, move_max_voltage);
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

// Left DB move PID gains
dlib::PidConfig left_move_pid_gains {
	{5,
	0,
	0},
	volts(12)
};

// Right DB move PID gains
dlib::PidConfig right_move_pid_gains {
	{5,
	0,
	0},
	volts(12)
};

// Chassis move PID settler
dlib::ErrorDerivativeSettler<Meters> move_pid_settler {
	inches(0.5),		// error threshold, the maximum error the pid can settle at
	meters_per_second(0.005) // derivative threshold, the maximum instantaneous error over time the pid can settle at
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
	37,
	0,
	2.3
	},
	volts(12)
};

// Chassis turn PID settler
dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler {
	degrees(1),		// error threshold, the maximum error the pid can settle at
	degrees_per_second(0.2)	// derivative threshold, the maximum instantaneous error over time the pid can settle at
};

// Intake object
Intake intake(
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
	feedforward
};

// ------------------------------ //
// Autonomous //
// ------------------------------ //

// 4 rings (1 on alliance stake, 3 on mogo) = 8 points
void blue_ring_side(){
	// alliance stake
	robot.move_ffwd(-14);
	robot.move_to_point(-14, 6,true,700);
	intake.max();
	pros::delay(500);
	// grab mogo
	robot.move_ffwd(13);
	robot.move_to_point(8.7,-30.7,true,10000,5750,8000);
	mogo.set_clamp_state(true);
	// intake ring 1
	robot.move_to_point(27.5,-33.9,false,10000,12000,8000);
	// back up
	robot.move_ffwd(-10);
	// intake ring 2
	robot.move_to_point(23.6,-46.3,false,10000,12000,8000);
	// intake ring 3
	robot.move_to_point(32,-49.6,false,10000,12000,8000);
	pros::delay(500);
	// back up
	robot.move_to_point(21,-38,true,10000,12000,8000);
	lift.lift_move(127);
	robot.move_to_point(0,-39.3,false,10000,12000,8000);
}

// 4 rings (1 on alliance stake, 3 on mogo) = 8 points
void red_ring_side(){
	// alliance stake
	robot.move_ffwd(-14);
	robot.move_to_point(-14, -6,true,700);
	intake.max();
	pros::delay(500);
	// grab mogo
	robot.move_ffwd(13);
	robot.move_to_point(8.7,30.7,true,10000,5750,8000);
	mogo.set_clamp_state(true);
	// intake ring 1
	robot.move_to_point(27.5,33.9,false,10000,12000,8000);
	// back up
	robot.move_ffwd(-10);
	// intake ring 2
	robot.move_to_point(23.6,45.3,false,10000,12000,8000);
	// intake ring 3
	robot.move_to_point(32,48.6,false,10000,12000,8000);
	pros::delay(500);

	robot.move_to_point(8.8,40.5,true,10000,12000,8000);
	pros::delay(1000);
	lift.lift_move(127);
	robot.move_to_point(3.1,41.4);

}

// 3 rings (1 on 5th mogo, 2 on safe mogo) = 7 points
void blue_goal_side(){
	// grab 5th mogo
	robot.move_ffwd(-30.4,1000);
	robot.move_to_point(-41.8,-7.4,true);
	mogo.set_clamp_state(true);
	// score preload
	intake.max();
	// grab + score ring 1
	robot.move_to_point(-30.7,-9.9,false,1000);
	pros::delay(750);
	intake.stop();
	mogo.set_clamp_state(false);
	// grab safe mogo
	robot.move_to_point(-24.2,-11.6,false,1000);
	robot.move_to_point(-27,-28.8,true,2000,5500);
	mogo.set_clamp_state(true);
	// indexer
	intake.max();
	pros::delay(2000);
	
	misc_pneumatics.set_indexer_state(true);
	robot.move_to_point(-7.2,-51,false,1250);
	misc_pneumatics.set_indexer_state(false);
	pros::delay(1000);
	robot.move_ffwd(-20);
	pros::delay(500);
	
	// touch ladder
	lift.lift_move(127);
	robot.move_to_point(-32,-43.2);
}

// 3 rings (1 on 5th mogo, 2 on safe mogo) = 7 points
void red_goal_side(){
	// grab 5th mogo
	robot.move_ffwd(-30.4,1000);
	robot.move_to_point(-40.8,7.4,true);
	mogo.set_clamp_state(true);
	// score preload
	intake.max();
	// grab + score ring 1
	robot.move_to_point(-30.7,9.9,false,1000);
	pros::delay(750);
	intake.stop();
	mogo.set_clamp_state(false);
	// grab safe mogo
	robot.move_to_point(-24.2,11.6,false,1000);
	robot.move_to_point(-25,27.3,true,2000,5500);
	mogo.set_clamp_state(true);
	// indexer
	intake.max();
	pros::delay(2000);
	misc_pneumatics.set_indexer_state(true);
	robot.move_to_point(-7.2,51,false,1250);
	misc_pneumatics.set_indexer_state(false);
	pros::delay(1000);
	robot.move_ffwd(-20);
	pros::delay(500);
	lift.lift_move(127);
	robot.move_to_point(-26,41.2);
}

// Skills
// 19 rings (1 on alliance stake, 6 on mogo 1, 6 on mogo 2, 6 on mogo 3, all in corner + 1 more) = 47 pts
void skills(){
	robot.intake.max();
	pros::delay(750);
	robot.move_ffwd(13.5);
	// grab mogo
	robot.move_to_point(14.1, 20.1, true, 10000, 5500);
	mogo.set_clamp_state(true);
	// ring 1
	robot.move_to_point(37.9, 22.7);
	// ring 2
	robot.move_to_point(57,51.6);
	// ring 3
	robot.move_to_point(42.2, 47.8);
	// rings 4,5
	robot.move_to_point(5.2, 42.9,false,10000,4000);
	pros::delay(1500);
	// back up
	robot.move_ffwd(-20);
	// ring 6
	robot.move_to_point(18.4, 51.7);
	// drop mogo
	robot.move_to_point(6.8, 55.9,true,10000,4000);
	mogo.set_clamp_state(false);
	intake.stop();
	// move to next mogo
	robot.move_to_point(19,50.9,false,1000);
	robot.move_to_point(14.5,22.9,true);
	robot.move_to_point(15.6,-25.4,true,10000,5500);
	mogo.set_clamp_state(true);
	intake.max();
	//ring 1
	robot.move_to_point(38.5,-23.9);
	// ring 2
	robot.move_to_point(58.7,-52);
	// ring 3
	robot.move_to_point(37.4,-48.6);
	// rings 4,5
	robot.move_to_point(5.1, -47.4,false,10000,4000);
	pros::delay(1500);
	// back up
	robot.move_ffwd(-23);
	// ring 6
	robot.move_to_point(18.1, -56.7);
	// drop mogo
	robot.move_to_point(7.2, -60.2, true, 10000, 4000);
	mogo.set_clamp_state(false);
	intake.stop();
	// third quad
	robot.move_to_point(60.5,-43.4);
	intake.max();
	// ring 1 kinda
	robot.move_to_point(77.4, -30.3);
	pros::delay(750);
	intake.stop();
	// grab mogo.
	robot.move_to_point(106.7, -5.6, true, 10000, 5500);
	mogo.set_clamp_state(true);
	intake.max();
	// grab more ring
	robot.move_to_point(85.2, -48.2);
	robot.move_to_point(100.9, -51);
	robot.move_to_point(100.9, -56.8);
	// drop the mogo off
	robot.move_to_point(116.7, -61.5, true, 2000, 4000);
	mogo.set_clamp_state(false);
	robot.move_to_point(91.2, -52.2);
}

// 6 rings (1 on alliance stake, 5 on mogo) = 11 pts
void awp(){
	// set alliance
	intake.set_alliance(Alliance::Red);

	robot.move_ffwd(-2);
	robot.move_left_with_pid(inches(-12));
	pros::delay(650);
	robot.chassis.brake();
	robot.move_ffwd(-4);
	intake.max();
	pros::delay(1700);
	intake.stop();
}

void left_pid_test(){
	robot.move_left_with_pid(inches(-24));
}

// This serves to test our tasks in autonomous
void test_tasks(){
	intake.set_alliance(Alliance::Blue);
	intake.auton_stick = true;
	while(true){
		if(!intake.intake_filter()){
			intake.move(127);
		}
		pros::delay(20);
	}
}

// Selector
// We use the Robodash library to control selection of our autons.
rd::Selector selector({
    {"Red Ring Side", red_ring_side},
	{"Red Goal Side", red_goal_side},
	{"Blue Ring Side", blue_ring_side},
	{"Blue Goal Side", blue_goal_side},
	{"Skills", skills},

});

rd::Console console;

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
	/*pros::Task intake_filter_task([&]() {
		while(true) {
			if(intake.intake_filter() && intake.redirect){
				// redirect macro aka just reverse
				intake.move(-127);
				pros::delay(500);
				intake.stop();
			}
			else if(intake.intake_filter() && intake.auton_stick){
				intake.stop();
			}
			else if(intake.intake_filter()){
				// fling macro
				intake.move(127);
				pros::delay(200);
				intake.stop();
			}
		}
	});*/

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
	robot.move_left_with_pid(inches(-12));
	auto elapsed_time = pros::millis() - start_time;

	std::cout << "settled in " << elapsed_time << " milliseconds." << "\n";
	dlib::Pose2d position = robot.odom.get_position();
	std::cout << position.x.in(inches) << ", " << position.y.in(inches) << ", " << position.theta.in(degrees) << "\n";
}	

void opcontrol() {
	lift.lift_move(-20);
	// Try arcade drive control!
	pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
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
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !intake.intake_filter()){
			intake.move(127);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && !intake.intake_filter()){
			intake.rev();
		}
		else if(!intake.intake_filter()){
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
			lift.lift_move(90);
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

		// ------------------- //
		// Intake Redirect
		// ------------------- //
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			intake.redirect = !intake.redirect;
			intake.set_redirect(intake.redirect);
		}

		pros::delay(20);
	}
	
	
}
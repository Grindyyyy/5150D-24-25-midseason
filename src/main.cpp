#include "main.h"
#include "au/au.hpp"
#include "dlib/controllers/pid.hpp"
#include "dlib/controllers/error_time_settler.hpp"
#include "dlib/controllers/error_derivative_settler.hpp"
#include "dlib/dlib.hpp"
#include "dlib/kinematics/odometry.hpp"
#include "intake.hpp"
#include "misc_pneumatics.hpp"
#include "mogo.hpp"
#include "lift.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"

using namespace au;

// Robot class
class Robot {
	public:
		dlib::Chassis chassis;
		dlib::Imu imu;

		dlib::Odometry odom;
		std::unique_ptr<pros::Task> odom_updater;

		dlib::Pid<Inches> move_pid;
		dlib::ErrorDerivativeSettler<Meters> move_settler;
		dlib::ErrorTimeSettler<Meters> time_move_settler;

		dlib::Pid<Degrees> turn_pid;
		dlib::ErrorDerivativeSettler<Degrees> turn_settler;
		dlib::ErrorTimeSettler<Degrees> time_turn_settler;

Robot(
	dlib::ChassisConfig chassis_config, 
	dlib::ImuConfig imu_config,
	dlib::PidGains move_pid_gains,
	dlib::ErrorTimeSettler<Meters> time_move_settler,
	dlib::ErrorDerivativeSettler<Meters> move_settler,
	dlib::PidGains turn_pid_gains,
	dlib::ErrorTimeSettler<Degrees> time_turn_settler,
	dlib::ErrorDerivativeSettler<Degrees> turn_settler
	) : 
	chassis(chassis_config), 
	imu(imu_config),
	move_pid(move_pid_gains),
	time_move_settler(time_move_settler),
	move_settler(move_settler),
	turn_pid(turn_pid_gains),
	time_turn_settler(time_turn_settler),
	turn_settler(turn_settler),
	odom() {

	}

	void initialize() {
		chassis.initialize();

		chassis.left_motors.raw.set_gearing_all(pros::MotorGearset::blue);
		chassis.right_motors.raw.set_gearing_all(pros::MotorGearset::blue);
		
		imu.initialize();
		//start_odom();
	}

	// Use PID to do a relative movement
	void move_with_pid_derivative(Quantity<Inches, double> displacement) {
		auto start_displacement = chassis.average_motor_displacement();
		auto target_displacement = start_displacement + displacement;

		auto voltage = move_pid.update(start_displacement, milli(seconds)(10));
		chassis.move_voltage(voltage);

		move_pid.target(target_displacement);

		while (!move_settler.is_settled(move_pid.get_error(), move_pid.get_derivative())) {
			auto reading = chassis.average_motor_displacement();
			auto error = displacement - reading;
			auto voltage = move_pid.update(reading, milli(seconds)(10));

			std::cout << reading << std::endl;
			chassis.move_voltage(voltage);

			pros::delay(5);
		}

		chassis.move(0);

	}

	// Use PID to do a relative movement
	void move_with_pid_time(Quantity<Inches, double> displacement, int max_voltage, double max_time) {
		auto start_displacement = chassis.average_motor_displacement();
		auto target_displacement = start_displacement + displacement;
		auto start_time = pros::millis();	
		double elapsed_time = 0;

		move_pid.target(target_displacement);

		while (!time_move_settler.is_settled(move_pid.get_error(), milli(seconds)(5)) && (elapsed_time < max_time)) {
			auto reading = chassis.average_motor_displacement();
			auto error = displacement - reading;
			auto voltage = move_pid.update(reading, milli(seconds)(5));
			if(au::abs(voltage) > milli(volts)(max_voltage)){
				if(voltage > milli(volts)(0)){
					voltage = milli(volts)(max_voltage);
				}
				else{
					voltage = -milli(volts)(max_voltage);
				}
				
			}
			chassis.move_voltage(voltage);

			elapsed_time += 5;

			pros::delay(5);
		}
		chassis.move(0);
		pros::delay(10);
		
	}


	void turn_with_pid(Quantity<Degrees, double> heading, int max_voltage, double max_time) {
		auto target_heading = heading;
		double cur_time = pros::millis();
		double elapsed_time = 0;

		turn_pid.target(target_heading);
		auto reading = imu.get_rotation();

		while (!time_turn_settler.is_settled(turn_pid.get_error(), milli(seconds)(5)) && (elapsed_time < max_time)) {
			auto reading = imu.get_rotation();
			
			auto voltage = turn_pid.update(reading, milli(seconds)(5));
			if(voltage > milli(volts)(max_voltage)){
				voltage = milli(volts)(max_voltage);
			}
			chassis.turn_voltage(voltage);

			elapsed_time += 10;
			pros::delay(10);
		}
		chassis.move(0);
		//std::cout << "lucas parham should die" << std::endl;
		pros::delay(10);

	}

	void turn_to_point(dlib::Vector2d point, bool reverse, int max_voltage = 12000, double max_time = 1000) {
		auto angle = odom.angle_to(point, reverse);

		turn_with_pid(angle,max_voltage,max_time);
	}

	void move_to_point(dlib::Vector2d point, bool reverse, int move_max_voltage = 12000, int turn_max_voltage = 12000, double max_time = 1000) {
		turn_to_point(point, reverse,turn_max_voltage, max_time);

		auto displacement = odom.displacement_to(point);
		if(reverse){
			displacement = -displacement;
		}
		move_with_pid_time(displacement,move_max_voltage, max_time);
	}

// Odom task
	void start_odom() {
		odom_updater = std::make_unique<pros::Task>([this]() {
			while (true) {
				odom.update(
					chassis.left_motors_displacement(), 
					chassis.right_motors_displacement(), 
					imu.get_rotation()
				);

				pros::delay(20);
			}
		});
	}

};

// CONFIGS
// adjust these to make robot do different stuff

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

// Adjust these gains
dlib::PidGains move_pid_gains {
	1.8, 	// kp, porportional gain
	0, 	// ki, integral gain
		0.1// kd, derivative gain
};

dlib::ErrorDerivativeSettler<Meters> move_pid_settler {
	inches(0.5),		// error threshold, the maximum error the pid can settle at
	meters_per_second(0.005) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

dlib::ErrorTimeSettler<Meters> time_move_pid_settler {
	inches(0.2),		// error threshold, the maximum error the pid can settle at
	milli(seconds)(100) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

// Adjust these gains
dlib::PidGains turn_pid_gains {
	1.15, 	// kp, porportional gain
	0, 	// ki, integral gain
	.039	// kd, derivative gain
};

dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler {
	degrees(1),		// error threshold, the maximum error the pid can settle at
	degrees_per_second(0.1)	// derivative threshold, the maximum instantaneous error over time the pid can settle at
};

dlib::ErrorTimeSettler<Degrees> time_turn_pid_settler {
	degrees(1),		// error threshold, the maximum error the pid can settle at
	milli(seconds)(50) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

// init robot + subsystems
//Robot
// Chassis + autonomous motion control will be inside of this class
Robot robot = Robot(
	chassis_config,
	imu_config,
	move_pid_gains,
	time_move_pid_settler,
	move_pid_settler,
	turn_pid_gains,
	time_turn_pid_settler,
	turn_pid_settler
);

// Intake
// the intake and anything related to it will be included inside of this class.
Intake intake(
	-8,
	3,
	4
);

// Mogo
// the mogo pneumatic will be included inside of this class.
Mogo mogo(
	'H',
	false
);

// Lift
// anything relating to a lift will be included inside of this class.
Lift lift(
	-1,
	10
);

// MiscPneumatics
// any miscellaneous pneumatics will be put here
MiscPneumatics misc_pneumatics(
	'G',
	false,
	'E',
	false,
	'C',
	false
);

void test(){
	robot.turn_to_point({inches(-10), inches(0)}, false, 12000, 2000);
}

void red_awp(){
	double cur_time = pros::millis();
	// Try a PID movement!
	// Move to Alliance Stake
	robot.move_to_point({inches(-15), inches(0)}, true, 12000, 12000, 750);
	robot.move_to_point({inches(-15), inches(-6)}, true, 12000, 12000, 600);
	
	// Intake onto alliance stake
	intake.max_intake();
	pros::delay(450);

	// Move to safe Mogo
	robot.move_to_point({inches(-15.5), inches(5)}, false, 12000, 12000,1000);
	robot.move_to_point({inches(10), inches(35)}, true, 5500, 12000,1500);
	
	// Clamp mogo
	mogo.set_clamp_state(true);

	// Intake safe ring
	robot.move_to_point({inches(32), inches(34)}, false, 12000, 8000,750);
	
	// move + intake middle rings
	robot.move_to_point({inches(18), inches(34)}, true, 12000, 12000,500);
	pros::delay(300);
	robot.move_to_point({inches(24), inches(45)}, false, 12000, 12000,800);
	robot.move_to_point({inches(32), inches(47)}, false, 12000, 6000,800);

	// touch bar
	robot.move_to_point({inches(23), inches(41)}, true, 12000, 6000,700);
	pros::delay(750);
	lift.lift_move(127);
	//misc_pneumatics.set_arm_state(true);
	robot.move_to_point({inches(2), inches(43)}, false, 12000, 12000,1000);
	// 18 -41 true
	// wait a sec
	// raise lift
	double elapsed_time = pros::millis() - cur_time;

	std::cout << "autonomous finished in " << elapsed_time << std::endl;
}

void blue_awp(){
	double cur_time = pros::millis();
	// Try a PID movement!
	// Move to Alliance Stake
	robot.move_to_point({inches(-15), inches(0)}, true, 12000, 12000, 750);
	robot.move_to_point({inches(-15), inches(6)}, true, 12000, 12000, 600);
	
	// Intake onto alliance stake
	intake.max_intake();
	pros::delay(450);

	// Move to safe Mogo
	robot.move_to_point({inches(-15.5), inches(-5)}, false, 12000, 12000,1000);
	robot.move_to_point({inches(10), inches(-35)}, true, 5500, 12000,1500);
	
	// Clamp mogo
	mogo.set_clamp_state(true);

	// Intake safe ring
	robot.move_to_point({inches(32), inches(-34)}, false, 12000, 8000,750);
	
	// move + intake middle rings
	robot.move_to_point({inches(18), inches(-34)}, true, 12000, 12000,500);
	pros::delay(300);
	robot.move_to_point({inches(24), inches(-45)}, false, 12000, 12000,800);
	robot.move_to_point({inches(32), inches(-47)}, false, 12000, 6000,800);

	// touch bar
	robot.move_to_point({inches(23), inches(-41)}, true, 12000, 6000,700);
	pros::delay(750);
	lift.lift_move(127);
	//misc_pneumatics.set_arm_state(true);
	robot.move_to_point({inches(2), inches(-43)}, false, 12000, 12000,1000);
	// 18 -41 true
	// wait a sec
	// raise lift
	double elapsed_time = pros::millis() - cur_time;

	std::cout << "autonomous finished in " << elapsed_time << std::endl;
}

void skills(){
	double cur_time = pros::millis();
	// intake onto ally stake
	intake.max_intake();
	pros::delay(600);

	// move to and clamp mogo
	robot.move_to_point({inches(13.5), inches(0)}, false, 12000, 12000, 750);
	robot.move_to_point({inches(14), inches(23)}, true, 5500, 8000, 3000);
	mogo.set_clamp_state(true);

	// intake ring 1
	robot.move_to_point({inches(40.6), inches(23)}, false, 9000, 12000, 1500);

	// intake ring 2
	robot.move_to_point({inches(58), inches(48)}, false, 9000, 12000, 2000);
	pros::delay(1000);

	// intake ring 3
	robot.move_to_point({inches(47), inches(46)}, false, 9000, 12000, 1000);
	
	// intake rings 4 and 5
	robot.move_to_point({inches(8), inches(47)}, false, 4000, 12000, 3000);
	pros::delay(2000);

	// intake ring 6
	robot.move_to_point({inches(15.5), inches(37.5)}, true, 12000, 12000, 750);
	robot.move_to_point({inches(14.7), inches(52.3)}, false, 7000, 12000, 2000);

	// drop mogo
	robot.move_to_point({inches(5), inches(60)}, true, 8000, 8000, 2000);
	mogo.set_clamp_state(false);
	double elapsed_time = pros::millis() - cur_time;
	std::cout << "autonomous skills finished in " << elapsed_time << " milliseconds." << std::endl;
}

void on_center_button() {}

void initialize() {
	pros::lcd::initialize();
	robot.initialize();
	
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	double cur_time = pros::millis();
	robot.chassis.left_motors.raw.tare_position_all();
	robot.chassis.right_motors.raw.tare_position_all();
	robot.start_odom();

	pros::Task screen_task([&]() {
        while (true) {
			dlib::Pose2d pose = robot.odom.get_position();
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", pose.x.in(inches)); // x
            pros::lcd::print(1, "Y: %f", pose.y.in(inches)); // y
            pros::lcd::print(2, "Theta: %f",pose.theta.in(degrees)); // heading
			std::cout << pose.theta.in(degrees) << std::endl;
            // delay to save resources
            pros::delay(20);
        }
	});

	blue_awp();
	//skills();
	//test();

}	



void opcontrol() {
	bool mogo_state = true;
	bool arm_state = false;
	int range = 16000;
	robot.start_odom();
	while(true){
		// Try arcade drive control!
		pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);

		// get power and turn
		double power = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X);

		robot.chassis.arcade(power,turn);

		dlib::Pose2d curPos = robot.odom.get_position();
		pros::lcd::print(0, "x: %f", (curPos.x).in(au::inches)); // print the x position
        pros::lcd::print(1, "y: %f", (curPos.y).in(au::inches)); // print the y position
    	pros::lcd::print(2, "heading: %f", (curPos.theta).in(au::degrees)); // print the heading

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake.intake(127);
		}
		else{
			intake.stop_intake();
		}
	
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
			mogo_state = !mogo_state;
			mogo.set_clamp_state(mogo_state);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
			lift.lift_toggle = true;
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
			lift.lift_toggle = false;
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			misc_pneumatics.doinker_toggle = !misc_pneumatics.doinker_toggle;
			misc_pneumatics.set_doinker_state(misc_pneumatics.doinker_toggle);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			misc_pneumatics.set_indexer_state(true);
		}
		else{
			misc_pneumatics.set_indexer_state(false);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			arm_state = !arm_state;
			misc_pneumatics.set_arm_state(arm_state);
		}


		lift.lift_range(16000);

		pros::delay(8);
	}
	
	
}
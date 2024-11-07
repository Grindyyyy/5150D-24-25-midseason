#include "main.h"
#include "au/au.hpp"
#include "dlib/controllers/pid.hpp"
#include "dlib/controllers/error_time_settler.hpp"
#include "dlib/controllers/error_derivative_settler.hpp"
#include "dlib/dlib.hpp"
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

			std::cout << "welcome to the loop" << std::endl;
			std::cout << reading << std::endl;
			chassis.move_voltage(voltage);

			pros::delay(10);
		}

		chassis.move(0);

		std::cout << "settled" << std::endl;
		
	}

	// Use PID to do a relative movement
	void move_with_pid_time(Quantity<Inches, double> displacement) {
		auto start_displacement = chassis.average_motor_displacement();
		auto target_displacement = start_displacement + displacement;
		auto start_time = pros::millis();

		auto voltage = move_pid.update(start_displacement, milli(seconds)(10));
		chassis.move_voltage(voltage);

		move_pid.target(target_displacement);

		while (!time_move_settler.is_settled(move_pid.get_error(), milli(seconds)(10))) {
			auto reading = chassis.average_motor_displacement();
			auto error = displacement - reading;
			auto voltage = move_pid.update(reading, milli(seconds)(10));

			chassis.move_voltage(voltage);

			pros::delay(10);
		}

		chassis.move(0);

		std::cout << "settled in " << pros::millis() - start_time << " milliseconds." << std::endl;
		
	}


	void turn_with_pid(Quantity<Degrees, double> heading) {
		auto target_heading = heading;

		turn_pid.target(target_heading);
		auto reading = imu.get_rotation();
		auto voltage = turn_pid.update(reading, milli(seconds)(10));
		chassis.turn_voltage(voltage);
		while (!turn_settler.is_settled(turn_pid.get_error(), turn_pid.get_derivative())) {
			auto reading = imu.get_rotation();
			auto voltage = turn_pid.update(reading, milli(seconds)(10));

			chassis.turn_voltage(voltage);

			pros::delay(10);
		}
	}

	void turn_to_point(dlib::Vector2d point) {
		auto angle = odom.angle_to(point);
		dlib::Pose2d cur_pose = odom.get_position();
		std::cout << "angle: " << angle << " theta: " << cur_pose.theta << " x: " << cur_pose.x.in(inches) << " y: " << cur_pose.y.in(inches) << " left: " << chassis.left_motors_displacement().in(inches) << " right: " << chassis.right_motors_displacement().in(inches) << std::endl;
		turn_with_pid(angle);
	}

	void move_to_point(dlib::Vector2d point) {
		turn_to_point(point);

		auto displacement = odom.displacement_to(point);
		move_with_pid_time(displacement);
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
	1.2, 	// kp, porportional gain
	0, 	// ki, integral gain
		0// kd, derivative gain
};

dlib::ErrorDerivativeSettler<Meters> move_pid_settler {
	inches(0.5),		// error threshold, the maximum error the pid can settle at
	meters_per_second(0.005) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

dlib::ErrorTimeSettler<Meters> time_move_pid_settler {
	inches(0.5),		// error threshold, the maximum error the pid can settle at
	milli(seconds)(200) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

// Adjust these gains
dlib::PidGains turn_pid_gains {
	1.35, 	// kp, porportional gain
	0, 	// ki, integral gain
	.073	// kd, derivative gain
};

dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler {
	degrees(0.5),		// error threshold, the maximum error the pid can settle at
	degrees_per_second(0.1)	// derivative threshold, the maximum instantaneous error over time the pid can settle at
};

dlib::ErrorTimeSettler<Degrees> time_turn_pid_settler {
	degrees(1),		// error threshold, the maximum error the pid can settle at
	milli(seconds)(200) // derivative threshold, the maximum instantaneous error over time the pid can settle at
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
	true
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
	true,
	'E',
	false
);

void on_center_button() {}

void initialize() {
	pros::lcd::initialize();
	robot.initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	robot.chassis.left_motors.raw.tare_position_all();
	robot.chassis.right_motors.raw.tare_position_all();
	robot.start_odom();

	// Try a PID movement!
	robot.move_to_point({inches(0), inches(24)});

}

void opcontrol() {
	bool mogo_state = true;
	int range = 16000;
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

		lift.lift_range(16000);

		pros::delay(8);
	}
	
	
}
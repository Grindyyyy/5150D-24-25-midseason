#include "main.h"
#include "dlib/dlib.hpp"
#include "dlib/kinematics/odometry.hpp"
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

	dlib::Pid<Degrees> turn_pid;
	dlib::ErrorDerivativeSettler<Degrees> turn_settler;

	dlib::Odometry odom = dlib::Odometry();
	std::unique_ptr<pros::Task> odom_updater = nullptr;
	

	void initialize() {
		chassis.initialize();

		chassis.left_motors.raw.set_gearing_all(pros::MotorGearset::blue);
		chassis.right_motors.raw.set_gearing_all(pros::MotorGearset::blue);
		
		imu.initialize();
		//start_odom();
	}

	// Use PID to do a relative movement
	void move_with_pid_time(double displacement, double max_time = 99999, int max_voltage = 12000) {
		auto start_displacement = chassis.average_motor_displacement();
		auto target_displacement = dlib::relative_target(start_displacement, inches(displacement));
		auto start_time = pros::millis();	
		double elapsed_time = 0;

		move_pid.reset();
		move_settler.reset();

		while (!move_settler.is_settled(move_pid.get_error(), move_pid.get_derivative()) /*&& (elapsed_time < max_time)*/) {
			auto error = dlib::linear_error(target_displacement, chassis.average_motor_displacement());
			if (elapsed_time > max_time) {
				auto brake_volatge = au::copysign(volts(0.9), -error);
				chassis.move_voltage(brake_volatge);
				std::cout << "timed out" << "\n";
				break;
			}
			auto voltage = move_pid.update(error, milli(seconds)(20));
			if(au::abs(voltage) > milli(volts)(max_voltage)){
				if(voltage > milli(volts)(0)){
					voltage = milli(volts)(max_voltage);
				}
				if(voltage < milli(volts)(0)){
					voltage = milli(volts)(-max_voltage);
				}
			}
			
			std::cout << "error: " << error << ", voltage:" << voltage << std::endl;
			chassis.move_voltage(voltage);

			elapsed_time += 5;
			pros::delay(5);
		}
		std::cout << "settled" << "\n";
		chassis.move(0);
	}	

	// Use PID to do a relative movement
	void turn_with_pid_time(double heading, double max_time, int max_voltage) {
		auto target_heading = degrees(heading);

		auto start_time = pros::millis();	
		double elapsed_time = 0;	

		turn_pid.reset();
		turn_settler.reset();

		auto voltage = volts(0.0);

		while (!turn_settler.is_settled(turn_pid.get_error(), turn_pid.get_derivative())) {

			auto error = dlib::angular_error(degrees(heading), imu.get_rotation());

			if (elapsed_time > max_time) {
				auto brake_volatge = au::copysign(volts(0.9), -error);
				chassis.turn_voltage(brake_volatge);
				std::cout << "timed out" << "\n";
				break;
			}

			
			voltage = turn_pid.update(error, milli(seconds)(20));
			
			if(au::abs(voltage) > milli(volts)(max_voltage)){
				if(voltage > milli(volts)(0)){
					voltage = milli(volts)(max_voltage);
				}
				if(voltage < milli(volts)(0)){
					voltage = milli(volts)(-max_voltage);
				}
			}

			std::cout << "target:" << heading << ", " << "current:" << imu.get_heading() << ", " << "error: " << error << ", voltage:" << voltage << std::endl;
			chassis.turn_voltage(voltage);

			elapsed_time += 20;
			pros::delay(20);
		}

		std::cout << "settled with voltage: " << voltage << "\n";
		chassis.left_motors.raw.brake();
		chassis.right_motors.raw.brake();
	}

	void turn_to_point(double x, double y, bool reverse = false, double max_time = 15000, int max_voltage = 12000) {
		auto point = dlib::Vector2d(inches(x), inches(y));
		auto angle = odom.angle_to(point, reverse);

		turn_with_pid_time(angle.in(degrees), max_time,max_voltage);
	}

	void move_to_point(double x, double y, bool reverse = false, double max_time = 15000, int move_max_voltage = 12000, int turn_max_voltage = 12000) {
		auto point = dlib::Vector2d(inches(x), inches(y));
		turn_to_point(x, y, reverse,turn_max_voltage, max_time);
		auto displacement = odom.displacement_to(point);
		if(reverse){
			displacement = -displacement;
		}
		move_with_pid_time(displacement.in(inches), max_time, move_max_voltage);
	}

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
	1.2 * 39.37, 	// kp, porportional gain
	0 *  39.37, 	// ki, integral gain
		0.3 *  39.37// kd, derivative gain
};

dlib::ErrorDerivativeSettler<Meters> move_pid_settler {
	inches(0.5),		// error threshold, the maximum error the pid can settle at
	meters_per_second(0.005) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

dlib::ErrorTimeSettler<Meters> time_move_pid_settler {
	inches(0.5),		// error threshold, the maximum error the pid can settle at
	milli(seconds)(100) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

// Adjust these gainsWWW
dlib::PidGains turn_pid_gains {
	45,
	0,
	3
};

dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler {
	degrees(1),		// error threshold, the maximum error the pid can settle at
	degrees_per_second(0.2)	// derivative threshold, the maximum instantaneous error over time the pid can settle at
};

dlib::ErrorTimeSettler<Degrees> time_turn_pid_settler {
	degrees(1),		// error threshold, the maximum error the pid can settle at
	milli(seconds)(100) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

Intake intake(
	-8,
	3,
	4
);

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

// init robot + subsystems
//Robot
// Chassis + autonomous motion control will be inside of this class
Robot robot = {
	chassis_config,
	imu_config,
	intake,
	mogo,
	lift,
	misc_pneumatics,
	move_pid_gains,
	move_pid_settler,
	turn_pid_gains,
	turn_pid_settler
};


void example() {
	std::array<std::function<void()>, 20> updaters;

	int delay = 20 / updaters.size();
	
	for(auto updater : updaters) {
		updater();
		pros::delay(delay);
	}
}


void red_ring_side(){

}

void blue_ring_side(){
	
}

void red_goal_side(){
	
}

void blue_goal_side(){
	
}


void skills(){
	// alliance stake
	robot.intake.max();
	pros::delay(600);
	robot.move_with_pid_time(14.5);
	// clamp mogo
	robot.move_to_point(14.5,15,true,12000,5500);
	robot.mogo.set_clamp_state(true);
	// intake ring1
	robot.move_to_point(29.7,18.6);
	pros::delay(500);
	// intake ring2
	robot.move_to_point(55.7,47.5);
	pros::delay(500);
	// intake ring3
	robot.move_to_point(47.6,47.2);
	// intake ring4,5
	robot.move_to_point(3.1,47.2,false,3000,5000,12000);
	pros::delay(500);
	// back up
	robot.move_with_pid_time(-25);
	// intake ring 6
	robot.move_to_point(17.3,56.6);
	pros::delay(500);

	// back into corner + drop mogo
	robot.move_to_point(3.9,60.7,true,1500,5000);
	robot.mogo.set_clamp_state(false);
}

rd::Selector selector({
    {"Red Ring Side", red_ring_side},
	{"Red Goal Side", red_goal_side},
	{"Blue Ring Side", blue_ring_side},
	{"Blue Goal Side", blue_goal_side},
	{"Skills", skills}
});

rd::Console console;

void initialize() {
	robot.initialize();
	robot.start_odom();
	selector.focus();

	robot.chassis.left_motors.raw.tare_position_all();
	robot.chassis.right_motors.raw.tare_position_all();
	auto current = pros::millis();

	// Intake Filter Task //
	// ------------------ //
	// when the sensor detects a ring, enable one of two macros:
	// redirect: redirect a alliance color ring onto wall stake mech
	// fling: fling non-alliance rings so they dont get scored
	pros::Task intake_task([&]() {
		while(true){
			if(robot.intake.intake_filter()){
				if(robot.intake.redirect){
					robot.intake.max();
					pros::delay(165);
					robot.intake.rev();
				}
				else{
					robot.intake.max();
					pros::delay(200);
					robot.intake.rev();
				}
			}
		pros::delay(20);
		}
	});

	pros::Task screen_task([&]() {
        while (true) {
			dlib::Pose2d pose = robot.odom.get_position();
			
			auto elapsed = pros::millis() - current;

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
	
}

void disabled() {}

void competition_initialize() {}



void autonomous() {
	double cur_time = pros::millis();
	robot.chassis.left_motors.raw.tare_position_all();
	robot.chassis.right_motors.raw.tare_position_all();
	auto last_position = robot.chassis.average_motor_displacement();
	auto voltage = 0;

	console.focus();
	skills();
}	

void opcontrol() {
	bool mogo_state = true;
	bool arm_state = false;
	bool redirect_bool = false;
	int range = 16000;
	while(true){
		// Try arcade drive control!
		pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);

		// get power and turn
		double power = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X);

		dlib::Pose2d pose = robot.odom.get_position();

		console.clear();
            // print robot location to the brain screen
        console.printf("X: %f", pose.x.in(inches)); // x
        console.printf("\n Y: %f", pose.y.in(inches)); // y
        console.printf("\n Theta: %f",pose.theta.in(degrees));

		robot.chassis.arcade(power,turn);

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !robot.intake.intake_filter()){
			robot.intake.move(127);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && !robot.intake.intake_filter()){
			robot.intake.rev();
		}
		else if(!robot.intake.intake_filter()){
			robot.intake.stop();
		}
	
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
			mogo_state = !mogo_state;
			robot.mogo.set_clamp_state(mogo_state);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
			robot.lift.lift_toggle = true;
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
			robot.lift.lift_toggle = false;
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			robot.pneumatics.doinker_toggle = !misc_pneumatics.doinker_toggle;
			robot.pneumatics.set_doinker_state(misc_pneumatics.doinker_toggle);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			robot.pneumatics.set_indexer_state(true);
		}
		else{
			robot.pneumatics.set_indexer_state(false);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			arm_state = !arm_state;
			robot.pneumatics.set_arm_state(arm_state);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			redirect_bool = !redirect_bool;
			robot.intake.set_redirect(redirect_bool);
		}

		robot.lift.lift_range(16000);

		pros::delay(10);
	}
	
	
}
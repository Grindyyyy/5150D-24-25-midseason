#include "main.h"
#include "au/au.hpp"
#include "dlib/controllers/feedforward.hpp"
#include "dlib/controllers/pid.hpp"
#include "dlib/controllers/error_time_settler.hpp"
#include "dlib/controllers/error_derivative_settler.hpp"
#include "dlib/dlib.hpp"
#include "dlib/kinematics/odometry.hpp"
#include "dlib/trajectories/trapezoid_profile.hpp"
#include "dlib/utilities/error_calculation.hpp"
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

		dlib::Feedforward<Meters> move_ffwd;

Robot(
	dlib::ChassisConfig chassis_config, 
	dlib::ImuConfig imu_config,
	dlib::PidGains move_pid_gains,
	dlib::ErrorTimeSettler<Meters> time_move_settler,
	dlib::ErrorDerivativeSettler<Meters> move_settler,
	dlib::PidGains turn_pid_gains,
	dlib::ErrorTimeSettler<Degrees> time_turn_settler,
	dlib::ErrorDerivativeSettler<Degrees> turn_settler,
	dlib::Feedforward<Meters> move_ffwd
	) : 
	chassis(chassis_config), 
	imu(imu_config),
	move_pid(move_pid_gains),
	time_move_settler(time_move_settler),
	move_settler(move_settler),
	turn_pid(turn_pid_gains),
	time_turn_settler(time_turn_settler),
	turn_settler(turn_settler),
	move_ffwd(move_ffwd),
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
	void move_with_pid_time(Quantity<Meters, double> displacement, double max_time, int max_voltage) {
		auto start_displacement = chassis.average_motor_displacement();
		auto target_displacement = dlib::relative_target(start_displacement, displacement);
		auto start_time = pros::millis();	
		double elapsed_time = 0;

		move_pid.reset();
		time_move_settler.reset();

		while (!time_move_settler.is_settled(move_pid.get_error(), milli(seconds)(5)) /*&& (elapsed_time < max_time)*/) {
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
	void move_pid_ffwd(Quantity<Meters, double> displacement, double max_time, int max_voltage) {
		auto start_displacement = chassis.average_motor_displacement();
		
		auto target_displacement = dlib::relative_target(start_displacement, displacement);
		auto start_time = pros::millis();	
		auto elapsed_time = 0;

		auto profile = dlib::TrapezoidProfile<Meters>(meters_per_second_squared(1),meters_per_second(1.4), displacement);

		move_pid.reset();
		time_move_settler.reset();

		while (profile.stage(milli(seconds)(elapsed_time)) != dlib::TrapezoidProfileStage::Done) /*&& (elapsed_time < max_time)*/ {
			elapsed_time += 20;
			auto setpoint = profile.calculate(milli(seconds)(elapsed_time));
			auto voltage = move_ffwd.calculate(setpoint.velocity, setpoint.acceleration);
			
			if (profile.stage(milli(seconds)(elapsed_time)) == dlib::TrapezoidProfileStage::Decelerating) {
				chassis.move_voltage(volts(0));
			}
			
			
			chassis.move_voltage(voltage);
			pros::delay(20);
			std::cout << elapsed_time << "," << voltage.in(volts) << "," << chassis.average_motor_velocity().in(meters_per_second) << "," << setpoint.velocity.in(meters_per_second) << "\n";
		}

		chassis.move(0);
	}	

	// Use PID to do a relative movement
	void turn_with_pid_time(Quantity<Degrees, double> heading, double max_time, int max_voltage) {
		auto target_heading = heading;

		auto start_time = pros::millis();	
		double elapsed_time = 0;	

		turn_pid.reset();
		time_turn_settler.reset();

		auto voltage = volts(0.0);

		while (!time_turn_settler.is_settled(turn_pid.get_error(), milli(seconds)(20))) {

			auto error = dlib::angular_error(heading, imu.get_rotation());

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


	

	void turn_to_point(dlib::Vector2d point, bool reverse, int max_voltage, double max_time) {
		auto angle = odom.angle_to(point, reverse);

		turn_with_pid_time(angle,max_time,max_voltage);
	}

	void move_to_point(dlib::Vector2d point, bool reverse, int move_max_voltage, int turn_max_voltage, double max_time = 15000) {
		turn_to_point(point, reverse,turn_max_voltage, max_time);

		auto displacement = odom.displacement_to(point);
		if(reverse){
			displacement = -displacement;
		}
		move_with_pid_time(displacement,max_time, move_max_voltage);
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
	45, 	// kp, porportional gain
	0, 	// ki, integral gain
	.085* 39.37	// kd, derivative gain
};

dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler {
	degrees(1),		// error threshold, the maximum error the pid can settle at
	degrees_per_second(0.1)	// derivative threshold, the maximum instantaneous error over time the pid can settle at
};

dlib::ErrorTimeSettler<Degrees> time_turn_pid_settler {
	degrees(1),		// error threshold, the maximum error the pid can settle at
	milli(seconds)(100) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

dlib::Feedforward<Meters> move_feedforward{
	{0.94,5.1,1}
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
	turn_pid_settler,
	move_feedforward
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
	robot.move_to_point({inches(-14), inches(0)}, true, 12000, 12000,1000);
	robot.move_to_point({inches(-14), inches(-6)}, true, 12000, 12000, 600);
	
	// Intake onto alliance stake
	intake.max_intake();
	pros::delay(450);

	// Move to safe Mogo
	robot.move_to_point({inches(-15.5), inches(5)}, false, 12000, 12000,1000);
	robot.move_to_point({inches(12), inches(36)}, true, 5500, 12000,1500);
	
	// Clamp mogo
	mogo.set_clamp_state(true);

	// Intake safe ring
	robot.move_to_point({inches(32), inches(32)}, false, 12000, 8000,1000);
	
	// move + intake middle rings
	robot.move_to_point({inches(18), inches(34)}, true, 12000, 12000,1000);
	pros::delay(300);
	robot.move_to_point({inches(24), inches(43)}, false, 12000, 12000,1000);
	robot.move_to_point({inches(32), inches(45)}, false, 12000, 6000,1000);

	// touch bar
	robot.move_to_point({inches(23), inches(41)}, true, 12000, 6000,1000);
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
void red_goal_side(){
	double start = pros::millis();
	// move to mid mogo
	robot.move_to_point({inches(23), inches(0)}, false, 12000, 12000, 650);
	robot.move_to_point({inches(38), inches(-5)}, false, 12000, 12000, 650);
	// tip it
	misc_pneumatics.set_doinker_state(true);
	pros::delay(300);
	// score preload
	robot.turn_to_point({inches(38), inches(-8)}, false, 9000, 1000);
	pros::delay(300);
	misc_pneumatics.set_doinker_state(false);
	robot.turn_to_point({inches(38), inches(0)}, false, 9000, 1000);
	robot.chassis.move(-60);
	pros::delay(300);
	robot.move_to_point({inches(27), inches(0)}, true,12000,12000,1000);

	// sweep
	pros::delay(300);
	robot.move_to_point({inches(25.5), inches(-15)}, false,12000,12000,1000);


	// grab other mogo
	robot.move_to_point({inches(28), inches(-28)}, true,5500, 7000, 1500);

	mogo.set_clamp_state(true);

	intake.max_intake();

	robot.move_to_point({inches(16.5), inches(-5)}, false,5500, 7000, 1500);

	auto elapsed_time = pros::millis() - start;

	std::cout << elapsed_time << std::endl;
}

void blue_goal_side(){
	double start = pros::millis();
	// move to mid mogo
	robot.move_to_point({inches(23), inches(0)}, false, 12000, 12000, 650);
	robot.move_to_point({inches(38), inches(5)}, false, 12000, 12000, 650);
	// tip it
	misc_pneumatics.set_doinker_state(true);
	pros::delay(300);
	// score preload
	robot.turn_to_point({inches(38), inches(0)}, false, 9000, 1000);
	pros::delay(300);
	misc_pneumatics.set_doinker_state(false);
	robot.chassis.move(-60);
	pros::delay(300);
	robot.move_to_point({inches(27), inches(0)}, true,12000,12000,1000);

	// sweep
	pros::delay(300);
	robot.move_to_point({inches(25.5), inches(15)}, false,12000,12000,1000);


	// grab other mogo
	robot.move_to_point({inches(28), inches(28)}, true,5500, 7000, 1500);

	mogo.set_clamp_state(true);

	intake.max_intake();

	pros::delay(1000);

	intake.stop_intake();

	robot.move_to_point({inches(16.5), inches(5)}, false,5500, 7000, 1500);

	auto elapsed_time = pros::millis() - start;

	std::cout << elapsed_time << std::endl;


	// intake
	// -25.5 -15 false
	// wait .5 sec
	// intake stop
	// -25 -28 true
	// mogo clamp
	// intake



}

void blue_awp(){
	double cur_time = pros::millis();
	// Try a PID movement!
	// Move to Alliance Stake
	robot.move_to_point({inches(-14), inches(0)}, true, 12000, 12000);
	robot.move_to_point({inches(-14), inches(6)}, true, 12000, 12000, 600);
	
	// Intake onto alliance stake
	intake.max_intake();
	pros::delay(450);

	// Move to safe Mogo
	robot.move_to_point({inches(-15.5), inches(-5)}, false, 12000, 12000);
	robot.move_to_point({inches(12), inches(-36)}, true, 5500, 12000);
	
	// Clamp mogo
	mogo.set_clamp_state(true);

	// Intake safe ring
	robot.move_to_point({inches(32), inches(-34)}, false, 12000, 8000);
	
	// move + intake middle rings
	robot.move_to_point({inches(18), inches(-34)}, true, 12000, 12000);
	pros::delay(300);
	robot.move_to_point({inches(24), inches(-45)}, false, 12000, 12000);
	robot.move_to_point({inches(32), inches(-47)}, false, 12000, 6000);

	// touch bar
	robot.move_to_point({inches(23), inches(-41)}, true, 12000, 6000);
	pros::delay(750);
	lift.lift_move(127);
	//misc_pneumatics.set_arm_state(true);
	robot.move_to_point({inches(2), inches(-43)}, false, 12000, 12000);
	// 18 -41 true
	// wait a sec
	// raise lift
	double elapsed_time = pros::millis() - cur_time;

	std::cout << "autonomous finished in " << elapsed_time << std::endl;
}

void skills(){
	// intake onto ally stake
	intake.max_intake();
	pros::delay(600);

	// move to and clamp mogo
	robot.move_to_point({inches(13.5), inches(0)}, false, 12000, 12000);
	robot.move_to_point({inches(14), inches(23)}, true, 5500, 8000);
	mogo.set_clamp_state(true);

	// intake ring 1
	robot.move_to_point({inches(40.6), inches(23)}, false, 9000, 12000);

	// intake ring 2
	robot.move_to_point({inches(58), inches(48)}, false, 9000, 12000);
	pros::delay(500);

	// intake ring 3
	robot.move_to_point({inches(47), inches(49)}, false, 9000, 12000);
	
	// intake rings 4 and 5
	robot.move_to_point({inches(8), inches(47)}, false, 3000, 12000);
	pros::delay(1000);

	// intake ring 6
	robot.move_to_point({inches(15.5), inches(37.5)}, true, 12000, 12000);
	robot.move_to_point({inches(14.7), inches(52.3)}, false, 7000, 12000);
	pros::delay(500);

	// drop mogo
	robot.move_to_point({inches(5), inches(60)}, true, 8000, 8000);
	mogo.set_clamp_state(false);

	// PART 2 (MIRROR)

	robot.move_to_point({inches(15), inches(50)}, false, 8000, 8000);

	robot.move_to_point({inches(11), inches(6)}, true, 8000, 8000);

	// move to and clamp mogo
	robot.move_to_point({inches(13.5), inches(0)}, false, 12000, 12000);
	robot.move_to_point({inches(14), inches(-23)}, true, 5500, 8000);
	mogo.set_clamp_state(true);

	// intake ring 1
	robot.move_to_point({inches(40.6), inches(-23)}, false, 9000, 12000);

	// intake ring 2
	robot.move_to_point({inches(58), inches(-48)}, false, 9000, 12000);
	pros::delay(1000);

	// intake ring 3
	robot.move_to_point({inches(47), inches(-46)}, false, 9000, 12000);
	
	// intake rings 4 and 5
	robot.move_to_point({inches(8), inches(-47)}, false, 3000, 12000);
	pros::delay(2000);

	// intake ring 6
	robot.move_to_point({inches(15.5), inches(-37.5)}, true, 12000, 12000);
	robot.move_to_point({inches(14.7), inches(-52.3)}, false, 7000, 12000);
	pros::delay(1000);

	// drop mogo
	robot.move_to_point({inches(5), inches(-60)}, true, 8000, 8000);
	mogo.set_clamp_state(false);
}

void on_center_button() {}

rd::Selector selector({
    {"Red Ring Side", red_awp},
	{"Red Goal Side", red_goal_side},
	{"Blue Ring Side", blue_awp},
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
	
	

	pros::Task screen_task([&]() {
        while (true) {
			dlib::Pose2d pose = robot.odom.get_position();
			
			auto elapsed = pros::millis() - current;
			auto voltage = 3;
			console.clear();
            // print robot location to the brain screen
            console.printf("X: %f", pose.x.in(inches)); // x
            console.printf("Y: %f", pose.y.in(inches)); // y
            console.printf("Theta: %f",pose.theta.in(degrees));
			std::cout << elapsed << "," << voltage << "," << robot.chassis.average_motor_displacement().in(meters) << "," << robot.chassis.average_motor_velocity().in(meters_per_second) << "\n";
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
	while(voltage < 12000){
		auto elapsed = pros::millis() - cur_time;
		
		auto delta_position = robot.chassis.average_motor_displacement() - last_position;
		auto velocity = delta_position / seconds(0.02);
		voltage = elapsed * .5;


		robot.chassis.move_voltage(milli(volts)(voltage));

		last_position = robot.chassis.average_motor_displacement();
		pros::delay(20);
	}
	robot.chassis.move_voltage(volts(0));

	//robot.chassis.move_voltage(volts(1));
	//selector.run_auton();
	//robot.move_pid_ffwd(inches(48),99999,12000);

}	



void opcontrol() {
	bool mogo_state = true;
	bool arm_state = false;
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

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake.intake(127);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			intake.rev_intake();
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
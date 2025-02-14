#include "main.h"
#include "au/au.hpp"
#include "dlib/controllers/error_derivative_settler.hpp"
#include "dlib/controllers/feedforward.hpp"
#include "dlib/dlib.hpp"
#include "dlib/hardware/rotation.hpp"
#include "dlib/hardware/timer.hpp"
#include "dlib/kinematics/odometry.hpp"
#include "dlib/trajectories/trapezoid_profile.hpp"
#include "dlib/utilities/error_calculation.hpp"
#include "intake.hpp"
#include "liblvgl/misc/lv_area.h"
#include "misc_pneumatics.hpp"
#include "mogo.hpp"
#include "lift.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/colors.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <functional>
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

	dlib::Pid<Meters> ramsete_pid;

	dlib::Pid<Degrees> turn_pid;
	dlib::ErrorDerivativeSettler<Degrees> turn_settler;

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
			//std::cout << error << std::endl;
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

			std::cout << elapsed_time << "," << setpoint.velocity.in(meters_per_second) << "," << chassis.forward_motor_velocity().in(meters_per_second) << "\n";

			chassis.move_voltage(ff_voltage + pid_voltage);
			
			pros::delay(20);
		}
		chassis.move_voltage(volts(0));
	}

	// Use PID to do a relative movement
	void move_with_pid(Quantity<Meters, double> displacement, double max_voltage, double max_time=99999) {
		auto start_displacement = chassis.forward_motor_displacement();
		auto target_displacement = dlib::relative_target(start_displacement, displacement);
		
		ramsete_pid.reset();
		move_settler.reset();

		auto start_time = pros::millis();

		while (!move_settler.is_settled(ramsete_pid.get_error(), ramsete_pid.get_derivative())) {
			auto error = dlib::linear_error(target_displacement, chassis.forward_motor_displacement());
			auto voltage = ramsete_pid.update(error, milli(seconds)(20));
			auto current_time = pros::millis();
			auto elapsed_time = current_time - start_time;
			if(std::abs(voltage.in(volts)) > max_voltage){
				if(voltage.in(volts) >= 0){
					voltage = volts(max_voltage);
				}
				else if(voltage.in(volts) < 0){
					voltage = volts(-max_voltage);
				}
			}
			if(elapsed_time > max_time){
				break;
			}
			std::cout << error << std::endl;
			//std::cout << voltage << std::endl;
			chassis.move_voltage(voltage);

			pros::delay(20);
		}
		std::cout << "settled" << std::endl;
	}

	void move_with_pid(double displacement, double max_voltage, double max_time=3000) {
		move_with_pid(inches(displacement),max_voltage);
	}

	void turn_with_pid(Quantity<Degrees, double> heading) {

		turn_pid.reset();
		turn_settler.reset();

		auto initial_error = dlib::angular_error(heading, imu.get_rotation());

		double timeout = std::clamp(heading.in(degrees) * 10,1000.0,10000.0);

		auto timer = dlib::Timer();
		timer.reset();

		while (!turn_settler.is_settled(turn_pid.get_error(), turn_pid.get_derivative())) {
			if(timer.get_elapsed_time().in(milli(seconds)) > timeout){
				break;
			}
			if(au::abs(initial_error) < degrees(3)){
				break;
			}
			auto error = dlib::angular_error(heading, imu.get_rotation());
			auto voltage = turn_pid.update(error, milli(seconds)(20));
			if(voltage.in(volts) > 0){
				chassis.turn_voltage(voltage + volts(.7));
			}
			if(voltage.in(volts) <= 0){
				chassis.turn_voltage(voltage - volts(.7));
			}
			

			pros::delay(20);
		}
		chassis.brake();
	}

	void turn_with_pid(double heading) {
		turn_with_pid(degrees(heading));
	}

	void turn_ffwd(double time){
		chassis.left_motors.raw.set_current_limit_all(2400);
		chassis.right_motors.raw.set_current_limit_all(2400);

		/*double a = -1844.1699;
		double k = -4.71929;*/ // for ccw 8v
		
		double a = 1403.04827;
		double k = -4.67879;

		time /= 1000.0;
		
		if(time >= 0){
			auto start_time = pros::millis();

			turn_pid.reset();
			turn_settler.reset();

			while(true) {
				auto elapsed_time = (pros::millis() - start_time)/1000.0;

				auto heading = ((a * (std::exp(k * elapsed_time) - k * elapsed_time))) / (k * k) - (a / (k * k));

				
				auto velocity = (a / k) * (std::exp(k * time)) - a/k;
				auto acceleration = a * (std::exp(k * time));
				
				auto error = dlib::angular_error(degrees(heading), imu.get_rotation());
				auto voltage = turn_pid.update(error, milli(seconds)(20));
				
				std::cout 
				<< elapsed_time << ", " 
				<< heading << ", " 
				<< imu.get_rotation() << ", "
				<< error << ", " 
				<< voltage << ", "
				<< std::endl;
				//std::cout << voltage  << std::endl;
				
				
				if(elapsed_time >= time){
					std::cout << "broke" << std::endl;
					break;
				}

				
				chassis.turn_voltage(volts(6.0));


				pros::delay(20);
			}

			std::cout << imu.get_rotation() << std::endl;
			//chassis.turn_voltage(volts(12));
			//pros::delay(60);
			chassis.brake();
		}
	}

	// Use arctan formula to turn to a relative point
	void turn_to_point(double x, double y, bool reverse = false, double max_time = 15000, int max_voltage = 5500) {
		auto point = dlib::Vector2d(inches(x), inches(y));
		auto angle = odom.angle_to(point, reverse);

		turn_with_pid(angle.in(degrees));
	}

	// Use pythagorean formula to move to a relative point
	void move_to_point(double x, double y, double max_velocity, bool reverse = false, double max_time = 3000, int turn_max_voltage = 12000) {
		auto point = dlib::Vector2d(inches(x), inches(y));
		turn_to_point(x, y, reverse,max_time, turn_max_voltage);
		auto displacement = odom.displacement_to(point);
		if(reverse){
			displacement = -displacement;
		}
		move_ffwd(displacement.in(inches),3000,max_velocity);
	}

	// can it use M_PI???
	double sinc(double x) {
		if (x == 0.0) {
			return 1.0;
		}
		return std::sin(x) / (x);
	}

	double signdetect(double num) {
	if (std::signbit(num)) {
		return 1;
	} else {
		return -1;
	}
	}

	// fwds true = mogo side
	// early_exit allows for waypoints to be set up (2 inches for waypoints??? 0.25 should be target for end)
	void ramseteTest(dlib::Vector2d point, bool fowards, double max_voltage, double min_voltage, double early_exit) {

	double track_width = 0.15; // stay in meters
	double k_lat = 3; // btwn 2.5-3

	// double lastvL = 0, lastvR = 0;

	// setting up PIDs
	ramsete_pid.reset();
	turn_pid.reset();

	// global errors in meters and degrees
	auto theta = (au::degrees)(0.0);
	auto x_g = (au::meters)(0.0);
	auto y_g = (au::meters)(0.0);

	// local errors in meters and degrees
	auto e_theta = (au::degrees)(0.0);
	auto e_x = (au::meters)(0.0);
	auto e_y = (au::meters)(0.0);

	// error from the point using pythagorean theorem
	auto tri_error = (au::meters)(0.0);

	// pid voltage outputs
	auto v_d = (au::volts)(0.0);
	auto a_velo = (au::volts)(0.0);

	// voltage output difference
	double l_velo = 0.0;
	double veloDiff = 0.0;

	// final left and right voltage output
	double v_L = 0.0;
	double v_R = 0.0;

	// scaling to ratio speed & voltage step rate
	double scaling_factor;
	double voltage_slew = 4; //4
	double step = 1.5; // +1 volt per 20 ms so 240 ms for 12 v

	// start el loop
	while (true) {
		dlib::Pose2d curPos = odom.get_position();
		// current theta and x, y error. should be in meters!!!!
		theta = curPos.theta;
		x_g = point.x - curPos.x;
		y_g = point.y - curPos.y;

		std::cout << "(" << curPos.x.in(inches) << "," << curPos.y.in(inches)<< "),";

		// local x & y error
		e_x = (cos(theta.in(au::radians)) * x_g) + (sin(theta.in(au::radians)) * y_g);
		e_y = ((-sin(theta.in(au::radians))) * x_g) + (cos(theta.in(au::radians)) * y_g);

		if (fowards) {
		e_theta = (au::radians)(atan2(-e_y.in(au::meters), -e_x.in(au::meters)));
		} else {
		e_theta = (au::radians)(atan2(e_y.in(au::meters), e_x.in(au::meters)));
		}


		// PID outputs volts soooo.... gotta convert it to double so it can "act" like velo PID
		// nvm gonna test with -12 & 12 clamp first
		tri_error = (au::meters)((sqrt(pow(e_x.in(au::meters), 2) + pow(e_y.in(au::meters), 2)) * signdetect(cos(e_theta.in(au::radians)))));
		v_d = ramsete_pid.update(tri_error, milli(seconds)(20));
		a_velo = turn_pid.update(e_theta, milli(seconds)(20)); // why is this not used???
		//std::cout << "tri: " << tri_error << std::endl;
		//std::cout << "e theta: " << e_theta << std::endl;

		l_velo = (fabs(cos(e_theta.in(au::radians))) * v_d.in(au::volts));
		// veloDiff = ((a_velo.in(au::volts)/12.0) * k_lat * e_y.in(au::meters) * sinc(e_theta.in(au::radians)))/0.25; //8
		veloDiff = ((a_velo.in(au::volts)/12.0) * k_lat * sinc(e_theta.in(au::radians)))/track_width; //8

		// veloDiff = a_velo.in(au::volts) * sinc(e_theta.in(au::radians));

		if (fowards) {
		v_L = -l_velo + veloDiff; // l_velo should be - for intake side
		v_R = -l_velo - veloDiff; // l_velo should be - for intake side
		} else {
		v_L = l_velo + veloDiff;
		v_R = l_velo - veloDiff;
		}

		// slew rate control voltage slew over time
		voltage_slew += step;

		if (voltage_slew > max_voltage) {
		voltage_slew = max_voltage;
		}


		// Ratio left and right voltages if it exceeds user max_voltage
		// also ratios it if it is lower than min_voltage
		// possible for voltage to exceed user defined max_voltage or min_voltage
		// but it is more important to keep them at the same ratio than under the same limit.
		if ((fabs(v_L) > voltage_slew) || (fabs(v_R) > voltage_slew)) {
		if (v_L > voltage_slew) {
			scaling_factor = voltage_slew / fabs(v_L);
		} else {
			scaling_factor = voltage_slew / fabs(v_R);
		}
		v_L = v_L * scaling_factor;
		v_R = v_R * scaling_factor;
		} else if ((fabs(v_L) < min_voltage) || (fabs(v_R) < min_voltage)) {
		if (v_L < min_voltage) {
			scaling_factor = min_voltage / fabs(v_L);
		} else {
			scaling_factor = min_voltage / fabs(v_R);
		}
		v_L = v_L * scaling_factor;
		v_R = v_R * scaling_factor;
		}

		// move the motors
		chassis.left_motors.move_voltage(volts(-v_L));
		chassis.right_motors.move_voltage(volts(-v_R));

		
		// Settle condition? -> nah Try to reduce error to .25 or +-.25 in either x or y
		if (fabs(tri_error.in(au::inches)) < early_exit) {
		break;
		}

		// std::cout << "(" << curPos.x.in(au::inches) << "," << curPos.y.in(au::inches) << ")" << std::endl;
		//std::cout << "(" << e_theta << ")" << std::endl;
		pros::delay(20);
	}
	chassis.brake(); // should be coast?

	}

	// Start the odom task responsible for updating coordinates
	// Odom task
	void start_odom() {
		odom_updater = std::make_unique<pros::Task>([this]() {
			while (true) {
				odom.update(
					chassis.left_motors_displacement(), 
					chassis.right_motors_displacement(), 
					//left_odom.get_linear_displacement(),
					//right_odom.get_linear_displacement(),
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
	{125, 	// kp, porportional gain
	0,
	0 //0, 	// ki, integral gain
	},
	volts(12) //11.8	// kd, derivative gain
};

// Chassis move PID settler
dlib::ErrorDerivativeSettler<Meters> move_pid_settler {
	inches(1),		// error threshold, the maximum error the pid can settle at
	meters_per_second(0.005) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

dlib::PidConfig ramsete_pid_gains {
	{45,
	0,
	3
	},
	volts(12)
};

// Chassis move Feedforward gains
dlib::Feedforward<Meters> move_feedforward {
	{
		1.0282869641475763,
	5.25,
	1.2
	}
};

dlib::Feedforward<Degrees> turn_feedforward {
	{
		1.97079,0.0162841,0.00418636118461
	}
};

// ------------------------------ //
// Turn PID Gains + Settler //
// ------------------------------ //

// Chassis turn PID gains
dlib::PidConfig turn_pid_gains {
	{
	18,0,.325
	},
	volts(7)
};

// Chassis turn PID settler
dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler {
	degrees(1.5),		// error threshold, the maximum error the pid can settle at
	degrees_per_second(1.5)	// derivative threshold, the maximum instantaneous error over time the pid can settle at
};

Intake intake(
	-20,
	12
);

// Mogo object
Mogo mogo(
	'B',
	false
);

// Lift object
Lift lift(
	-4,
	10,
	-18
);

// MiscPneumatics object
MiscPneumatics misc_pneumatics(
	'C',
	false,
	'A',
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
	ramsete_pid_gains,
	turn_pid_gains,
	turn_pid_settler,
	move_feedforward,
	turn_feedforward
};

struct ExponentialProfileState {
	const double displacement;
	const double velocity;
	const double acceleration;

	ExponentialProfileState(double displacement, double velocity, double acceleration) 
	: displacement(displacement), velocity(velocity), acceleration(acceleration) {

	}
};

struct ExponentialProfileTiming {
	const double inflection_time;
	const double total_time;

	ExponentialProfileTiming(double inflection_time, double total_time) : inflection_time(inflection_time), total_time(total_time) {

	}
};

class ExponentialProfile {
	const double A;
	const double k;
	const double u;

	double v_max;
	double t_decel;
	double d_decel;

	const double time_upper_bound = 6;
	const double newton_iterations = 4;
public:
	ExponentialProfile(double A, double k, double u) : A(A), k(k), u(u) {
		v_max = -(u * A) / k;
        t_decel = -std::log(2) / k;
        d_decel = decel_displacement_from_time(t_decel);
	}

	double accel_acceleration_from_time(double time) {
		// from exponential decay of acceleration of constant voltage
		return A * u * std::exp(k*time);
	}

	double accel_velocity_from_time(double time) {
		// integral of above function
        return ((A * u) * (std::exp(k * time) - 1)) / k;
	}
    
    double accel_displacement_from_time(double time) {
        // integral of above function
        return ((A * u) / (k * k)) * (std::exp(k * time) - k * time - 1);
	}

	double accel_time_from_displacement(double displacement) {
        // get an initial estimate from a linear approximation
        double estimate = (time_upper_bound * displacement) / accel_displacement_from_time(time_upper_bound);
        
        for (int i = 0; i < newton_iterations; i++) {
			estimate = estimate - (accel_displacement_from_time(estimate) - displacement) / accel_velocity_from_time(estimate);
		}
        
        return estimate;
	}

	double decel_acceleration_from_time(double time){
        return -2 * A * u * std::exp(k * time);
	}
    
    double decel_velocity_from_time(double time){
        return v_max * (2 * std::exp(k * time) - 1);
	}

    double decel_displacement_from_time(double time){
        return v_max * ((2 * std::exp(k * time) / k) - (2 / k) - time);
	}

	ExponentialProfileTiming calculate_timing(double displacement){
        double inflection_time = accel_time_from_displacement(displacement - d_decel);
        double total_time = inflection_time + t_decel;
        return ExponentialProfileTiming(inflection_time, total_time);
	}
    
    ExponentialProfileState calculate(double displacement, double time){
        ExponentialProfileTiming timing = calculate_timing(displacement);

        if(time < timing.inflection_time){
            double accel = accel_acceleration_from_time(time);
            double velo = accel_velocity_from_time(time);
            double disp = accel_displacement_from_time(time);

            return ExponentialProfileState(disp, velo, accel);
		}
        else if(time < timing.total_time){
            double accel = decel_acceleration_from_time(time - timing.inflection_time);
            double velo = decel_velocity_from_time(time - timing.inflection_time);
            double disp = accel_displacement_from_time(timing.inflection_time) + decel_displacement_from_time(time - timing.inflection_time);

            return ExponentialProfileState(disp, velo, accel);
		}
        else {
            return ExponentialProfileState(0, 0, 0);
		}
	}
};


Alliance alliance;

bool auto_clamp = false;
bool clamped = false;

void brake(){
	robot.chassis.move_voltage(volts(1));
	pros::delay(50);
	robot.chassis.move_voltage(volts(0));
	pros::delay(100);
	robot.chassis.move_voltage(volts(1));
	pros::delay(75);
	robot.chassis.move_voltage(volts(0));
	pros::delay(100);
}

void shake(){
	robot.chassis.move_voltage(volts(12));
	pros::delay(75);
	robot.chassis.move_voltage(volts(-1));
	pros::delay(100);
	robot.chassis.move_voltage(volts(0));
	pros::delay(75);
}

bool do_intake = false;

void click(){
	intake.intake_motor.move_voltage(-12000);
	pros::delay(100);
	intake.intake_motor.move_voltage(9000);
	pros::delay(400);
	do_intake = false;
}

void driver_macro(){
	lift.lift_rot.set_position(1300);
	lift.set_state(LiftState::Stake);
	pros::delay(700);
	auto_clamp = true;

	while(auto_clamp){
		pros::delay(20);
	}

	lift.set_state(LiftState::Idle);
}

bool intake_max = false;


// unfinished.
void skills(){
	driver_macro();
	do_intake = true;
	// ring 1
	robot.move_to_point(-25.06,13.82,1.6);
	// special
	robot.move_to_point(-62.12,27.76,1.6);
	robot.move_to_point(-77.56,35.71,1.6);
	// rev into wall
	robot.move_to_point(-57.31,24.94,1.6,true);
	lift.set_state(LiftState::Prime);
	robot.move_to_point(-73.31,9.25,.5);
	pros::delay(1100);
	do_intake = false;
	lift.set_state(LiftState::Stake);
	pros::delay(1100);
	lift.set_state(LiftState::Idle);
	robot.move_ffwd(-7);

	do_intake = true;
	robot.move_to_point(-49.8,3.94,1.6);
	robot.move_to_point(-21.23,-25.60,0.5);

	robot.move_to_point(-38.83,-6.29,1.6,true);
	robot.move_to_point(-38.74,-19.30,1);
	pros::delay(300);
	robot.turn_to_point(-33.89,-31.93,true);
	mogo.unclamp();
	robot.move_to_point(-33.89,-31.93,0.5,true);
	mogo.unclamp();
	shake();
	robot.move_ffwd(10);

	robot.move_to_point(6.01,13.78,1.6,true);
	robot.turn_to_point(22.10,25.69,true);
	auto_clamp = true;

	while(auto_clamp){
		pros::delay(20);
	}

	robot.move_to_point(9.39,43.86,1.6);
	robot.move_to_point(4.88,77.83,1.6);
	robot.move_to_point(-4.85,96.70,1.6);

	// newark, connecticut
	robot.move_to_point(7.99,76.42,1.6,true);
	lift.set_state(LiftState::Prime);
	robot.move_to_point(19.37,87.34,0.7);
	pros::delay(1100);
	lift.set_state(LiftState::Stake);
	pros::delay(1100);
	click();
	lift.set_state(LiftState::Idle);
	robot.move_ffwd(-7);
	do_intake = true;
	robot.move_to_point(23.28,65.269,1.6); // Nice!
	robot.move_to_point(48.769,37.3,0.5); // Nice!
	robot.move_to_point(32.71,56.42,1.6,true);
	robot.move_to_point(44.48,53.62,1.6);
	robot.turn_to_point(56.45,46.18,true);
	mogo.unclamp();
	robot.move_to_point(56.45,46.18,0.5,true);
	do_intake = false;
	shake();
	robot.move_to_point(4.71,79.82,1.6);
	robot.move_to_point(-64.01,81.70,1.6);
	robot.move_to_point(-53.23,45.27,1.6);
	robot.move_ffwd(-5);
	// -58.76 55.77
	//-64.01 81.70
	// stake
	// -5 idle
	// -22.9 125.59 bang
}

// immensely consistent
void blue_ring_wp(){
// move into alliance stake and score a ring
	robot.chassis.move_voltage(volts(2));
	intake.set_alliance(Alliance::Blue);
	lift.lift_rot.set_position(1300);
	lift.set_state(LiftState::Stake);
	pros::delay(700);
	robot.chassis.move_voltage(volts(-5));
	pros::delay(350);

	// brake out of manual control
	brake();

	// move lift back up
	lift.set_state(LiftState::Idle);

	// -29.4 14.5
	robot.turn_to_point(-32.4, 14.5,true);
	std::cout << "settled" << std::endl;
	auto_clamp = true;
	
	while(auto_clamp){
		pros::delay(20);
	}

	do_intake = true;
	robot.move_to_point(-55.0,5.3,1);
	robot.move_to_point(-62.5,1.7,1);
	pros::delay(300);
	// -61.5 3.7
	
	// curve out of rings to not cross
	robot.chassis.left_motors.move_voltage(volts(-10));
	robot.chassis.right_motors.move_voltage(volts(-6));
	pros::delay(450);
	brake();
	do_intake = false;
	robot.turn_to_point(-46.7, 1.8);
	do_intake = true;
	robot.move_to_point(-46.7, 1.8, 1.6);
	robot.turn_to_point(-8.1,14.2);
	misc_pneumatics.set_indexer_state(true);
	robot.move_to_point(-11.1,13.2, 1.6);
	misc_pneumatics.set_indexer_state(false);
	// -8.1 14.2 indexer thing
	robot.move_ffwd(-6);
	robot.turn_to_point(-18.01,19.07);
	robot.chassis.move_voltage(volts(5));
	pros::delay(200);
	lift.set_state(LiftState::Stake);
}

void blue_ring_elim(){
// move into alliance stake and score a ring
	robot.chassis.move_voltage(volts(2));
	intake.set_alliance(Alliance::Blue);
	lift.lift_rot.set_position(1300);
	lift.set_state(LiftState::Stake);
	pros::delay(700);
	robot.chassis.move_voltage(volts(-5));
	pros::delay(350);

	// brake out of manual control
	brake();

	// move lift back up
	lift.set_state(LiftState::Idle);

	// -29.4 14.5
	robot.turn_to_point(-32.4, 14.5,true);
	std::cout << "settled" << std::endl;
	auto_clamp = true;
	
	while(auto_clamp){
		pros::delay(20);
	}

	/*do_intake = true;
	robot.move_to_point(-55.0,6.3,1);
	robot.move_to_point(-62.5,1.7,1);
	pros::delay(300);
	// -61.5 3.7
	
	// curve out of rings to not cross
	robot.chassis.left_motors.move_voltage(volts(-10));
	robot.chassis.right_motors.move_voltage(volts(-6));
	pros::delay(450);
	brake();
	do_intake = false;*/
	robot.turn_to_point(-46.7, 1.8);
	do_intake = true;
	robot.move_to_point(-46.7, 1.8, 1.6);
	pros::delay(500);
	robot.turn_to_point(-8.1,9.2);
	misc_pneumatics.set_indexer_state(true);
	robot.move_to_point(-8.1,9.2, 1.6);
	misc_pneumatics.set_indexer_state(false);
	// -8.1 14.2 indexer thing
	robot.move_ffwd(-6);
}

// immensely consistent
void red_ring_wp(){
// move into alliance stake and score a ring
	robot.chassis.move_voltage(volts(2));
	intake.set_alliance(Alliance::Blue);
	lift.lift_rot.set_position(1300);
	lift.set_state(LiftState::Stake);
	pros::delay(700);
	robot.chassis.move_voltage(volts(-5));
	pros::delay(350);

	// brake out of manual control
	brake();

	// move lift back up
	lift.set_state(LiftState::Idle);

	// -29.4 14.5
	robot.turn_to_point(-32.4, -14.5,true);
	std::cout << "settled" << std::endl;
	auto_clamp = true;
	
	while(auto_clamp){
		pros::delay(20);
	}

	do_intake = true;
	robot.move_to_point(-55.0,-7.3,1);
	robot.move_to_point(-62.5,-3.7,1);
	pros::delay(300);
	// -61.5 3.7
	
	// curve out of rings to not cross
	robot.chassis.left_motors.move_voltage(volts(-6));
	robot.chassis.right_motors.move_voltage(volts(-10));
	pros::delay(450);
	brake();
	do_intake = false;
	robot.turn_to_point(-46.7, -1.8);
	do_intake = true;
	robot.move_to_point(-46.7, -1.8, 1.6);
	robot.turn_to_point(-6.1,-14.2);
	misc_pneumatics.set_indexer_state(true);
	robot.move_to_point(-6.1,-14.2, 1.6);
	misc_pneumatics.set_indexer_state(false);
	// -8.1 14.2 indexer thing
	robot.move_ffwd(-6);
	robot.turn_to_point(-18.01,-19.07);
	robot.chassis.move_voltage(volts(5));
	pros::delay(200);
	lift.set_state(LiftState::Stake);
}

void red_ring_elim(){
// move into alliance stake and score a ring
	robot.chassis.move_voltage(volts(2));
	intake.set_alliance(Alliance::Blue);
	lift.lift_rot.set_position(1300);
	lift.set_state(LiftState::Stake);
	pros::delay(700);
	robot.chassis.move_voltage(volts(-5));
	pros::delay(350);

	// brake out of manual control
	brake();

	// move lift back up
	lift.set_state(LiftState::Idle);

	// -29.4 14.5
	robot.turn_to_point(-32.4, -14.5,true);
	std::cout << "settled" << std::endl;
	auto_clamp = true;
	
	while(auto_clamp){
		pros::delay(20);
	}

	do_intake = true;
	/*robot.move_to_point(-55.0,-7.8,1);
	robot.move_to_point(-62.5,-2.5,1);
	pros::delay(300);
	// -61.5 3.7
	
	// curve out of rings to not cross
	robot.chassis.left_motors.move_voltage(volts(-6));
	robot.chassis.right_motors.move_voltage(volts(-10));
	pros::delay(450);
	brake();*/
	do_intake = false;
	robot.turn_to_point(-46.7, -1.8);
	do_intake = true;
	robot.move_to_point(-46.7, -1.8, 1.6);
	pros::delay(500);
	robot.turn_to_point(-4.1,-20.2);
	misc_pneumatics.set_indexer_state(true);
	robot.move_to_point(-4.1,-16.2, 1.6);
	misc_pneumatics.set_indexer_state(false);
	// -8.1 14.2 indexer thing
	robot.move_ffwd(-6);
}

void blue_goal(){
	// move into alliance stake and score a ring
	robot.chassis.move_voltage(volts(2));
	intake.set_alliance(Alliance::Blue);
	lift.lift_rot.set_position(1300);
	lift.set_state(LiftState::Stake);
	pros::delay(700);
	robot.chassis.move_voltage(volts(-5));
	pros::delay(350);

	// brake out of manual control
	brake();

	// move lift back up
	lift.set_state(LiftState::Idle);

	// -29.4 14.5
	robot.turn_to_point(-32.4, -14.5,true);
	std::cout << "settled" << std::endl;
	auto_clamp = true;
	
	while(auto_clamp){
		pros::delay(20);
	}

	do_intake = true;
	robot.move_to_point(-46.7, -1.8, 1.6);
	robot.turn_to_point(-8.1,-14.2);
	misc_pneumatics.set_indexer_state(true);
	robot.move_to_point(-8.1,-9.2, 1.6);
	misc_pneumatics.set_indexer_state(false);
	// -8.1 14.2 indexer thing
	robot.move_ffwd(-6);
}

void blue_goal_elim(){
	// move into alliance stake and score a ring
	robot.chassis.move_voltage(volts(2));
	intake.set_alliance(Alliance::Blue);
	lift.lift_rot.set_position(1300);
	lift.set_state(LiftState::Stake);
	pros::delay(700);
	robot.chassis.move_voltage(volts(-5));
	pros::delay(350);

	// brake out of manual control
	brake();

	// move lift back up
	lift.set_state(LiftState::Idle);

	// -29.4 14.5
	robot.turn_to_point(-32.4, -14.5,true);
	std::cout << "settled" << std::endl;
	auto_clamp = true;
	
	while(auto_clamp){
		pros::delay(20);
	}

	do_intake = true;
	robot.move_to_point(-46.7, -1.8, 1.6);
}

void red_goal(){
	// move into alliance stake and score a ring
	robot.chassis.move_voltage(volts(2));
	intake.set_alliance(Alliance::Blue);
	lift.lift_rot.set_position(1300);
	lift.set_state(LiftState::Stake);
	pros::delay(700);
	robot.chassis.move_voltage(volts(-5));
	pros::delay(350);

	// brake out of manual control
	brake();

	// move lift back up
	lift.set_state(LiftState::Idle);

	// -29.4 14.5
	robot.turn_to_point(-32.4, 14.5,true);
	std::cout << "settled" << std::endl;
	auto_clamp = true;
	
	while(auto_clamp){
		pros::delay(20);
	}

	do_intake = true;
	robot.move_to_point(-46.7, 1.8, 1.6);
	robot.turn_to_point(-8.1,14.2);
	misc_pneumatics.set_indexer_state(true);
	robot.move_to_point(-8.1,9.2, 1.6);
	misc_pneumatics.set_indexer_state(false);
	// -8.1 14.2 indexer thing
	robot.move_ffwd(-6);
}

void red_goal_elim(){
	// move into alliance stake and score a ring
	robot.chassis.move_voltage(volts(2));
	intake.set_alliance(Alliance::Blue);
	lift.lift_rot.set_position(1300);
	lift.set_state(LiftState::Stake);
	pros::delay(700);
	robot.chassis.move_voltage(volts(-5));
	pros::delay(350);

	// brake out of manual control
	brake();

	// move lift back up
	lift.set_state(LiftState::Idle);

	// -29.4 14.5
	robot.turn_to_point(-32.4, 14.5,true);
	std::cout << "settled" << std::endl;
	auto_clamp = true;
	
	while(auto_clamp){
		pros::delay(20);
	}

	do_intake = true;
	robot.move_to_point(-46.7, 1.8, 1.6);
}

bool auton = true;


pros::Distance dist(5);

void initialize() {
	robot.initialize();
	robot.start_odom();
	//selector.focus();
	lift.set_state(LiftState::Idle);

	robot.chassis.left_motors.raw.tare_position_all();
	robot.chassis.right_motors.raw.tare_position_all();

	pros::Task screen_task([&]() {
		dlib::Pose2d pose = robot.odom.get_position();
		auto last_x = pose.x;
		auto last_y = pose.y;
		auto last_theta = pose.theta;
        while (true) {
				dlib::Pose2d pose = robot.odom.get_position();

				std::cout << "x: " << pose.x.in(inches) << " y: " << pose.y.in(inches) << "\n";
				
	
				last_x = pose.x;
				last_y = pose.y;
				last_theta = pose.theta;
				pros::delay(1000);
		}
	});

	// Intake Filter Task //
	// ------------------ //
	// when the sensor detects a ring, enable one of two macros:
	// redirect: redirect a alliance color ring onto wall stake mech
	// fling: fling non-alliance rings so they dont get scored
	// shouldn't interfere with auton

	pros::Task lift_task([&]() {
		auto last_error = lift.get_lift_position() / 100;
		while(true){
			auto kP = 250;
			auto kD = 150;

			auto error = (lift.target_position - lift.get_lift_position()) / 100;
			auto delta_error = error - last_error;

			auto p = error * kP;
			auto d = (last_error/15) * kD;

			if(p + d > 0){
				lift.move_voltage(p + d + 200);
			}
			if(p + d <= 0){
				lift.move_voltage(p + d + 500);
			}
			last_error = lift.get_lift_position() / 100;
			//console.printf("lb rotation: %f",lift.get_lift_position() / 100);
			pros::delay(15);
		}
	});

	pros::Task clamp_auto([&]() {
		while(true){
			if(auto_clamp){
				if(dist.get() < 12){
					mogo.clamp();
					robot.chassis.move_voltage(volts(0));
					pros::delay(200);
					auto_clamp = false;
				}
				else{
					if(dist.get() < 225){
						robot.chassis.move_voltage(volts(-5.5));
					}
					else{
						auto voltage = volts(-(dist.get() / 60));
						voltage = au::clamp(voltage,volts(-10), volts(0));
						robot.chassis.move_voltage(voltage);
					}	
				}
			}
			pros::delay(20);
		}
	});

	pros::Task([&](){
		while(true){
			if(do_intake){
				intake.auto_max();
			}
			pros::delay(20);
		}		
	});
}

void disabled() {}

void competition_initialize() {}

void test_exp_turn() {
	// in rad/s
	double ks = 1.5779;
	//double ks = 1.97079;
	double kv = 0.93301;
	double ka = 0.239860827437;

	double A = 1 / ka;
	double k = -kv / ka;
	double u = 10;
	
	auto profile = ExponentialProfile(156.41685, -2.97049, u);
	//auto ff = dlib::Feedforward<Degrees>({ks, kv, ka});

	// 10v limit !!!
	
	auto timer = dlib::Timer();
	timer.reset();
	au::Quantity<Degrees, double> last_displacement = ZERO;

	auto timing = profile.calculate_timing(90);
	auto elapsed_time = timer.get_elapsed_time();

	while (elapsed_time < seconds(timing.total_time)) {
		elapsed_time = timer.get_elapsed_time();
		auto targets = profile.calculate(90, elapsed_time.in(seconds));
		//auto voltage = ff.calculate(radians_per_second(targets.velocity), radians_per_second_squared(targets.acceleration));
		
		auto displacement = robot.imu.get_rotation();
		auto velocity = (displacement - last_displacement) / seconds(0.02);
		
		if (elapsed_time < seconds(timing.inflection_time)) {
			robot.chassis.turn_voltage(volts(10));
		}
		
		std::cout 
		<< "elapsed time: " << elapsed_time 
		<< ", measured velocity: " << velocity
		<< ", measured displacement: " << displacement
		<< std::endl;

		last_displacement = robot.imu.get_rotation();
		pros::delay(20);
	}
}

// Add selected autons in here.
// Prints the completion time of autons.
void autonomous() {
	//console.focus();

	auto start_time = pros::millis();
	
	robot.chassis.left_motors.raw.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	robot.chassis.right_motors.raw.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	//test_exp_turn();

	robot.turn_with_pid(90);

	/*robot.chassis.left_motors.move(50);
	pros::delay(500);
	robot.chassis.left_motors.move(0);
	robot.chassis.right_motors.move(50);
	pros::delay(500);
	robot.chassis.right_motors.move(0);*/
	//do_intake = true;
	//robot.ramseteTest({inches(14.89),inches(60)},true,4,0,30);
	
	
	/*double last_rotation;
	
	double ks = 1.5779;
	double kv = 0.93301;
	double ka = 0.239860827437;

	double A = 1 / ka;
	double k = -kv / ka;
	double u = 10;
	
	auto profile = ExponentialProfile(A, k, u);
	auto ff = dlib::Feedforward<Degrees>({ks, kv, ka});

	while(true){
		double current_time = pros::millis();
		double elapsed_time = current_time - start_time;
		double current_rotation = robot.imu.get_rotation().in(degrees);
	robot.chassis.turn_voltage(volts(10));
		std::cout << "(" << (current_rotation - last_rotation) * 5 << "," << elapsed_time / 1000 << "),"<<std::endl;

		last_rotation = current_rotation;
		
		pros::delay(200);
	}*/
	/*robot.chassis.turn_voltage(volts(-12));
	pros::delay(500);
	std::cout << robot.imu.get_rotation();
	robot.chassis.turn_voltage(volts(12));
	pros::delay(35);
	robot.chassis.brake();*/


	//robot.turn_ffwd(500);

	auto elapsed_time = pros::millis() - start_time;

	//robot.turn_with_pid(radians(3.14159));

	
	std::cout << "settled in " << elapsed_time << " milliseconds." << "\n";
	dlib::Pose2d position = robot.odom.get_position();
	std::cout << position.x.in(inches) << ", " << position.y.in(inches) << ", " << position.theta.in(degrees) << "\n";
}	

bool driver_lift_toggle = true;

void opcontrol() {
	pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
	
	//robot.chassis.left_motors.raw.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	//robot.chassis.right_motors.raw.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	//intake.set_alliance(Alliance::Blue);
	pros::delay(500);
	driver_macro();
	do_intake = true;
	robot.move_to_point(-30.3,17.6,1.6);
	robot.move_to_point(-56.9,24.7,1.6);
	lift.set_state(LiftState::Prime);
	robot.move_to_point(-83.89,42.64,1.6);
	pros::delay(500);
	do_intake = false;
	intake.stop();
	robot.move_to_point(-60.21,24.16,1,true);
	lift.set_state(LiftState::Up);
	pros::delay(300);
	do_intake = true;
	robot.move_to_point(-75.27,11.7,.7);
	//lift.set_state(LiftState::Score);
	pros::delay(700);
	lift.set_state(LiftState::Prime);
	do_intake = false;
	intake.rev();
	pros::delay(500);
	do_intake = false;
	intake.stop();
	lift.set_state(LiftState::Idle);
	pros::delay(300);
	do_intake = true;
	robot.move_to_point(-60.21,24.16,1.6,true);
	robot.move_to_point(-25.07,-23.7,.7);
	pros::delay(500);
	robot.chassis.left_motors.move_voltage(volts(-10));
	pros::delay(500);
	brake();
	robot.move_to_point(-36.89,-20.125,1.6);
	pros::delay(500);
	robot.turn_to_point(-33.25, -28.6,true);
	mogo.unclamp();
	robot.move_to_point(-33.25,-33.6,.8,true);
	do_intake = false;
	intake.stop();

	// part 2
	robot.chassis.left_motors.move_voltage(volts(8));
	pros::delay(500);
	brake();
	robot.move_to_point(-0.94,12,1.6,true);
	robot.turn_to_point(28.2,32.93,true);
	auto_clamp = true;

	while(auto_clamp){
		pros::delay(20);
	}
	do_intake = true;
	robot.move_to_point(6.89,41.37,1.6);
	robot.move_to_point(9.32,67.79,1.6);
	lift.set_state(LiftState::Prime);
	robot.move_to_point(-5.79,95.21,1.6);
	robot.move_to_point(4.5,75.95,1,true);
	do_intake = false;
	intake.stop();
	lift.set_state(LiftState::Up);
	pros::delay(250);
	do_intake = true;
	robot.move_to_point(19.34,89.2,.8);
	lift.set_state(LiftState::Score);
	pros::delay(700);
	lift.set_state(LiftState::Prime);
	do_intake = false;
	intake.stop();
	intake.rev();
	pros::delay(500);
	lift.set_state(LiftState::Idle);
	do_intake = true;
	robot.move_to_point(6.7,77,1.6,true);
	robot.move_to_point(45.70,33.49,0.7);
	pros::delay(500);
	robot.chassis.right_motors.move_voltage(volts(-10));
	pros::delay(500);
	brake();
	robot.move_to_point(45.33,45.36,1.6);
	pros::delay(500);
	robot.turn_to_point(49.68,46.39,true);
	mogo.unclamp();
	robot.move_to_point(54.88,43.28,0.7,true);
	do_intake = false;
	intake.stop();

	std::cout << "time: (inaccurate): " << pros::millis();


	//driver_macro();
	auton = false;
	double voltage = 0;
	//driver_lift_toggle = !driver_lift_toggle;
	while(true){
		// ------------------- //
		// Chassis Controls
		// ------------------- //
		// get power and turn
		double power = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X);
		//std::cout << power << std::endl;
		if(!auto_clamp){
			robot.chassis.arcade(power,turn);
		}
		// ------------------- //
		// Intake Controls
		// ------------------- //

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			do_intake = true;
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
			do_intake = false;
			intake.rev();
		}
		else{
			do_intake = false;
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
		
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			lift.set_state(LiftState::Score);
		}
		else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			if(lift.lift_state == LiftState::Prime){
				lift.set_state(LiftState::Up);
			}
			else{
				lift.set_state(LiftState::Prime);
			}
		}
		else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
			if(lift.lift_state == LiftState::Prime){
				lift.set_state(LiftState::Idle);
			}
			else if(lift.lift_state == LiftState::Idle){
				lift.set_state(LiftState::Prime);
			}
			else{
				lift.set_state(LiftState::Prime);
			}
		}
		else{
			if(driver_lift_toggle){
				if(lift.lift_state == LiftState::Prime){
					lift.set_state(LiftState::Prime);
				}
				else if(lift.lift_state == LiftState::Up){
					lift.set_state(LiftState::Up);
				}
				else{
					lift.set_state(LiftState::Idle);
				}
			}
			else{
				if(lift.lift_state == LiftState::Idle){
					lift.set_state(LiftState::Idle);
				}
				else if(lift.lift_state == LiftState::Up){
					lift.set_state(LiftState::Up);
				}
				else{
					lift.set_state(LiftState::Prime);
				}
			}
			
		}

		// ------------------- //
		// Doinker Controls
		// ------------------- //
		// 
		if(master.get_digital_new_press(DIGITAL_RIGHT)){
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

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
			driver_lift_toggle = !driver_lift_toggle;
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			auto_clamp = !auto_clamp;
		}

		
		/*if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
			voltage += 0.05;
			robot.chassis.turn_voltage(volts(voltage));
			std::cout << voltage << std::endl;
		}*/


		pros::delay(20);
	}
	
	
}
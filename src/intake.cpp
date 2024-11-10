#include "api.h"
#include "pros/distance.hpp"
#include <cstdint>
#include "intake.hpp"

// Initializer class for intake
Intake::Intake(
    int8_t intake_port,
    int8_t color_sensor_port,
    int8_t distance_sensor_port
) :
intake_motor(intake_port),
color_sensor(color_sensor_port),
distance_sensor(distance_sensor_port){
intake_motor.set_gearing(pros::E_MOTOR_GEAR_BLUE);
};

// Move intake forward at x volts
void Intake::intake(int8_t volts){
    intake_motor.move(volts);
}

// Move intake in reverse at x volts
void Intake::outtake(int8_t volts){
    intake_motor.move(-volts);
}

// Move intake forward
void Intake::max_intake(){
    intake_motor.move(127);
}

// Move intake in reverse
void Intake::rev_intake(){
    intake_motor.move(-127);
}

// Stop intake
void Intake::stop_intake(){
    intake_motor.brake();
}

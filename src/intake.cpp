#include "api.h"
#include "pros/distance.hpp"
#include <cstdint>
#include <memory>
#include <mutex>
#include "intake.hpp"


// Initializer class for intake
Intake::Intake(
    int8_t intake_port,
    int8_t color_sensor_port,
    int8_t distance_sensor_port,
    Alliance2 alliance, // false = blue, true = red
    bool redirect,
    bool auton_stick,
    bool auton
) :
    intake_motor(intake_port),
    color_sensor(color_sensor_port),
    distance_sensor(distance_sensor_port),
    alliance(alliance),
    auton_stick(auton_stick),
    auton(auton)
    {
    intake_motor.set_gearing(pros::E_MOTOR_GEAR_BLUE);
};

void Intake::set_alliance(Alliance2 new_alliance){
    std::lock_guard<pros::Mutex> guard(intake_mutex);
    alliance = new_alliance;
}

Alliance2 Intake::get_alliance(){
    return alliance;
}

// Move intake forward at x volts
void Intake::move(int8_t volts){
    std::lock_guard<pros::Mutex> guard(intake_mutex);
    intake_motor.move(volts);
}

// Move intake forward
void Intake::max(){
    std::lock_guard<pros::Mutex> guard(intake_mutex);
    intake_motor.move(127);
}

// Move intake in reverse
void Intake::rev(){
    std::lock_guard<pros::Mutex> guard(intake_mutex);
    intake_motor.move(-127);
}

// Stop intake
void Intake::stop(){
    std::lock_guard<pros::Mutex> guard(intake_mutex);
    intake_motor.brake();
}
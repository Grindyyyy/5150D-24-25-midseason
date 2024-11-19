#include "api.h"
#include "pros/distance.hpp"
#include <cstdint>
#include "intake.hpp"


// Initializer class for intake
Intake::Intake(
    int8_t intake_port,
    int8_t color_sensor_port,
    int8_t distance_sensor_port,
    Alliance alliance, // false = blue, true = red
    bool redirect
) :
    intake_motor(intake_port),
    color_sensor(color_sensor_port),
    distance_sensor(distance_sensor_port),
    alliance(alliance) {
    intake_motor.set_gearing(pros::E_MOTOR_GEAR_BLUE);
};

void Intake::set_alliance(Alliance new_alliance){
    alliance = new_alliance;
}

void Intake::set_redirect(bool new_redirect){
    redirect = new_redirect;
}

// Move intake forward at x volts
void Intake::move(int8_t volts){
    intake_motor.move(volts);
}

// Move intake forward
void Intake::max(){
    intake_motor.move(127);
}

// Move intake in reverse
void Intake::rev(){
    intake_motor.move(-127);
}

// Stop intake
void Intake::stop(){
    intake_motor.brake();
}

double Intake::get_red(){
    pros::c::optical_rgb_s_t rgb = color_sensor.get_rgb();
    return(rgb.red);
}
double Intake::get_green(){
    pros::c::optical_rgb_s_t rgb = color_sensor.get_rgb();
    return(rgb.green);
}
double Intake::get_blue(){
    pros::c::optical_rgb_s_t rgb = color_sensor.get_rgb();
    return(rgb.blue);
}

bool Intake::intake_filter(){
    if(alliance == Alliance::Blue){
        if((get_red() > get_blue() * 1.5) || (redirect && get_blue() > get_red() * 1.5)) {
            return true;
        }
        else{
            return false;
        }
    }
    if(alliance == Alliance::Red){
        if((get_blue() > get_red() * 1.5) || (redirect && get_red() > get_blue() * 1.5)) {
            return true;
        }
        else{
            return false;
        }
    }
}
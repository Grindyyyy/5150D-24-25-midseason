#include "api.h"
#include "lift.hpp"

// Initializer class for intake
Lift::Lift(
    int8_t lift_port,
    int8_t lift_rot_port
) :
lift_motor(lift_port), lift_rot(lift_rot_port){
lift_motor.set_gearing(pros::E_MOTOR_GEAR_RED);
};

// Move intake forward at x volts
void Lift::lift_move(double volts){
    lift_motor.move(volts);
}

void Lift::lift_range(double range){
    if(lift_toggle){
        if(lift_rot.get_position() > range){
            lift_motor.move(90);
        }
        else{
            lift_motor.move(30);
        }
    }
    else if(!lift_toggle){
        if(lift_rot.get_position() < range){
            lift_motor.move(-90);
        }
        else{
            lift_motor.brake();
        }
    } 
}

double Lift::get_lift_range(){
    return lift_rot.get_position();
}
// Move intake forward
void Lift::lift_stop(){
    lift_motor.move(0);
}

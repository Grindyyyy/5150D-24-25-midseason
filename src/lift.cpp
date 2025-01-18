#include "api.h"
#include "lift.hpp"

// Initializer class for intake
Lift::Lift(
    int8_t lift_port,
    int8_t lift_rot_port,
    char piston_port,
    LiftState lift_state
) :
lift_motor(lift_port), lift_rot(lift_rot_port), piston(piston_port), lift_state(lift_state){
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

void Lift::move_voltage(double millivolts){
    lift_motor.move_voltage(millivolts);
}

void Lift::set_state(LiftState state){
    lift_state = state;
    if(state == LiftState::Up){
        target_position = 56000;
    }
    else if(state == LiftState::Touch){
        target_position = 43000;
    }
    else{
        target_position = 0;
    }
}

double Lift::get_lift_range(){
    return lift_rot.get_position();
}
// Move intake forward
void Lift::lift_stop(){
    lift_motor.move(0);
}

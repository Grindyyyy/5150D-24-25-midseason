#include "api.h"
#include "main.h"
#include "lift.hpp"
#include <iostream>
#include <memory>

// lift.cpp

// Initializer class for intake
Lift::Lift(
    int8_t lift_port_1,
    int8_t lift_port_2,
    int8_t lift_rot_port,
    LiftState initial
) :
lift_motor_1(lift_port_1), lift_motor_2(lift_port_2), lift_rot(lift_rot_port){
    lift_rot.set_position(0);
};


// Move intake forward at x volts
void Lift::lift_move(double volts){
    lift_motor_1.move(volts);
    lift_motor_2.move(volts);
}

void Lift::move_voltage(double millivolts){
    lift_motor_1.move_voltage(millivolts);
    lift_motor_2.move_voltage(millivolts);
}

void Lift::set_state(LiftState state){
    lift_state = state;
    if(state == LiftState::Score){
        target_position = 24000;
    }
    else if(state == LiftState::Up){
        target_position = 6500;
    }
    else if(state == LiftState::Stake){
        target_position = 19000;
    }
    else if(state == LiftState::Prime){
        target_position = 1450;
    }
    else{
        target_position = 0;
    }
}

void Lift::toggle_state(){
    if(lift_state == LiftState::Prime){
        lift_state = LiftState::Idle;
    }
    else if(lift_state == LiftState::Idle){ 
        lift_state = LiftState::Prime;
    }
}

double Lift::get_lift_position(){
    return lift_rot.get_position();
}
// Move intake forward
void Lift::lift_stop(){
    lift_motor_1.move(0);
    lift_motor_2.move(0);
}

#include "api.h"
#include "pros/colors.hpp"
#include "pros/motors.h"
#include "intake.hpp"

// intake.cpp

bool detected = false;

Intake::Intake(
    int8_t intake_motor_port,
    int8_t color_sorter_port
) : intake_motor(intake_motor_port), color_sorter(color_sorter_port)
{
    intake_motor.set_gearing(pros::E_MOTOR_GEAR_GREEN);
    color_sorter.set_led_pwm(50);
    auto current_position = intake_motor.get_position();
    auto beginning_position = intake_motor.get_position();
};

void Intake::set_alliance(Alliance alliance){
    current_alliance = alliance;
}

void Intake::set_redirect(bool redirect){
    do_redirect = redirect;
}

void Intake::auto_max(){
    auto color = color_sorter.get_rgb();
    if(((current_alliance == Alliance::Red || (current_alliance == Alliance::Blue && do_redirect)) && color.blue > color.red * 1.5) && current_state != States::After){
        if(!detected){
            detected = true;
            beginning_position = intake_motor.get_position();
            current_state = States::Before;
        }
        current_state = States::Before;
        current_position = intake_motor.get_position();
    }
    else if(((current_alliance == Alliance::Blue || (current_alliance == Alliance::Red && do_redirect)) && color.red > color.blue * 2.5) && current_state != States::After){
        if(!detected){
            detected = true;
            beginning_position = intake_motor.get_position();
            current_state = States::Before;
        }
        current_state = States::Before;
        current_position = intake_motor.get_position();
    }
    else{
        detected = false;
    }
    
    if(!detected && current_state == States::None){
        intake_motor.move_voltage(15000);
    }

    if(current_state == States::Before && current_position - beginning_position > 0){
        current_state = States::After;
        current_position = intake_motor.get_position();
        beginning_position = intake_motor.get_position();
    }
    else if(current_state == States::Before && current_position - beginning_position < 400){
        intake_motor.move_voltage(15000);
    }

    if(current_state == States::After && std::abs(current_position - beginning_position) < 400){
        intake_motor.move_voltage(-15000);
        current_position = intake_motor.get_position();
    }
    else if(current_state == States::After && std::abs(current_position - beginning_position) >= 400){
        current_state = States::None;
    }
    //intake_motor.move_voltage(12000);
}

void Intake::max(){
    intake_motor.move_voltage(12000);
}

void Intake::move_voltage(double volts){
    intake_motor.move_voltage(volts);
}

void Intake::rev(){
    intake_motor.move_voltage(-8000);
}   

void Intake::stop(){
    intake_motor.move_voltage(0);
}
//meow
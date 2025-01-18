#pragma once
#include "api.h"

enum class Alliance {
    Red,
    Blue
};

enum class States {
    None,
    Before,
    After
};

class Intake{
public:
    pros::Motor intake_motor;
    pros::Optical color_sorter;
    Alliance current_alliance;
    States current_state;
    bool do_redirect = false;
    double current_position;
    double beginning_position;

    Intake(
        int8_t intake_motor_port,
        int8_t color_sorter_port
    );

    void set_alliance(Alliance alliance);

    void set_redirect(bool redirect);

    void max();

    void rev();

    void stop();
};
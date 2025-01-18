#pragma once
#include "api.h"
#include "pros/adi.hpp"

enum class LiftState{
    Idle,
    Touch,
    Up
};

class Lift{
    public:
        pros::Motor lift_motor;
        pros::Rotation lift_rot;
        pros::adi::DigitalOut piston;
        bool lift_toggle = false;
        LiftState lift_state;
        double target_position;

        Lift(
            int8_t lift_port,
            int8_t lift_rot_port,
            char piston_port,
            LiftState lift_state = LiftState::Up
        );

        void lift_move(double volts);

        void move_voltage(double millivolts);

        void lift_range(double range);

        void set_state(LiftState state);

        double get_lift_range();

        void lift_stop();
};
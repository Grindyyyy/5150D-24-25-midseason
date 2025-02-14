#pragma once
#include "api.h"
#include "pros/adi.hpp"

// lift.hpp

enum class LiftState{
    Score,
    Prime,
    Idle,
    Up,
    Stake
};

class Lift{
    public:
        pros::Motor lift_motor_1;
        pros::Motor lift_motor_2;
        pros::Rotation lift_rot;
        bool lift_toggle = false;
        LiftState lift_state;
        double target_position;

        Lift(
            int8_t lift_port_1,
            int8_t lift_port_2,
            int8_t lift_rot_port,
            LiftState lift_state = LiftState::Idle
        );

        void lift_move(double volts);

        void move_voltage(double millivolts);

        void lift_range(double range);

        void set_state(LiftState state);

        void toggle_state();

        double get_lift_position();

        void lift_stop();
};
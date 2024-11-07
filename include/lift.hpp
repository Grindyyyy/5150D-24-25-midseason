#pragma once
#include "api.h"

class Lift{
    public:
        pros::Motor lift_motor;
        pros::Rotation lift_rot;
        bool lift_toggle = false;
        int last_range = 0;

        Lift(
            int8_t lift_port,
            int8_t lift_rot_port
        );

        void lift_move(double volts);

        void lift_range(double range);

        double get_lift_range();

        void lift_stop();
};
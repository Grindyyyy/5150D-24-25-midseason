#pragma once
#include "api.h"

class Mogo{
    public:
        pros::adi::DigitalOut mogo;

        Mogo(
            char digital_port,
            bool init_state
        );

        void set_clamp_state(bool state);
};
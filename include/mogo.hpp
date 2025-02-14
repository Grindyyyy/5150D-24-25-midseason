#pragma once
#include "api.h"

// mogo.hpp

class Mogo{
    public:
        pros::adi::DigitalOut mogo;
        bool mogo_state;
        
        Mogo(
            char digital_port,
            bool init_state,
            bool mogo_state = false
        );

        void set_clamp_state(bool state);

        void clamp();

        void unclamp();

        void reverse_clamp_state();
};
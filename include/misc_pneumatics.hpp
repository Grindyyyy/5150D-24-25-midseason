#pragma once
#include "api.h"

// misc_pneumatics.hpp

class MiscPneumatics{
    public:
        pros::adi::DigitalOut indexer;
        pros::adi::DigitalOut doinker;
        pros::adi::DigitalOut arm_piston;

        bool doinker_toggle;
        bool indexer_toggle;
        bool arm_piston_toggle;


        MiscPneumatics(
            char indexer_port,
            bool indexer_init_state,
            char doinker_port,
            bool doinker_init_state,
            char arm_piston_port,
            bool arm_piston_init_state
        );

        void set_indexer_state(bool state);
        void set_doinker_state(bool state);
        void set_arm_state(bool state);

};
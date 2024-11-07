#pragma once
#include "api.h"

class MiscPneumatics{
    public:
        pros::adi::DigitalOut indexer;
        pros::adi::DigitalOut doinker;

        MiscPneumatics(
            char indexer_port,
            bool indexer_init_state,
            char doinker_port,
            bool doinker_init_state
        );

        void set_indexer_state(bool state);
        void set_doinker_state(bool state);

};
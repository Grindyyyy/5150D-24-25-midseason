#include "misc_pneumatics.hpp"

// misc_pneumatics.cpp

// Initializer class for Clamp
MiscPneumatics::MiscPneumatics(
    char indexer_port,
    bool indexer_init_state,
    char doinker_port,
    bool doinker_init_state,
    char arm_piston_port,
    bool arm_piston_init_state
) :
indexer(indexer_port, false), doinker(doinker_port, doinker_init_state), arm_piston(arm_piston_port, arm_piston_init_state){};

void MiscPneumatics::set_indexer_state(bool state){
    indexer.set_value(state);
}

void MiscPneumatics::set_doinker_state(bool state){
    doinker.set_value(state);
}

void MiscPneumatics::set_arm_state(bool state){
    arm_piston.set_value(state);
}
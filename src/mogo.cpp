#include "mogo.hpp"

// mogo.cpp

// Initializer class for Clamp
Mogo::Mogo(
    char digital_port,
    bool init_state,
    bool mogo_state
) :
mogo(digital_port, init_state){};

void Mogo::set_clamp_state(bool state){
    mogo_state = state;
    mogo.set_value(state);
}

void Mogo::clamp(){
    mogo_state = true;
    mogo.set_value(true);
}

void Mogo::unclamp(){
    mogo_state = false;
    mogo.set_value(false);
}

void Mogo::reverse_clamp_state(){
    mogo_state = !mogo_state;
    mogo.set_value(mogo_state);
}
#include "mogo.hpp"

// Initializer class for Clamp
Mogo::Mogo(
    char digital_port,
    bool init_state,
    bool mogo_state
) :
mogo(digital_port, init_state){};

void Mogo::set_clamp_state(bool state){
    mogo.set_value(state);
}

void Mogo::reverse_clamp_state(){
    mogo_state = !mogo_state;
    mogo.set_value(mogo_state);
}
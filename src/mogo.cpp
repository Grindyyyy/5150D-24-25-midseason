#include "mogo.hpp"

// Initializer class for Clamp
Mogo::Mogo(
    char digital_port,
    bool init_state
) :
mogo(digital_port, init_state){};

void Mogo::set_clamp_state(bool state){
    mogo.set_value(state);
}
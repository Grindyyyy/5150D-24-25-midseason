#include "misc_pneumatics.hpp"

// Initializer class for Clamp
MiscPneumatics::MiscPneumatics(
    char indexer_port,
    bool indexer_init_state,
    char doinker_port,
    bool doinker_init_state
) :
indexer(indexer_port, indexer_init_state), doinker(doinker_port, doinker_init_state){};

void MiscPneumatics::set_indexer_state(bool state){
    indexer.set_value(state);
}

void MiscPneumatics::set_doinker_state(bool state){
    indexer.set_value(state);
}
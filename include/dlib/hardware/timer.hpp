#pragma once
#include "au/au.hpp"

namespace dlib {

class Timer {
public:
    Timer();
    
    void reset();

    au::Quantity<au::Seconds, double> get_start_time();
    au::Quantity<au::Seconds, double> get_current_time();
    au::Quantity<au::Seconds, double> get_elapsed_time();

protected:
    au::Quantity<au::Seconds, double> start_time;
};


}
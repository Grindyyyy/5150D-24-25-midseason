#pragma once
#include "au/au.hpp"

namespace dlib {

// timer.hpp

class Timer {
public:
    Timer();
    
    /**
     * @brief Reset the Timer
     * 
     */
    void reset();

    /**
     * @brief Get the start time of the Timer
     * 
     * @return start time
     */
    au::Quantity<au::Seconds, double> get_start_time();

    /**
     * @brief Get the current time
     * 
     * @return the current time
     */
    au::Quantity<au::Seconds, double> get_current_time();

    /**
     * @brief Get the elapsed time since the timer started
     * 
     * @return elapsed time
     */
    au::Quantity<au::Seconds, double> get_elapsed_time();

protected:
    au::Quantity<au::Seconds, double> start_time;
};


}
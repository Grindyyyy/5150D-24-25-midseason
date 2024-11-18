#include "dlib/hardware/timer.hpp"
#include "pros/rtos.hpp"

namespace dlib {

Timer::Timer() {
    this->start_time = au::milli(au::seconds)(pros::millis());
}

void Timer::reset() {
    this->start_time = au::milli(au::seconds)(pros::millis());
}

au::Quantity<au::Seconds, double> Timer::get_start_time() {
    return this->start_time;
}

au::Quantity<au::Seconds, double> Timer::get_current_time() {
    return au::milli(au::seconds)(pros::millis());
}

au::Quantity<au::Seconds, double> Timer::get_elapsed_time() {
    return get_current_time() - get_start_time();
}

}
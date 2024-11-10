#pragma once
#include "au/au.hpp"

namespace dlib {

template<typename Units>
class ErrorTimeSettler {
public:
    ErrorTimeSettler(
        au::Quantity<Units, double> error_threshold,    // the maximum error at which we can settle
        au::Quantity<au::Seconds, double> settle_time   // the minimum time below the error threshold to settle
    ) :
        error_threshold(error_threshold),
        settle_time(settle_time) {

    }

    /**
     * @brief Check if controller has settled
     *
     * @param error distance from setpoint
     * @param period time since last settle iteration - current time - last time
     * @return if the controller has settled
     * 
     * @b Example
     * @code {.cpp}
     * 
     * dlib::Pid<Meters> pid({1, 2, 3});

	 * // Construct a last_period period
	 * Quantity<Seconds, double> last_period;

     * // Construct an TimeErrorSettler controller
     * dlib::TimeErrorSettler<Meters> pid_settler {
     *    meters(0.01),	    // the maximum error the pid can settle at
     *    seconds(0.250)    // the time the Pid needs to remain below the error threshold
     * }
     * 
     * bool settled = pid_settler.is_settled(pid.get_error(), seconds(0.02));
     * 
     * @endcode
    */
    bool is_settled(au::Quantity<Units, double> error, au::Quantity<au::Seconds, double> period) {
        auto absolute_error = au::abs(error);

        if (this->is_settling) {
            // if we are settling, and we are still below the error add the period to the elapsed time 
            if (absolute_error <= this->error_threshold) {
                this->time_below_threshold += period;
            } else {
                // we went outside the error threshold; stop settling
                this->is_settling = false;
                this->time_below_threshold = au::ZERO;
            }
        }

        // if we aren't settling yet, and we are below the error threshold, start settling
        if (!this->is_settling && absolute_error <= this->error_threshold) {
            this->is_settling = true;
        }

        // we are settled iff:
        // - we have started settling
        // - we are below the error threshold
        // - we have been below the threshold for the settle time
        if (this->is_settling && this->time_below_threshold >= this->settle_time && absolute_error <= this->error_threshold) {
            this->is_settling = false;
            this->time_below_threshold = au::seconds(0);
            return true;
        } else {
            return false;
        }
    }

protected:
    bool is_settling;
    au::Quantity<Units, double> error_threshold;
    au::Quantity<au::Seconds, double> settle_time;
    au::Quantity<au::Seconds, double> time_below_threshold;
};

}
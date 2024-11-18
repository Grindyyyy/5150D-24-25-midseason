#pragma once
#include "au/au.hpp"

namespace dlib {

// Units is templated so that the user can config what units they want at object creation time
template<typename Units>
class ErrorDerivativeSettler {
public:
    // Constructor takes in error_threshold & derivative_threshold
    // Assigns it to error_threshold & derivative_threshold in object variables
    ErrorDerivativeSettler(
        au::Quantity<Units, double> error_threshold,
        au::Quantity<au::TimeDerivative<Units>, double> derivative_threshold
    ) : 
        error_threshold(error_threshold), 
        derivative_threshold(derivative_threshold)  {
    }
    /**
     * @brief Check if controller has settled
     *
     * @param error distance from setpoint
     * @param derivative velocity
     * @return if the controller has settled
     * 
     * @b Example
     * @code {.cpp}
     * 
     * // Construct a PID controller
     * dlib::Pid<Meters> pid({1, 2, 3});
     *
     * // Construct an ErrorDerivativeSettler controller
     * dlib::ErrorDerivativeSettler<Meters> pid_settler {
	 *     meters(0.01),		    // error threshold, the maximum error the pid can settle at
	 *     meters_per_second(0.01)  // derivative threshold, the maximum instantaneous error over time the pid can settle at
     * }
     *
     * bool settled = pid_settler.is_settled(pid.get_error(), pid.get_derivative());
     * 
     * @endcode
    */
    bool is_settled(au::Quantity<Units, double> error, au::Quantity<au::TimeDerivative<Units>, double> derivative) {
        
        // if the error and derivative are both exactly zero it is almost certain the pid has not been called yet
        if (error == au::ZERO && derivative == au::ZERO) {
            return false;
        }

        // user passes in error & derivative from the pid read methods
        // if it falls within the range given when the ErrorDerivativeSettler object is created
        // it will return true
        return au::abs(error) <= error_threshold && au::abs(derivative) <= derivative_threshold;
    }

    void reset() {
        
    }
protected:
    au::Quantity<Units, double> error_threshold; // The error range that is acceptable
    au::Quantity<au::TimeDerivative<Units>, double> derivative_threshold; // The derivative that is acceptable (speed)
};


}
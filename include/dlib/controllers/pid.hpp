#pragma once
#include "au/au.hpp"
#include "pros/rtos.hpp"

namespace dlib {

/**
 * @brief The gains for a standard Pid controller
 * 
 */
struct PidGains {
    /** The porpotional gain: how aggresively the controller responds to error */
    double kp = 0;
    /** The integral gain: how much to increase the response if error persists too long */
    double ki = 0;
    /** The derivative gain: used to limit the speed of the controllers response */
    double kd = 0;
};

template<typename Units>
class Pid {
public:
    Pid(PidGains gains) : gains(gains) {};

    /**
     * @brief Set Pid setpoint
     *
     * @param setpoint the end goal for the controller
     * 
     * @b Example
     * @code {.cpp}
     * 
     * // Construct a Pid controller
     * dlib::Pid<Meters> pid({1, 2, 3});
     * 
     * // Target an example setpoint
     * move_pid.target(meters(1));
     * 
     * @endcode
    */
    void target(au::Quantity<Units, double> setpoint) {
        this->setpoint = setpoint;
        
        this->p = au::ZERO;
        this->i = au::ZERO;
        this->d = au::ZERO;

        this->last_error = au::ZERO;
    }

    /**
     * @brief Calculate Pid voltage
     *
     * @param reading the current sensor reading
     * @param period the interval at which the pid is updated
     * @return the voltage to send to your mechanism
     * 
     * @b Example
     * @code {.cpp}
     * 
     * // Construct a Pid controller
     * dlib::Pid<Meters> pid({1, 2, 3});
     * 
     * // Calculate voltage with a test input
     * Quantity<Volts, double> voltage = pid.update(meters(0.05), seconds(0.02));
     * 
     * @endcode
    */
    au::Quantity<au::Volts, double> update(
        au::Quantity<Units, double> reading, 
        au::Quantity<au::Seconds, double> period
    ) {
        auto error = setpoint - reading;

        auto delta_time = period;
        auto delta_error = error - last_error;

        // TODO: Integral anti-windup
        // calculate Pid terms

        this->p = error;
        this->i = this->i + error* delta_time;
        this->d = (delta_error / delta_time);

        // TODO: Using `in` this way means that Pid constants are affected by the choice of unit
        // eg. gains in Pid<Centi<Meters>> will effectively be 100x the gains in Pid<Meters>
        // is there a way to avoid this?

        double output = std::clamp(
            this->p.in(Units{}) * this->gains.kp
            + this->i.in(au::TimeIntegral<Units>{}) * this->gains.ki 
            + this->d.in(au::TimeDerivative<Units>{}) * this->gains.kd, 
            -12.0, 12.0
        );

        // update Pid state
        this->last_error    = error;

        return au::volts(output);
    };

    /**
     * @brief Get Pid gains
     *
     * @return The current Pid gains
     * 
     * @b Example
     * @code {.cpp}
     * 
     * // Construct a Pid controller
     * dlib::Pid<Meters> pid({1, 2, 3});
     * 
     * // Get the Pid gains
     * dlib::PidGains gains = pid.get_gains();
     * 
     * @endcode
    */
    PidGains get_gains() {
        return this->gains;
    }

    /**
     * @brief Set Pid gains
     *
     * @param gains gains to set the Pid constructor to
     * 
     * @b Example
     * @code {.cpp}
     * 
     * // Construct a Pid controller
     * dlib::Pid<Meters> pid({1, 2, 3});
     * 
     * // Set the Pid gains
     * pid.set_gains({4, 5, 6});
     * 
     * @endcode
    */
    void set_gains(PidGains gains) {
        this->gains = gains;
    }

    /**
     * @brief Get Pid error
     *
     * @return The current error (how far the controller is from the setpoint)
     * 
     * @b Example
     * @code {.cpp}
     * v
     * // Construct a Pid controller
     * dlib::Pid<Meters> pid({1, 2, 3});
     * 
     * // Get Pid error
     * Quantity<Meters, double> error = pid.get_error();
     * 
     * @endcode
    */
    au::Quantity<Units, double> get_error() {
        return this->last_error;
    }

    /**
     * @brief Get Pid derivative
     *
     * @return The current derivative (how quickly the error is changing)
     * 
     * @b Example
     * @code {.cpp}
     *
     * // Construct a Pid controller
     * dlib::Pid<Meters> pid({1, 2, 3});
     * 
     * // Get Pid derivative
     * Quantity<TimeDerivative<Meters>, double> derivative = pid.get_derivative();
     * 
     * @endcode
    */
    au::Quantity<au::TimeDerivative<Units>, double> get_derivative() {
        return this->d;
    }
protected:
    PidGains gains;

    au::Quantity<Units, double> p = au::ZERO;
    au::Quantity<au::TimeIntegral<Units>, double> i = au::ZERO;
    au::Quantity<au::TimeDerivative<Units>, double> d = au::ZERO;

    au::Quantity<Units, double> setpoint;
    au::Quantity<Units, double> last_error;
};

}
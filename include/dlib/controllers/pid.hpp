#pragma once
#include "au/au.hpp"
#include "dlib/controllers/pid.hpp"
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

struct PidConfig {
    PidGains gains{};
    au::Quantity<au::Volts, double> max_voltage = au::volts(12);
};

namespace detail {
    template<typename Units>
    struct PidGainsWithUnits {
        au::Quantity<decltype(au::Volts{} / Units{}), double> kp;
        au::Quantity<decltype(au::Volts{} / au::TimeIntegral<Units>{}), double> ki;
        au::Quantity<decltype(au::Volts{} / au::TimeDerivative<Units>{}), double> kd;

        PidGainsWithUnits(
            PidGains gains
        ) : 
            kp(au::make_quantity<decltype(au::Volts{} / Units{})>(gains.kp)), 
            ki(au::make_quantity<decltype(au::Volts{} / au::TimeIntegral<Units>{})>(gains.ki)), 
            kd(au::make_quantity<decltype(au::Volts{} / au::TimeDerivative<Units>{})>(gains.kd)) {

        }
    };
}

template<typename Units>
class Pid {
public:
    Pid(PidConfig config) : gains(config.gains), max_voltage(config.max_voltage) {};

    /**
     * @brief Reset all of the Pid state
     * 
     */
    void reset() {
        this->p = au::ZERO;
        this->i = au::ZERO;
        this->d = au::ZERO;

        this->last_error = au::ZERO;
        this->last_derivative = au::ZERO;
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
        au::Quantity<Units, double> error, 
        au::Quantity<au::Seconds, double> period
    ) {
        auto delta_time = period;
        auto delta_error = error - last_error;

        auto derivative = (delta_error / delta_time);
        
        // calculate Pid terms

        // integral reset on sign flip
        if ((error < au::ZERO) != (last_error < au::ZERO)) {
            this->i = au::ZERO;
        }
        
        if (this->i > this->max_voltage) {
            this->i = this->max_voltage;
        } else if (this->i < -this->max_voltage) {
            this->i = -this->max_voltage;
        }
        
        this->p = error * this->gains.kp;
        this->i = this->i + error * delta_time * this->gains.ki;
        this->d = (delta_error / delta_time) * this->gains.kd;

        auto output = std::clamp(
            this->p + this->i + this->d, 
            au::volts(-12.0), au::volts(12.0)
        );

        // update Pid state
        this->last_error      = error;
        this->last_derivative = derivative;
        
        return output;
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
    PidGains get_gains() const {
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
    au::Quantity<Units, double> get_error() const {
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
    au::Quantity<au::TimeDerivative<Units>, double> get_derivative() const {
        return this->last_derivative;
    }

    /**
     * @brief Get the P term of the Pid
     * 
     * @return au::Quantity<Units, double> 
     */
    au::Quantity<Units, double> get_p() const {
        return this->p;
    }

    /**
     * @brief Get the I term of the Pid
     * 
     * @return au::Quantity<au::TimeIntegral<Units>, double> 
     */
    au::Quantity<au::TimeIntegral<Units>, double> get_i() const {
        return this->i;
    }

    /**
     * @brief Get the D term of the Pid
     * 
     * @return au::Quantity<au::TimeDerivative<Units>, double> 
     */
    au::Quantity<au::TimeDerivative<Units>, double> get_d() const {
        return this->d;
    }
protected:
    using BaseUnits = au::UnitImpl<au::detail::DimT<Units>>;
    detail::PidGainsWithUnits<BaseUnits> gains;
    
    au::Quantity<au::Volts, double> max_voltage;

    au::Quantity<au::Volts, double> p = au::ZERO;
    au::Quantity<au::Volts, double> i = au::ZERO;
    au::Quantity<au::Volts, double> d = au::ZERO;

    au::Quantity<Units, double> last_error;
    au::Quantity<au::TimeDerivative<Units>, double> last_derivative;
};

}
#pragma once
#include <iostream>
#include "au/au.hpp"

namespace dlib {

/**
 * @brief The gains for a simple motor feedforward controller
 * 
 */
struct FeedforwardGains {
    /** The the voltage needed to overcome static friction */
    double ks = 0;
    /** The voltage needed to cruise at a constant velocity while overcoming 
    back EMF and any other movement induced friction */
    double kv = 0;
    /** The voltage needed to induce a given acceleration */
    double ka = 0;
};

// TODO: Feedforward should also be generic over units

class Feedforward {
    
public:
    Feedforward(FeedforwardGains gains_settings);

    /**
     * @brief Calculate Feedforward voltage
     *
     * @param target_velocity the target velocity
     * @param target_accleration the target acceleration
     * @return the voltage required for the mechanism to achieve the target velocity and acceleration
     * 
     * @b Example
     * @code {.cpp}
     * 
     * // Construct a Feedforward controller
     * dlib::Feedforward ff = dlib::Feedforward({1, 2, 3});
     * 
     * // Calculate the voltage needed to coast at 1 m/s
     * Quantity<Volts, double> voltage = ff.calculate(meters_per_second(1), ZERO); 
     * 
     * @endcode
    */
    au::Quantity<au::Volts, double> calculate(
        au::Quantity<au::MetersPerSecond, double> target_velocity, 
        au::Quantity<au::MetersPerSecondSquared, double> target_acceleration = au::ZERO
    );

    /**
     * @brief Get Feedforward Gains
     *
     * @return the current gains for the feedforward controller
     * 
     * @b Example
     * @code {.cpp}
     * 
     * // Construct a Feedforward controller
     * dlib::Feedforward ff = dlib::Feedforward({1, 2, 3});
     * 
     * // Get the Feedforward gains
     * dlib::FeedforwardGains gains = ff.get_gains();
     * 
     * @endcode
    */
    FeedforwardGains get_gains();

    /**
     * @brief Set Feedforward gains
     * 
     * @param new_gains the new gains for the feedforward controller
     * 
     * @b Example
     * @code {.cpp}
     * 
     * // Construct a Feedforward controller
     * dlib::Feedforward ff = dlib::Feedforward({1, 2, 3});
     * 
     * // Set the Feedforward gains
     * ff.set_gains({4, 5, 6});
     * 
     * @endcode
    */
    void set_gains(FeedforwardGains new_gains);

protected:
    FeedforwardGains gains;

};
}
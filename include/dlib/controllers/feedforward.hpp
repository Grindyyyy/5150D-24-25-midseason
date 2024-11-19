#pragma once
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

template<typename Units>
class Feedforward {
public:
    Feedforward(FeedforwardGains gains) : gains(gains) {

    }

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
    au::Quantity<au::TimeDerivative<Units>, double> target_velocity, 
    au::Quantity<au::Time2ndDerivative<Units>, double> target_acceleration
    ) {
        using BaseUnits = au::UnitImpl<au::detail::DimT<Units>>;

        auto s = std::copysign(gains.ks, target_velocity.in(au::TimeDerivative<BaseUnits>{}));
        auto v = gains.kv * target_velocity.in(au::TimeDerivative<BaseUnits>{});
        auto a = gains.ka * target_acceleration.in(au::Time2ndDerivative<BaseUnits>{});

        return au::volts(s + v + a);
    }

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
    FeedforwardGains get_gains() {
        return this->gains;
    }

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
    void set_gains(FeedforwardGains new_gains) {
        this->gains = new_gains;
    }

protected:
    FeedforwardGains gains;

};
}
#pragma once
#include <iostream>
#include <cmath>
#include "au/au.hpp"
#include "dlib/trajectories/profile_setpoint.hpp"

namespace dlib {

enum class TrapezoidProfileStage {
    Accelerating,
    Coasting,
    Decelerating,
    Done
};

template<
    typename Units
> requires 
    au::HasSameDimension<Units, au::Meters>::value || 
    au::HasSameDimension<Units, au::Radians>::value
class TrapezoidProfile {
protected:
    const au::Quantity<au::Time2ndDerivative<Units>, double> m_max_acceleration;
    const au::Quantity<au::TimeDerivative<Units>, double> m_max_velocity;
    const au::Quantity<Units, double> m_total_distance;

    au::Quantity<au::Seconds, double> total_time;
    au::Quantity<au::Seconds, double> accel_cutoff;
    au::Quantity<au::Seconds, double> coast_cutoff;
    au::Quantity<au::Seconds, double> decel_cutoff;

    bool invert = false;
public:
    TrapezoidProfile(
        au::Quantity<au::Time2ndDerivative<Units>, double> max_acceleration, 
        au::Quantity<au::TimeDerivative<Units>, double> max_velocity, 
        au::Quantity<Units, double> total_distance
    )  : 
        m_max_acceleration(max_acceleration), 
        m_max_velocity(max_velocity), 
        m_total_distance(au::abs(total_distance)),
        invert(total_distance < au::ZERO)
    {   
        // get the time it takes to accelerate to max velocity
        auto accel_time = m_max_velocity / m_max_acceleration;

        // the deceleration time is the same
        auto decel_time = accel_time;

        // get the acceleration distance via x = 1/2at^2
        auto accel_distance = (m_max_acceleration * au::int_pow<2>(accel_time)) / 2;

        // the deceleration distance is the same
        auto decel_distance = accel_distance;

        // gets coast distance by total - (accel + decel)
        auto coast_distance = m_total_distance - accel_distance - decel_distance;

        // if the coast distance is less than zero, compute the maximum acceleration we can reach in the time given
        if (coast_distance < au::ZERO) {
            // find the accel time via the equation sqrt(2x/a) = t
            accel_time = au::sqrt(m_total_distance / m_max_acceleration);

            // decel time will be the same as accel time
            decel_time = accel_time;

            // no coast distance
            coast_distance = au::ZERO;
        } 

        std::cout << accel_distance << std::endl;
        std::cout << coast_distance << std::endl;
        std::cout << decel_distance << std::endl;

        // compute the amount of time we want to coast for
        auto coast_time = coast_distance / m_max_velocity;

        // total time is all segments added together
        total_time = accel_time + decel_time + coast_time;

        std::cout << m_max_velocity << std::endl;

        std::cout << total_time << std::endl;

        accel_cutoff = accel_time;
        coast_cutoff = accel_time + coast_time;
        decel_cutoff = accel_time + coast_time + decel_time;
    }

    TrapezoidProfileStage stage(const au::Quantity<au::Seconds, double> elapsed_time) const {
        if (elapsed_time < this->accel_cutoff) {
        return TrapezoidProfileStage::Accelerating;
        } else if (elapsed_time < this->coast_cutoff) {
            return TrapezoidProfileStage::Coasting;
        } else if (elapsed_time < this->decel_cutoff) {
            return TrapezoidProfileStage::Decelerating;
        } else {
            return TrapezoidProfileStage::Done;
        }
    }

    ProfileSetpoint<Units> calculate(const au::Quantity<au::Seconds, double> elapsed_time) const {
        // integration constant for the coasting segment
        au::Quantity<Units, double> const_1 = 
            -m_max_velocity * accel_cutoff 
            + (m_max_acceleration / 2) * au::int_pow<2>(accel_cutoff);
        
        // integration constant for the deceleration segment
        au::Quantity<Units, double> const_2 = 
            -m_max_acceleration * total_time * coast_cutoff
            + (m_max_acceleration / 2) * au::int_pow<2>(coast_cutoff) 
            + m_max_velocity * coast_cutoff + const_1;
        
        ProfileSetpoint<Units> setpoint = ProfileSetpoint<Units>(au::ZERO, au::ZERO, au::ZERO);

        switch (this->stage(elapsed_time)) {
            case TrapezoidProfileStage::Accelerating:
                setpoint = ProfileSetpoint(
                    (m_max_acceleration / 2) * au::int_pow<2>(elapsed_time), 
                    m_max_acceleration * elapsed_time,
                    m_max_acceleration
                ); 
                break;
            case TrapezoidProfileStage::Coasting:
                setpoint =  ProfileSetpoint(
                    m_max_velocity * elapsed_time + const_1,
                    m_max_velocity,
                    m_max_acceleration - m_max_acceleration // long ass way to say 0 bc complier complains
                );
                break;
            case TrapezoidProfileStage::Decelerating:
                setpoint = ProfileSetpoint(
                    m_max_acceleration * total_time * elapsed_time 
                    - (m_max_acceleration / 2) * au::int_pow<2>(elapsed_time) 
                    + const_2,
                    m_max_acceleration * (total_time - elapsed_time),
                    -m_max_acceleration
                );
                break;
            case TrapezoidProfileStage::Done:
                setpoint = ProfileSetpoint<Units>(m_total_distance, au::ZERO, au::ZERO);
                break;
            default:
                // to stop the compiler from complaining
                setpoint = ProfileSetpoint<Units>(m_total_distance, au::ZERO, au::ZERO);
        }

        if (invert) {
            return setpoint.negative();
        } else {
            return setpoint;
        }
    }
};
}
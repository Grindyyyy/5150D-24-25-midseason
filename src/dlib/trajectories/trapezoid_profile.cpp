#include "dlib/trajectories/trapezoid_profile.hpp"
#include "api.h"
#include "au/au.hpp"
#include "profile_setpoint.hpp"
#include <cmath>

namespace dlib {

TrapezoidProfile::TrapezoidProfile(
    au::Quantity<au::MetersPerSecondSquared, double> max_acceleration, 
    au::Quantity<au::MetersPerSecond, double> max_velocity, 
    au::Quantity<au::Meters, double> total_distance
) : 
    max_acceleration(max_acceleration), 
    max_velocity(max_velocity), 
    total_distance(total_distance) 
{

    // flip max velocity and acceleration if the distance is negative
    if (total_distance < au::ZERO) {
        this->max_acceleration = -max_acceleration;
        this->max_velocity = -max_velocity;
    } else {
        this->max_acceleration = max_acceleration;
        this->max_velocity = max_velocity;
    }

    // get the time it takes to accelerate to max velocity
    auto accel_time = max_velocity / max_acceleration;

    // the deceleration time is the same
    auto decel_time = accel_time;

    // get the acceleration distance via x = 1/2at^2
    auto accel_distance = (max_acceleration * au::int_pow<2>(accel_time)) / 2;

    // the deceleration distance is the same
    auto decel_distance = accel_distance;

    // gets coast distance by total - (accel + decel)
    auto coast_distance = total_distance - accel_distance - decel_distance;

    // if the coast distance is less than zero, compute the maximum acceleration we can reach in the time given
    if (coast_distance < au::ZERO) {
        // find the accel time via the equation sqrt(2x/a) = t
        accel_time = au::sqrt(total_distance / max_acceleration);

        // decel time will be the same as accel time
        decel_time = accel_time;

        // no coast distance
        coast_distance = au::ZERO;
    } 

    // compute the amount of time we want to coast for
    auto coast_time = coast_distance / max_velocity;

    // total time is all segments added together
    total_time = accel_time + decel_time + coast_time;

    accel_cutoff = accel_time;
    coast_cutoff = accel_time + coast_time;
    decel_cutoff = accel_time + coast_time + decel_time;
}

TrapezoidProfileStage TrapezoidProfile::stage(au::Quantity<au::Seconds, double> elapsed_time) {
    if (elapsed_time <= this->accel_cutoff) {
        return TrapezoidProfileStage::Accelerating;
    } else if (elapsed_time <= this->coast_cutoff) {
        return TrapezoidProfileStage::Coasting;
    } else if (elapsed_time <= this->decel_cutoff) {
        return TrapezoidProfileStage::Decelerating;
    } else {
        return TrapezoidProfileStage::Done;
    }
}

ProfileSetpoint TrapezoidProfile::calculate(au::Quantity<au::Seconds, double> elapsed_time){
    // integration constant for the coasting segment
    au::Quantity<au::Meters, double> const_1 = 
        -max_velocity * accel_cutoff 
        + (max_acceleration / 2) * au::int_pow<2>(accel_cutoff);
    
    // integration constant for the deceleration segment
    au::Quantity<au::Meters, double> const_2 = 
        -max_acceleration * total_time * coast_cutoff
        + (max_acceleration / 2) * au::int_pow<2>(coast_cutoff) 
        + max_velocity * coast_cutoff + const_1;
    
    switch (this->stage(elapsed_time)) {
        case TrapezoidProfileStage::Accelerating:
            return ProfileSetpoint(
                (max_acceleration / 2) * au::int_pow<2>(elapsed_time), 
                max_acceleration * elapsed_time
            ); 
            break;
        case TrapezoidProfileStage::Coasting:
            return ProfileSetpoint(
                max_velocity * elapsed_time + const_1,
                max_velocity
            );
            break;
        case TrapezoidProfileStage::Decelerating:
            return ProfileSetpoint(
                max_acceleration * total_time * elapsed_time 
                - (max_acceleration / 2) * au::int_pow<2>(elapsed_time) 
                + const_2,
                max_acceleration * (total_time - elapsed_time)
            );
            break;
        case TrapezoidProfileStage::Done:
            return ProfileSetpoint(au::ZERO, au::ZERO);
            break;
    }

    // to stop the compiler from complaining
    return ProfileSetpoint(au::ZERO, au::ZERO);
}

}

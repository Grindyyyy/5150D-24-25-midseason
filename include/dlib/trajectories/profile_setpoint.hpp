#pragma once
#include "au/au.hpp"

namespace dlib {

// TODO: Technically these field names are only correct provided that the user uses a nonderivative unit
// like position or rotation. If the user uses velocity, for example, the 'correct' field names would be (velocity, acceleration, jerk)
// not sure there's any solution to this without making the field names needlessly vague

template<typename Units>
struct ProfileSetpoint {
    au::Quantity<Units, double> position;
    au::Quantity<au::TimeDerivative<Units>, double> velocity;
    au::Quantity<au::Time2ndDerivative<Units>, double> acceleration;

    ProfileSetpoint(
        au::Quantity<Units, double> position, 
        au::Quantity<au::TimeDerivative<Units>, double> velocity,
        au::Quantity<au::Time2ndDerivative<Units>, double> acceleration
    ) : 
        position(position), 
        velocity(velocity),
        acceleration(acceleration) {

    };
};


}
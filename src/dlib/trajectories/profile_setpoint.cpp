#include "dlib/trajectories/profile_setpoint.hpp"

namespace dlib {

ProfileSetpoint::ProfileSetpoint(
    au::Quantity<au::Meters, double> position, 
    au::Quantity<au::MetersPerSecond, double> velocity
) : 
    position(position), velocity(velocity) {

};

}
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

class TrapezoidProfile {
protected:
    au::Quantity<au::MetersPerSecondSquared, double> max_acceleration;
    au::Quantity<au::MetersPerSecond, double> max_velocity;
    
    au::Quantity<au::Meters, double> total_distance;
    au::Quantity<au::Seconds, double> total_time;

    au::Quantity<au::Seconds, double> accel_cutoff;
    au::Quantity<au::Seconds, double> coast_cutoff;
    au::Quantity<au::Seconds, double> decel_cutoff;
public:
    TrapezoidProfile(
        au::Quantity<au::MetersPerSecondSquared, double> max_acceleration, 
        au::Quantity<au::MetersPerSecond, double> max_velocity, 
        au::Quantity<au::Meters, double> total_distance
    );

    TrapezoidProfileStage stage(au::Quantity<au::Seconds, double> elapsed_time);
    ProfileSetpoint calculate(au::Quantity<au::Seconds, double> elapsed_time);
};
}
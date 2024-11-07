#pragma once
#include "au/au.hpp"
#include "pros/rtos.hpp"

namespace dlib {

struct Vector2d {
    au::Quantity<au::Meters, double> x;
    au::Quantity<au::Meters, double> y;

    Vector2d(
        au::Quantity<au::Meters, double> x, 
        au::Quantity<au::Meters, double> y
    );
};

struct Pose2d {
    au::Quantity<au::Meters, double> x;
    au::Quantity<au::Meters, double> y;
    au::Quantity<au::Degrees, double> theta;

    Pose2d(
        au::Quantity<au::Meters, double> x, 
        au::Quantity<au::Meters, double> y,
        au::Quantity<au::Degrees, double> theta
    );
};

class Odometry {
public:
    Odometry();

    void update(
        au::Quantity<au::Meters, double> left_displacement,
        au::Quantity<au::Meters, double> right_displacement,
        au::Quantity<au::Degrees, double> heading
    );

    Pose2d get_position();
    void set_position(Pose2d pose);

    au::Quantity<au::Meters, double> displacement_to(Vector2d point, bool reverse = false);
    au::Quantity<au::Degrees, double> angle_to(Vector2d point, bool reverse = false);

protected:
    Pose2d position = Pose2d(au::ZERO, au::ZERO, au::ZERO);
    au::Quantity<au::Meters, double> previous_forward = au::ZERO;
    au::Quantity<au::Radians, double> previous_theta = au::ZERO;

    pros::Mutex mutex{};
};

}
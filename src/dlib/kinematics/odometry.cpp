#include "dlib/kinematics/odometry.hpp"
#include "au/au.hpp"
#include <mutex>

namespace dlib {

Vector2d::Vector2d(au::Quantity<au::Meters, double> x, au::Quantity<au::Meters, double> y) : x(x), y(y) {

}

Pose2d::Pose2d(
    au::Quantity<au::Meters, double> x, 
    au::Quantity<au::Meters, double> y, 
    au::Quantity<au::Degrees, double> theta
) : x(x), y(y), theta(theta) {

}

Odometry::Odometry(
    au::Quantity<au::Meters, double> horizontal_wheel_offset
) : horizontal_wheel_offset(horizontal_wheel_offset) 
{

}

void Odometry::update(
    au::Quantity<au::Meters, double> left_displacement, 
    au::Quantity<au::Meters, double> right_displacement, 
    au::Quantity<au::Meters, double> horizontal_displacement,
    au::Quantity<au::Degrees, double> rotation
) {
    std::lock_guard<pros::Mutex> guard(this->mutex);

    auto current_forward = (left_displacement + right_displacement) / 2;
    auto current_horizontal = horizontal_displacement;
    auto current_theta = rotation.as(au::radians);

    auto delta_forward = current_forward - previous_forward;
    auto delta_horizontal = current_horizontal - previous_horizontal;
    auto delta_theta = current_theta - previous_theta;

    auto local_x = delta_forward;
    auto local_y = delta_horizontal;

    // if delta_theta is 0, the robot moved in a straight line
    if (delta_theta != au::radians(0)) {
        // arc_to_line * radius = chord_length
        double arc_to_line = 2 * au::sin(delta_theta / 2);
        local_x = arc_to_line * (local_x / delta_theta.in(au::radians));
        local_y = arc_to_line * (local_y / delta_theta.in(au::radians) + this->horizontal_wheel_offset);
    }

    // the average heading of the movement, simplified from (self.theta + (self.theta + delta_theta)) / 2
    auto average_theta = position.theta + delta_theta / 2;

    // rotate the local movement into the global frame
    auto global_x = position.x + (local_x * au::cos(average_theta) - local_y * au::sin(average_theta));
    auto global_y = position.y + (local_y * au::cos(average_theta) + local_x * au::sin(average_theta));
    auto global_theta = position.theta + delta_theta;

    // update odometry state
    previous_forward = current_forward;
    previous_horizontal = current_horizontal;
    previous_theta = current_theta;

    position = Pose2d {global_x,global_y,global_theta};
}

Pose2d Odometry::get_position() {
    return this->position;
}

void Odometry::set_position(Pose2d position) {
    this->position = position;
}

au::Quantity<au::Meters, double> Odometry::displacement_to(Vector2d point, bool reverse) {
    auto target_x = point.x;
    auto target_y = point.y;

    auto current_pose = this->position;
    auto current_x = current_pose.x;
    auto current_y = current_pose.y;
    
    // au does not include a hypot function unfortunately
    auto dist = au::sqrt(
        au::int_pow<2>(target_x - current_x) + au::int_pow<2>(target_y - current_y)
    );

    if (reverse) {
        dist = -dist;
    }

    return dist;
}

au::Quantity<au::Degrees, double> Odometry::angle_to(Vector2d point, bool reverse) {
    auto target_x = point.x;
    auto target_y = point.y;

    auto current_position = this->position;
    auto current_x = current_position.x;
    auto current_y = current_position.y;

    auto angle = au::arctan2(target_y - current_y, target_x - current_x);

    // if travelling in reverse we need the back of the robot to face the point
    if (reverse) {
        angle += au::degrees(180);
    }

    // convert the angle to be within [-180, +180] degrees
    angle = au::fmod(angle, au::degrees(360));

    if(angle > au::degrees(180)){
        angle -= au::degrees(360);
    }

    return angle;
}

}
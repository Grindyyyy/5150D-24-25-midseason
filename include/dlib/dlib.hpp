#pragma once
#include "dlib/controllers/feedforward.hpp"
#include "dlib/controllers/pid.hpp"
#include "dlib/controllers/error_derivative_settler.hpp"
#include "dlib/controllers/error_time_settler.hpp"

#include "dlib/hardware/chassis.hpp"
#include "dlib/hardware/imu.hpp"
#include "dlib/hardware/motor_group.hpp"
#include "dlib/hardware/motor.hpp"
#include "dlib/hardware/rotation.hpp"

#include "dlib/kinematics/odometry.hpp"

#include "dlib/trajectories/profile_setpoint.hpp"
#include "dlib/trajectories/trapezoid_profile.hpp"
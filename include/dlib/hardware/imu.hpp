#pragma once

#include "pros/imu.hpp"
#include "au/au.hpp"

namespace dlib {

struct ImuConfig {
    int8_t port;
    double scalar;

    ImuConfig(int8_t port, double scalar = 1);
};

class Imu {
protected:
    double scalar;
public:
    /**
     * @brief Initialize the Imu
     * 
     * @b Example
     * @code {.cpp}
     * void opcontrol(){
     * dlib::ImuConfig imu_config({
	 * 7,	// imu port
	 * 1.001	// optional imu scaling constant
	 * });
     * 
	 * dlib::Imu imu(imu_config);
     * 
	 * imu.initialize();
     * @endcode
    */
    void initialize();
    
    /**
     * @brief Get Imu rotation
     * 
     * @b Example
     * @code {.cpp}
     * void opcontrol(){
     * dlib::ImuConfig imu_config({
	 * 7,	// imu port
	 * 1.001	// optional imu scaling constant
	 * });
     * 
	 * dlib::Imu imu(imu_config);
     * 
	 * au::Quantity<au::Degrees, double> degrees = imu.get_rotation();
     * @endcode
    */
    au::Quantity<au::Degrees, double> get_rotation();

    /**
     * @brief Get Imu heading
     * 
     * @b Example
     * @code {.cpp}
     * void opcontrol(){
     * dlib::ImuConfig imu_config({
	 * 7,	// imu port
	 * 1.001	// optional imu scaling constant
	 * });
     * 
	 * dlib::Imu imu(imu_config);
     * 
	 * au::Quantity<au::Degrees, double> degrees = imu.get_heading();
     * @endcode
    */
    au::Quantity<au::Degrees, double> get_heading();
    
    Imu(ImuConfig config);

    pros::Imu raw;
};

}
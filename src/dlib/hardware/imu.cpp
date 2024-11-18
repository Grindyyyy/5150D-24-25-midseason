#include "dlib/hardware/imu.hpp"
#include "au/au.hpp"

namespace dlib {

ImuConfig::ImuConfig(int8_t port, double scalar) : port(port), scalar(scalar) {

}

Imu::Imu(ImuConfig config) : raw(config.port), scalar(config.scalar) {

}

void Imu::initialize() {
    this->raw.reset(true);
}

au::Quantity<au::Degrees, double> Imu::get_rotation() {
    // TODO: drift correction in addition to scaling?
    
    auto raw_reading = au::degrees(this->raw.get_rotation());
    auto corrected_reading = raw_reading * this->scalar;
    return corrected_reading;
}

au::Quantity<au::Degrees, double> Imu::get_heading() {
    auto raw_reading = au::degrees(this->raw.get_heading());
    auto corrected_reading = raw_reading * this->scalar;
    return corrected_reading;
}

}
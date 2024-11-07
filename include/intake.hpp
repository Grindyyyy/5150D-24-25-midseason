#pragma once
#include "api.h"

class Intake{
    public:
        pros::Motor intake_motor;
        pros::Optical color_sensor;
        pros::Distance distance_sensor;

        Intake(
            int8_t intake_port, // intake port
            int8_t color_sensor_port, // color sensor for the intake
            int8_t distance_sensor_port // distance sensor for the intake
        );

        void intake(int8_t volts);

        void outtake(int8_t volts);

        void max_intake();

        void rev_intake();

        // intake color sorter / wall stake sorter code
        // TODO: make it lossier and less laggy
        double intake_filter();

        void stop_intake();
};
#pragma once
#include "api.h"

enum class Alliance {
    Red,
    Blue
};

class Intake{
    public:
        pros::Motor intake_motor;
        pros::Optical color_sensor;
        pros::Distance distance_sensor;
        
        Alliance alliance;
        bool redirect;
        

        Intake(
            int8_t intake_port, // intake port
            int8_t color_sensor_port, // color sensor for the intake
            int8_t distance_sensor_port,
            Alliance alliance = Alliance::Blue,
            bool redirect = false // distance sensor for the intake
        );


        void set_alliance(Alliance new_alliance);

        void set_redirect(bool redirect);

        void move(int8_t volts);

        void max();

        void rev();

        // intake color sorter / wall stake sorter code
        // TODO: make it lossier and less laggy
        bool intake_filter();

        double get_red();
        double get_green();
        double get_blue();

        void stop();
};
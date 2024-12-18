#pragma once
#include "api.h"

enum class Alliance2 {
    Red,
    Blue
};

class Intake{
    public:
        pros::Motor intake_motor;
        pros::Optical color_sensor;
        pros::Distance distance_sensor;
        
        Alliance2 alliance;
        bool redirect;
        bool auton;
        bool auton_stick;

        pros::Mutex intake_mutex{};

        Intake(
            int8_t intake_port, // intake port
            int8_t color_sensor_port, // color sensor for the intake
            int8_t distance_sensor_port,
            Alliance2 alliance = Alliance2::Blue,
            bool redirect = false, // distance sensor for the intake
            bool auton_stick = false,
            bool auton = false
        );


        void set_alliance(Alliance2 new_alliance);

        Alliance2 get_alliance();

        void set_redirect(bool redirect);

        void set_mode(bool auton);

        void move(int8_t volts);

        void max();

        void rev();

        // intake color sorter / wall stake sorter code
        // TODO: make it lossier and less laggy
        bool intake_filter();

        double get_red();
        double get_green();
        double get_blue();

        bool get_redirect();

        void stop();
};
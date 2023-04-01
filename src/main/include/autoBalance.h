#pragma once
#include <frc/BuiltInAccelerometer.h>
#include <cmath>

class autoBalance{
    public:
        autoBalance();
        double get_pitch();
        double get_roll();
        double get_tilt();
        double get_tilt_backwards();
        double auto_balance_routine();
        double auto_balance_routine_backwards();
        int seconds_to_ticks(double time);
        
    private:
        frc::BuiltInAccelerometer m_accel{};
        int state;
        int time_counter;
        double robot_speed_slow;
        double robot_speed_fast;
        double on_charge_station_degree;
        double level_degree;
        double max_time;
        double single_tap_time;
        double scoring_back_up_time;
        double double_tap_time;
};
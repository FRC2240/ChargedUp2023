#include "autoBalance.h"
#include <iostream>

autoBalance::autoBalance(){
    state = 0;
    time_counter = 0;

    /**********
     * CONFIG *
     **********/
    //Speed the robot drived while scoring/approaching station, default = 0.4
    robot_speed_fast = 0.4;
    
    //Speed the robot drives while balancing itself on the charge station.
    //Should be roughly half the fast speed, to make the robot more accurate, default = 0.2
    robot_speed_slow = 0.25;

    //Angle where the robot knows it is on the charge station, default = 13.0
    on_charge_station_degree = 13.0;

    //Angle where the robot can assume it is level on the charging station
    //Used for exiting the drive forward sequence as well as for auto balancing, default = 6.0
    level_degree = 6.0;

    //Amount of time a sensor condition needs to be met before changing states in seconds
    //Reduces the impact of sensor noice, but too high can make the auto run slower, default = 0.2
    max_time = 0.2;
}

double autoBalance::get_pitch()
{
    return std::atan2((- m_accel.GetX()) , std::sqrt(m_accel.GetY() * m_accel.GetY() + m_accel.GetZ() * m_accel.GetZ())) * 57.3;
}

double autoBalance::get_roll()
{
    return std::atan2(m_accel.GetY() , m_accel.GetZ()) * 57.3;
}

//returns the magnititude of the robot's tilt calculated by the root of
//pitch^2 + roll^2, used to compensate for diagonally mounted rio
double autoBalance::get_tilt()
{
	double pitch = get_pitch();
	double roll = get_roll();
    if((pitch + roll) >= 0)
    {
        return -std::sqrt(pitch * pitch + roll * roll);
    } 
    else 
    {
        return std::sqrt(pitch * pitch + roll * roll);
    }
}

double autoBalance::get_tilt_backwards()
{
	double pitch = get_pitch();
	double roll = get_roll();
    if((pitch + roll) >= 0)
    {
        return std::sqrt(pitch * pitch + roll * roll);
    } 
    else 
    {
        return -std::sqrt(pitch * pitch + roll * roll);
    }
}

int autoBalance::seconds_to_ticks(double time)
{
    return (int)(time * 50);
}


//routine for automatically driving onto and engaging the charge station.
//returns a value from -1.0 to 1.0, which left and right motors should be set to.
double autoBalance::auto_balance_routine()
{
    switch (state)
    {
        //drive forwards to approach station, exit when tilt is detected
        case 0:
            if(get_tilt() > on_charge_station_degree)
            {
                time_counter++;
            }
            if(time_counter > seconds_to_ticks(max_time))
            {
                state = 1;
                time_counter = 0;
                return robot_speed_slow;
            }
            return robot_speed_fast;
        //driving up charge station, drive slower, stopping when level
        case 1:
            if (get_tilt() < level_degree)
            {
                time_counter++; 
            }
            if(time_counter > seconds_to_ticks(time_counter))
            {
                state = 2;
                time_counter = 0;
                return 0;
            }
            return robot_speed_slow;
        //on charge station, stop motors and wait for end of auto
        case 2:
            if(fabs(get_tilt()) <= level_degree/2)
            {
                time_counter++;
            }
            if(time_counter > seconds_to_ticks(max_time))
            {
                state = 4;
                time_counter = 0;
                return 0;
            }
            if(get_tilt() >= level_degree) {
                return 0.07;
            } else if(get_tilt() <= -level_degree) {
                return -0.07;
            }
        case 3:
            return 0;
    }
    return 0;
}

double autoBalance::auto_balance_routine_backwards(){
    switch (state){
        //drive forwards to approach station, exit when tilt is detected
        case 0:
            if(get_tilt_backwards() < -on_charge_station_degree)
            {
                time_counter++;
            }
            if(time_counter > seconds_to_ticks(max_time))
            {
                state = 1;
                time_counter = 0;
                return robot_speed_slow;
            }
            return robot_speed_fast;
        //driving up charge station, drive slower, stopping when level
        case 1:
            if (get_tilt_backwards() > -level_degree)
            {
                time_counter++; 
            }
            if(time_counter > seconds_to_ticks(max_time))
            {
                state = 2;
                time_counter = 0;
                return 0;
            }
            return robot_speed_slow;
        //on charge station, stop motors and wait for end of auto
        case 2:
            if(fabs(get_tilt_backwards()) <= level_degree/2)
            {
                time_counter++;
            }
            if(time_counter > seconds_to_ticks(max_time))
            {
                state = 4;
                time_counter = 0;
                return 0;
            }
            if(get_tilt_backwards() >= level_degree) 
            {
                return 0.07;
            } else if(get_tilt() <= -level_degree) 
            {
                return -0.07;
            }
        case 3:
            return 0;
    }
    return 0;
}
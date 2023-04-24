#include "autoBalance.h"
#include <iostream>
#include <AHRS.h>

static std::unique_ptr<AHRS> navx;

autoBalance::autoBalance(){
    state = 0;
    time_counter = 0;

    /**********
     * CONFIG *
     **********/
    //Speed the robot drived while scoring/approaching station, default = 0.4
    robot_speed_fast = 0.6;
    
    //Speed the robot drives while balancing itself on the charge station.
    //Should be roughly half the fast speed, to make the robot more accurate, default = 0.2
    robot_speed_slow = 0.1;

    //Angle where the robot knows it is on the charge station, default = 13.0
    on_charge_station_degree = 11.0;

    //Angle where the robot can assume it is level on the charging station
    //Used for exiting the drive forward sequence as well as for auto balancing, default = 6.0
    level_degree = 9.5;

    //Amount of time a sensor condition needs to be met before changing states in seconds
    //Reduces the impact of sensor noice, but too high can make the auto run slower, default = 0.2
    max_time = 0.2;

    navx = std::make_unique<AHRS>(frc::SPI::Port::kMXP);
}

int autoBalance::seconds_to_ticks(double time)
{
    return (int)(time * 50);
}

double autoBalance::get_pitch()
{
    return navx->GetPitch();
}

double autoBalance::auto_balance_routine(){
    //std::cout << "pitch: " << get_pitch() << std::endl;
    switch (state){
        //drive forwards to approach station, exit when tilt is detected
        case 0:
            //std::cout << "aproaching\n";
            if(fabs(get_pitch()) > on_charge_station_degree)
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
            //std::cout << "on charge station\n";
            if (fabs(get_pitch()) < level_degree)
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
        //std::cout << "balancing\n";
            if(fabs(get_pitch()) <= level_degree/2)
            {
                time_counter++;
                //std::cout << "time counter: " << time_counter << std::endl;
            }
            if(time_counter > (seconds_to_ticks(max_time)))
            {
                //std::cout << "balanced\n";
                state = 4;
                time_counter = 0;
                return 0;
            }
            if(get_pitch() >= level_degree) 
            {
                //std::cout << "Backing up\n";
                return -0.0725;
            } else if(get_pitch() <= -level_degree) 
            {
                //std::cout << "Going forward\n";
                return 0.0725;
            }
            
        case 3:
            return 0;
    }
    return 0;
}
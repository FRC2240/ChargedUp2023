
#include "autoBalance.h"
#include "iostream"

autoBalance::autoBalance(){
    state = 0;
    debounceCount = 0;

    /**********
     * CONFIG *
     **********/
    //Speed the robot drived while scoring/approaching station, default = 0.4
    robotSpeedFast = 0.3;
    
    //Speed the robot drives while balancing itself on the charge station.
    //Should be roughly half the fast speed, to make the robot more accurate, default = 0.2
    robotSpeedSlow = 0.25;

    //Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree = 13.0;

    //Angle where the robot can assume it is level on the charging station
    //Used for exiting the drive forward sequence as well as for auto balancing, default = 6.0
    levelDegree = 6.0;

    //Amount of time a sensor condition needs to be met before changing states in seconds
    //Reduces the impact of sensor noice, but too high can make the auto run slower, default = 0.2
    debounceTime = 0.2;
}

double autoBalance::getPitch(){
    return std::atan2((- mAccel.GetX()) , std::sqrt(mAccel.GetY() * mAccel.GetY() + mAccel.GetZ() * mAccel.GetZ())) * 57.3;
}

double autoBalance::getRoll(){
    return std::atan2(mAccel.GetY() , mAccel.GetZ()) * 57.3;
}

//returns the magnititude of the robot's tilt calculated by the root of
//pitch^2 + roll^2, used to compensate for diagonally mounted rio
double autoBalance::getTilt(){
	double pitch = getPitch();
	double roll = getRoll();
    if((pitch + roll)>= 0){
        return -std::sqrt(pitch*pitch + roll*roll);
    } else {
        return std::sqrt(pitch*pitch + roll*roll);
    }
}

int autoBalance::secondsToTicks(double time){
    return (int)(time*50);
}


//routine for automatically driving onto and engaging the charge station.
//returns a value from -1.0 to 1.0, which left and right motors should be set to.
double autoBalance::autoBalanceRoutine(){
    
    switch (state){
        //drive forwards to approach station, exit when tilt is detected
        
        case 0:
            std::cout << "aproaching\n";
            if(getTilt() > onChargeStationDegree){
                debounceCount++;
            }
            if(debounceCount > secondsToTicks(debounceTime)){
                state = 1;
                debounceCount = 0;
                return robotSpeedSlow;
            }
            return robotSpeedFast;
        //driving up charge station, drive slower, stopping when level
        case 1:
        std::cout << "climbing\n";
            if (getTilt() < levelDegree){
                debounceCount++; 
            }
            if(debounceCount > secondsToTicks(debounceTime)){
                state = 2;
                debounceCount = 0;
                return 0;
            }
            return robotSpeedSlow;
        //on charge station, stop motors and wait for end of auto
        case 2:
        std::cout << "balancing\n";
            if(fabs(getTilt()) < levelDegree){
                debounceCount++;
            }
            if(debounceCount>secondsToTicks(debounceTime)){
                state = 3;
                debounceCount = 0;
                return 0;
            }
            if(getTilt() >= levelDegree) {
                return 0.1;
            } else if(getTilt() <= -levelDegree) {
                return -0.1;
            }
            break;

        case 3:
        std::cout << "balanced\n";
            if(fabs(getTilt()) >= onChargeStationDegree){
                debounceCount++;
            }
            if(debounceCount > secondsToTicks(debounceTime)){
                if (getTilt() >= onChargeStationDegree){
                    state = 1;
                    debounceCount = 0;
                    return robotSpeedSlow;
                }
                else if (getTilt() <= -onChargeStationDegree){
                    state = 4;
                    debounceCount = 0;
                    return -robotSpeedSlow;
                }
            }
            return 0;

        case 4: 
        std::cout << "climbing other side\n";
            if (getTilt() > -levelDegree){
                debounceCount++; 
            }
            if(debounceCount > secondsToTicks(debounceTime)){
                state = 2;
                debounceCount = 0;
                return 0;
            }
            return -robotSpeedSlow;
    }
    return 0;
}


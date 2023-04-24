#pragma once

#include <units/length.h>
#include <units/angle.h>
#include <frc/DriverStation.h>
#include <iostream>
#include <vector>
#include <units/time.h>
#include <units/velocity.h>
#include <ctre/Phoenix.h>

namespace CONSTANTS 
{
   constexpr double NON_TURBO = 1;
  constexpr double DEADBAND = 0.15;
  constexpr bool DEBUGGING = true; //DO NOT USE IN COMP.
  //SLOWS DOWN EVERYTHING & MAY CAUSE WATCHDOG EXEPTIONS.
    enum STATES 
  {
            STORED,
            LOW,
            MID, 
            HP,
            HIGH,
            PICKUP,
            SCORE,
            FALLBACK,
            FALLBACK2,
            ABORT,
            O_LOW,
            O_MID,
            O_HP,
            O_HIGH,
            IDLE,
            O_OPEN,
            O_UP,
            O_PICKUP,
            O_HP_PICKUP,
            AUTO_SIMP_HIGH,
            HP_AUTO_INIT,
            HP_AUTO
};

namespace DRIVE 
{

 }

namespace TRAJECTORY
  {
    constexpr auto HP_VEL = 0.4_mps;
    constexpr units::meter_t SIMPLE_FORWARDS = -22_in; //CHANGEME
    const std::vector<units::meter_t> Y_POS =
      {
        /*
         * A list of all Y positions to score at.
         * Ordered from the pipe farthest on the robot's left side of the
         * red alliance grid to the rightmost.
         *
         * Since the field isn't mirrored, the lists are the same for the
         * red alliance and the blue alliance.
         *
         *  Copyright Westly Miller, 2023.
         */
        38.386_in,
        //16.386_in, //cube
        -5.614_in,
        -27.614_in,
        //-49.614_in, //cube
        -71.614_in,
        -93.614_in,
        //-115.614_in, //cube
        -137.614_in
      };
    namespace R
    {
      //Red Team and blue team will use seperate data.

      constexpr units::meter_t HIGH_X = 6.50_m;
      constexpr units::meter_t MID_X = 6.12_m;
      constexpr units::meter_t LOW_X = 6.09_m;
      constexpr units::meter_t HP = -6.6_m;
    }

    namespace B
    {
      constexpr units::meter_t HIGH_X = -6.50_m;
      constexpr units::meter_t MID_X = -6.12_m;
      constexpr units::meter_t LOW_X = -6.06_m;
      constexpr units::meter_t HP = 6.6_m;
    }
  }
  namespace GRIPPAD
  {
    constexpr int GRIPPAD_PISTON_ID1 = 0;
    constexpr int GRIPPAD_PISTON_ID2 = 3;
  }

  namespace WRIST
  {
    constexpr int WRIST_MOTOR_ID = 1;
    constexpr double PICKUP = 0.66;
    constexpr double HP = 0.48;
  }

  namespace ARM
  {
    constexpr units::second_t DELAY = 2_s;
    constexpr int RIGHT_ARM_MOTOR_ID = 3;
    constexpr int LEFT_ARM_MOTOR_ID = 2;
    constexpr double ARM_ENCODER_OFFSET = 284.0; // tracks the difference from original calibration to current
    constexpr int ARM_CANCODER_ID = 1;
    constexpr double THRESHOLD = 17.0;
    constexpr int HORIZONTAL_POINT = 143;
    constexpr double MAX_AFF = 0.075;

    namespace PID
    {
      constexpr double kP = 0.1;
      constexpr double kI = 0.0;
      constexpr double kD = 0.0;
      constexpr double kIz = 0.0;
      constexpr double kFF = 0.0;
      constexpr double kMaxOutput = 1.0;
      constexpr double kMinOutput = -1.0;
      constexpr double slotIdx = 0;
      constexpr double pidIdx = 0;

    }

    namespace MOTORPOSITIONS
    {
      // All units in degrees
      constexpr double STORED = 36.0;
      constexpr double LOW = 65.7;
      constexpr double MID = 136.1;
      constexpr double HP = 151.3;
      constexpr double HIGH = 155.0;
      constexpr double PICKUP = 65.8;
      constexpr double UP = 155.0;
    }
  }

  namespace VISION {
    //Remove above warning when values found
    constexpr int APRILTAG_PIPE = 1; 
    //Remove above warning when values found
    constexpr int BUFFER_SIZE = 14;
    constexpr int MIN_GOOD_FRAMES = 9;
    constexpr double MAX_STD_DEV = 0.04;
    constexpr double MIN_STD_DEV = 1.0e-16;
    constexpr double MAX_STD_DEV_ROT = 10; //CHANGEME
    constexpr double MIN_STD_DEV_ROT = 1.0e-10; //CHANGEME

  }

  namespace CANDLE {
    constexpr int CANDLE_ID = 5;
  }

  namespace GRABBER{
    constexpr int GRABBER_PISTON_ID1 = 1;
    constexpr int GRABBER_PISTON_ID2 = 2;
    constexpr int TOF_CAN = 88;
    constexpr double TOF_MAX = 250.0;

  }
}


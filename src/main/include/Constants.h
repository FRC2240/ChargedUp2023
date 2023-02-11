#pragma once

#include <units/length.h>
#include <units/angle.h>
#include <frc/DriverStation.h>
#include <iostream>

namespace CONSTANTS 
{
  constexpr bool DEBUGGING = true; //DO NOT USE IN COMP. 
  //SLOWS DOWN EVERYTHING & MAY CAUSE WATCHDOG EXEPTIONS.
  namespace COLORS
  {

    units::meter_t HIGH_X;
    units::meter_t MID_X;
    units::meter_t GROUND_X;

    namespace Y
    {
      units::meter_t L1;
      units::meter_t L2;
      units::meter_t L3;

      units::meter_t C1;
      units::meter_t C2;
      units::meter_t C3;

      units::meter_t R1;
      units::meter_t R2;
      units::meter_t R3;
    }
  }
  namespace GRIPPAD
  {
    constexpr int GRIPPAD_CHANNEL = 0;
  }

  namespace WRIST
  {
    constexpr int WRIST_MOTOR_ID = 1;
  }

  namespace ARM
  {
    constexpr int RIGHT_ARM_MOTOR_ID = 3;
    constexpr int LEFT_ARM_MOTOR_ID = 2;
    constexpr double ARM_ENCODER_OFFSET = -112.0;
    constexpr int ARM_CANCODER_ID = 1;
    constexpr double MIN_THRESHOLD = 0.98;
    constexpr double MAX_THRESHOLD = 1.02;
    
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
      //Add 112 (the offset) and 7 (the margin of error) to all values
      constexpr double STORED = 134.0;
      constexpr double LOW = 189.0;
      constexpr double MED = 244.0;
      constexpr double HP = 0.0;
      constexpr double HIGH = 260.0;
      constexpr double PICKUP = 159.0;
    }
  }

  namespace VISION {
    //Remove above warning when values found
    constexpr int APRILTAG_PIPE = 1; 
    constexpr int BUFFER_SIZE = 5; 
    constexpr int MIN_GOOD_FRAMES = 4;
    constexpr double MAX_STD_DEV = 10; //CHANGEME
    constexpr double MIN_STD_DEV = 1.0e-10; //CHANGEME
    constexpr double MAX_STD_DEV_ROT = 10; //CHANGEME
    constexpr double MIN_STD_DEV_ROT = 1.0e-5; //CHANGEME

  }

  namespace CANDLE {
    constexpr int CANDLE_ID = 5;
  }

  namespace GRABBER{
    constexpr int GRABBER_PISTON_ID1 = 1;
    constexpr int GRABBER_PISTON_ID2 = 2;

  }
}

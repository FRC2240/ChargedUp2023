#ifndef CONSTANTS_H_
#define CONSTANTS_H_

namespace CONSTANTS 
{
  namespace GRIPPAD
  {
    constexpr int GRIPPAD_CHANNEL = 0;
  }

  namespace WRIST
  {
    constexpr int WRIST_MOTOR_ID = 1;
    constexpr double WRIST_ENCODER_OFFSET = 0.709833;
    constexpr double WRIST_FLARE_OFFSET = 0.1;
  }

  namespace ARM
  {
    constexpr int RIGHT_ARM_MOTOR_ID = 2;
    constexpr int LEFT_ARM_MOTOR_ID = 3;
    constexpr double ARM_ENCODER_OFFSET = 106.807;
    constexpr double ARM_FLARE_HIGH = -60;
    constexpr double ARM_FLARE_LOW = -70;
    constexpr int ARM_CANCODER_ID = 1;
    
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
      constexpr int STORED = 0.0;
      constexpr int LOW = 0.0;
      constexpr int MED = 0.0;
      constexpr int HP = 0.0;
      constexpr int HIGH = 0.0;
    }
  }

  namespace VISION {
    constexpr int APRILTAG_PIPE = 1; //CHANGEME
    constexpr int ARRAY_SIZE = 5;
    constexpr double MAX_STD_DEV = 1; //CHANGEME

  }

  namespace CANDLE {
    constexpr int CANDLE_ID = 5;
  }

  namespace GRABBER{
    constexpr int GRABBER_PISTON_ID1 = 1;
    constexpr int GRABBER_PISTON_ID2 = 2;

  }
}
#endif // CONSTANTS_H_

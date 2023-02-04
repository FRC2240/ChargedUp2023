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
    constexpr double WRIST_ENCODER_OFFSET = 0.0;
    constexpr double WRIST_FLARE_OFFSET = 10.0;
  }

  namespace ARM
  {
    constexpr int RIGHT_ARM_MOTOR_ID = 2;
    constexpr int LEFT_ARM_MOTOR_ID = 3;
    constexpr double ARM_ENCODER_OFFSET = 0.0;
    constexpr double ARM_FLARE_HIGH = -60;
    constexpr double ARM_FLARE_LOW = -70;
    
    namespace PID
    {
      constexpr int kP = 0.0;
      constexpr int kI = 0.0;
      constexpr int kD = 0.0;
      constexpr int kIz = 0.0;
      constexpr int kFF = 0.0;
      constexpr int kMaxOutput = 0.0;
      constexpr int kMinOutput = 0.0;
      constexpr int slotIdx = 0;
      constexpr int pidIdx = 0;

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
    constexpr int GRABBER_MOTOR_ID = 6;
    constexpr int GRABBER_PISTON_ID1 = 1;
    constexpr int GRABBER_PISTON_ID2 = 2;

  }
}
#endif // CONSTANTS_H_

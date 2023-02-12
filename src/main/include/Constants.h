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
  }

  namespace ARM
  {
    constexpr int RIGHT_ARM_MOTOR_ID = 3;
    constexpr int LEFT_ARM_MOTOR_ID = 2;
    constexpr double ARM_ENCODER_OFFSET = -112.0;
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

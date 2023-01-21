#ifndef CONSTANTS_H_
#define CONSTANTS_H_

namespace CONSTANTS 
{
  namespace GRIPPAD
  {
    constexpr int ALPHA_CHANNEL = 0;
    constexpr int BETA_CHANNEL = 1;
    constexpr int GAMMA_CHANNEL = 2;
    constexpr int DELTA_CHANNEL = 3;
  }

  namespace GRABBER
  {
    constexpr int WRIST_MOTOR_ID = 1;

  }

  namespace ARM
  {
    constexpr int RIGHT_ARM_MOTOR_ID = 2;
    constexpr int LEFT_ARM_MOTOR_ID = 3;
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
  }

  namespace VISION {
    constexpr int APRILTAG_PIPE = 1; //CHANGEME
    constexpr int ARRAY_SIZE = 5;
    constexpr double MAX_STD_DEV = 1; //CHANGEME

  }

  namespace CANDLE {
    constexpr int CANDLE_ID = 5;
  }
}
#endif // CONSTANTS_H_

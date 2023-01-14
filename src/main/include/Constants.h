#ifndef CONSTANTS_H_
#define CONSTANTS_H_

namespace CONSTANTS 
{

    namespace GRABBER
    {
        constexpr int WRIST_MOTOR_ID = 1;

    }

    namespace ARM
    {
      constexpr int RIGHT_ARM_MOTOR_ID = 2;
      constexpr int LEFT_ARM_MOTOR_ID = 3;
        
    }

  namespace VISION {
    constexpr int APRILTAG_PIPE = 1; //CHANGEME
    constexpr int ARRAY_SIZE = 100; //CHANGEME
  }
}
#endif // CONSTANTS_H_

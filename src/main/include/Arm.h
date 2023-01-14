#ifndef ARM_H_
#define ARM_H_

#include "ctre/Phoenix.h"
#include <frc/Encoder.h>
#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"

class Arm
{
 public:
  Arm();
  ~Arm();
  void Up();
  void Down();
  void ArmPIDInit();
  void ArmDashInit();
  void ArmDashRead();

//Arm PID

 private:

    WPI_TalonFX m_arm_motor_right {CONSTANTS::ARM::RIGHT_ARM_MOTOR_ID};
    WPI_TalonFX m_arm_motor_left {CONSTANTS::ARM::LEFT_ARM_MOTOR_ID};

    TalonFXSensorCollection m_Arm_RightEncoder = m_arm_motor_right.GetSensorCollection();

    struct pidCoeff 
    {
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    };

  pidCoeff m_Arm_RightCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};
#endif //ARM_H_

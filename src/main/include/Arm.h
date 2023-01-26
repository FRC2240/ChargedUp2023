#ifndef ARM_H_
#define ARM_H_

#include "ctre/Phoenix.h"
#include <frc/Encoder.h>
#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"

class Arm
{

  public:
  enum TARGETS {LOW, MED, HUMANPLAYER, HI};

  Arm();
  ~Arm();
  enum STATES { STORED, MOVING, MOVED};

  STATES arm_logic(TARGETS desired_pos,
                   bool stored_button,
                   bool ground_button,
                   bool mid_button,
                   bool hp_button,
                   bool high_button);
  void move(TARGETS pos);
  void arm_pid_init();
  void arm_dash_init();
  void arm_dash_read();
  double desired_position;

 private:
  enum POSITIONS { STOWED, GROUND, MID, HP_STATION, HIGH};

  STATES state = STORED;

    WPI_TalonFX m_arm_motor_right {CONSTANTS::ARM::RIGHT_ARM_MOTOR_ID};
    WPI_TalonFX m_arm_motor_left {CONSTANTS::ARM::LEFT_ARM_MOTOR_ID};

    TalonFXSensorCollection m_Arm_RightEncoder = m_arm_motor_right.GetSensorCollection();

    struct pidCoeff 
    {
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    };

    pidCoeff m_Arm_RightCoeff{CONSTANTS::ARM::PID::kP, CONSTANTS::ARM::PID::kI, 
                              CONSTANTS::ARM::PID::kD, CONSTANTS::ARM::PID::kIz, 
                              CONSTANTS::ARM::PID::kFF, CONSTANTS::ARM::PID::kMaxOutput, 
                              CONSTANTS::ARM::PID::kMinOutput};
};
#endif //ARM_H_

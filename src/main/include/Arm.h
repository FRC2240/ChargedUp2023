#pragma once

#include "ctre/Phoenix.h"
#include <frc/Encoder.h>
#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <cmath>
#include <frc/Timer.h>

class Arm
{

  public:
  Arm();
  ~Arm();

  void move();
  void test();
  double read_position();
  void force_move(double pos);
  bool arm_moved(CONSTANTS::STATES state);
  double desired_position;

  double position;

  WPI_CANCoder arm_cancoder{CONSTANTS::ARM::ARM_CANCODER_ID};

  double AFF;

 private:
    WPI_TalonFX m_arm_motor_right {CONSTANTS::ARM::RIGHT_ARM_MOTOR_ID};
    WPI_TalonFX m_arm_motor_left {CONSTANTS::ARM::LEFT_ARM_MOTOR_ID};

    frc::Timer m_timer;
};

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
  double get_absolute_pos();
  void force_move(double pos);
  bool arm_moved(CONSTANTS::STATES state);
  double desired_position = CONSTANTS::ARM::MOTORPOSITIONS::STORED;

  double position;

 private:
  double translate_pos(double raw);
    WPI_TalonFX m_arm_motor_right {CONSTANTS::ARM::RIGHT_ARM_MOTOR_ID};
    WPI_TalonFX m_arm_motor_left {CONSTANTS::ARM::LEFT_ARM_MOTOR_ID};

    WPI_CANCoder arm_cancoder{CONSTANTS::ARM::ARM_CANCODER_ID};

    frc::Timer m_timer;

    TalonFXSensorCollection m_arm_right_encoder = m_arm_motor_right.GetSensorCollection();
};

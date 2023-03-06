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

  double position;

 private:

    WPI_TalonFX m_arm_motor_right {CONSTANTS::ARM::RIGHT_ARM_MOTOR_ID};
    WPI_TalonFX m_arm_motor_left {CONSTANTS::ARM::LEFT_ARM_MOTOR_ID};

    WPI_CANCoder arm_cancoder{CONSTANTS::ARM::ARM_CANCODER_ID};

    frc::Timer m_timer;

    double horizontal_point = 244;

    double desired_position;

    double max_AFF = 0.075;

    TalonFXSensorCollection m_arm_right_encoder = m_arm_motor_right.GetSensorCollection();

    struct pidCoeff 
    {
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    };

    pidCoeff m_arm_right_coeff{CONSTANTS::ARM::PID::kP, CONSTANTS::ARM::PID::kI, 
                              CONSTANTS::ARM::PID::kD, CONSTANTS::ARM::PID::kIz, 
                              CONSTANTS::ARM::PID::kFF, CONSTANTS::ARM::PID::kMaxOutput, 
                              CONSTANTS::ARM::PID::kMinOutput};
};

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
  void arm_pid_init();
  void arm_dash_init();
  void arm_dash_read();
  void test();
  double Read_Position();
  void force_move(double pos);
  void Up();
  void Down();
  void Stop();
  bool arm_moved(CONSTANTS::STATES state);
  double desired_position;

  double position;

 private:
  double ARM_FLARE_HIGH;
  double ARM_FLARE_LOW;
  bool store_button;
  bool pickup_button;
  bool low_button;
  bool med_button;
  bool hp_button;
  bool high_button;
  bool open_grabber;

    WPI_TalonFX m_arm_motor_right {CONSTANTS::ARM::RIGHT_ARM_MOTOR_ID};
    WPI_TalonFX m_arm_motor_left {CONSTANTS::ARM::LEFT_ARM_MOTOR_ID};

    WPI_CANCoder arm_cancoder{CONSTANTS::ARM::ARM_CANCODER_ID};

    frc::Timer m_timer;

    double horizontalPoint = 244;

    double maxAFF = 0.075;

    TalonFXSensorCollection m_Arm_RightEncoder = m_arm_motor_right.GetSensorCollection();

    int setPoint = 200;

    struct pidCoeff 
    {
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    };

    pidCoeff m_Arm_RightCoeff{CONSTANTS::ARM::PID::kP, CONSTANTS::ARM::PID::kI, 
                              CONSTANTS::ARM::PID::kD, CONSTANTS::ARM::PID::kIz, 
                              CONSTANTS::ARM::PID::kFF, CONSTANTS::ARM::PID::kMaxOutput, 
                              CONSTANTS::ARM::PID::kMinOutput};
};

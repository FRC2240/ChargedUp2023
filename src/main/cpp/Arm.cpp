#include "Arm.h"

Arm::Arm()
{
    m_arm_motor_left.Follow(m_arm_motor_right);
    Arm::ArmPIDInit();
    Arm::ArmDashInit();
}

void Arm::Up()
{


}

void Arm::Down()
{


}

void Arm::ArmPIDInit()
{
    Arm::ArmDashRead();
 
  m_arm_motor_right.ConfigFactoryDefault();

  m_arm_motor_right.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);

  m_arm_motor_right.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 0, 0);
  m_arm_motor_right.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 0, 0);

  m_arm_motor_right.ConfigNominalOutputForward(0, 0);
  m_arm_motor_right.ConfigNominalOutputReverse(0, 0);
  m_arm_motor_right.ConfigPeakOutputForward(m_Arm_RightCoeff.kMaxOutput, 0);
  m_arm_motor_right.ConfigPeakOutputReverse(m_Arm_RightCoeff.kMinOutput, 0);

  m_arm_motor_right.SelectProfileSlot(0, 0);
  m_arm_motor_right.Config_kF(0, m_Arm_RightCoeff.kFF, 0);
  m_arm_motor_right.Config_kP(0, m_Arm_RightCoeff.kP, 0);
  m_arm_motor_right.Config_kI(0, m_Arm_RightCoeff.kI, 0);
  m_arm_motor_right.Config_kD(0, m_Arm_RightCoeff.kD, 0);
}

void Arm::ArmDashInit()
{
    frc::SmartDashboard::PutNumber("Left Elevator P Gain", m_Arm_RightCoeff.kP);
    frc::SmartDashboard::PutNumber("Left Elevator I Gain", m_Arm_RightCoeff.kI);
    frc::SmartDashboard::PutNumber("Left Elevator D Gain", m_Arm_RightCoeff.kD);
    frc::SmartDashboard::PutNumber("Left Elevator FF Gain", m_Arm_RightCoeff.kFF);
    frc::SmartDashboard::PutNumber("Left Elevator Max Output", m_Arm_RightCoeff.kMaxOutput);
    frc::SmartDashboard::PutNumber("Left Elevator Min Output", m_Arm_RightCoeff.kMinOutput);

}

void Arm::ArmDashRead()
{
    m_Arm_RightCoeff.kP = frc::SmartDashboard::GetNumber("Left Elevator P Gain", 0.0);
    m_Arm_RightCoeff.kI = frc::SmartDashboard::GetNumber("Left Elevator I Gain", 0.0);
    m_Arm_RightCoeff.kD = frc::SmartDashboard::GetNumber("Left Elevator D Gain", 0.0);
    m_Arm_RightCoeff.kFF = frc::SmartDashboard::GetNumber("Left Elevator FF Gain", 0.0);
    m_Arm_RightCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Left Elevator Min Output", -1.0);
    m_Arm_RightCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Left Elevator Mtput", 1.0);

}



Arm::~Arm(){}

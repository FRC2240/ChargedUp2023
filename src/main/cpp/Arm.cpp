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

  m_arm_motor_right.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, CONSTANTS::ARM::PID::pidIdx, 0);

  m_arm_motor_right.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 0, 0);
  m_arm_motor_right.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 0, 0);

  m_arm_motor_right.ConfigNominalOutputForward(0, 0);
  m_arm_motor_right.ConfigNominalOutputReverse(0, 0);
  m_arm_motor_right.ConfigPeakOutputForward(m_Arm_RightCoeff.kMaxOutput, 0);
  m_arm_motor_right.ConfigPeakOutputReverse(m_Arm_RightCoeff.kMinOutput, 0);

  m_arm_motor_right.SelectProfileSlot(CONSTANTS::ARM::PID::slotIdx, CONSTANTS::ARM::PID::pidIdx);
  m_arm_motor_right.Config_kF(CONSTANTS::ARM::PID::slotIdx, m_Arm_RightCoeff.kFF, 0);
  m_arm_motor_right.Config_kP(CONSTANTS::ARM::PID::slotIdx, m_Arm_RightCoeff.kP, 0);
  m_arm_motor_right.Config_kI(CONSTANTS::ARM::PID::slotIdx, m_Arm_RightCoeff.kI, 0);
  m_arm_motor_right.Config_kD(CONSTANTS::ARM::PID::slotIdx, m_Arm_RightCoeff.kD, 0);
}

void Arm::ArmDashInit()
{
    frc::SmartDashboard::PutNumber("Right Arm P Gain", m_Arm_RightCoeff.kP);
    frc::SmartDashboard::PutNumber("Right Arm I Gain", m_Arm_RightCoeff.kI);
    frc::SmartDashboard::PutNumber("Right Arm D Gain", m_Arm_RightCoeff.kD);
    frc::SmartDashboard::PutNumber("Right Arm FF Gain", m_Arm_RightCoeff.kFF);
    frc::SmartDashboard::PutNumber("Right Arm Max Output", m_Arm_RightCoeff.kMaxOutput);
    frc::SmartDashboard::PutNumber("Right Arm Min Output", m_Arm_RightCoeff.kMinOutput);

}

void Arm::ArmDashRead()
{
    m_Arm_RightCoeff.kP = frc::SmartDashboard::GetNumber("Right Arm P Gain", 0.0);
    m_Arm_RightCoeff.kI = frc::SmartDashboard::GetNumber("Right Arm I Gain", 0.0);
    m_Arm_RightCoeff.kD = frc::SmartDashboard::GetNumber("Right Arm D Gain", 0.0);
    m_Arm_RightCoeff.kFF = frc::SmartDashboard::GetNumber("Right Arm FF Gain", 0.0);
    m_Arm_RightCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Right Arm Min Output", -1.0);
    m_Arm_RightCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Right Arm Max Output", 1.0);

}



Arm::~Arm(){}

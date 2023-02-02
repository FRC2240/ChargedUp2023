#include <iostream>
#include <Wrist.h>
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>


Wrist::Wrist(){

        WristPIDInit();
        WristDashInit();
        WristDashRead();
        
  m_wrist_Encoder.SetInverted(false);
  m_wrist_PIDController.SetFeedbackDevice(m_wrist_Encoder);

}

Wrist::~Wrist(){}

void Wrist::WristPIDInit() {
 
    Wrist::WristDashInit();
 
    m_wrist_PIDController.SetP(m_wrist_coeff.kP);
    m_wrist_PIDController.SetI(m_wrist_coeff.kI);
    m_wrist_PIDController.SetD(m_wrist_coeff.kD);
    m_wrist_PIDController.SetIZone(m_wrist_coeff.kIz);
    m_wrist_PIDController.SetFF(m_wrist_coeff.kFF);
    m_wrist_PIDController.SetOutputRange(m_wrist_coeff.kMinOutput, m_wrist_coeff.kMaxOutput);
    

}

void Wrist::WristDashInit()
{
    frc::SmartDashboard::PutNumber("Wrist P Gain", m_wrist_coeff.kP);
    frc::SmartDashboard::PutNumber("Wrist I Gain", m_wrist_coeff.kI);
    frc::SmartDashboard::PutNumber("Wrist D Gain", m_wrist_coeff.kD);
    frc::SmartDashboard::PutNumber("Wrist FF Gain", m_wrist_coeff.kFF);
    frc::SmartDashboard::PutNumber("Wrist Max Output", m_wrist_coeff.kMaxOutput);
    frc::SmartDashboard::PutNumber("Wrist Min Output", m_wrist_coeff.kMinOutput);
 
}
 
void Wrist::WristDashRead()
{
    m_wrist_coeff.kP = frc::SmartDashboard::GetNumber("Wrist P Gain", 0.0);
    m_wrist_coeff.kI = frc::SmartDashboard::GetNumber("Wrist I Gain", 0.0);
    m_wrist_coeff.kD = frc::SmartDashboard::GetNumber("Wrist D Gain", 0.0);
    m_wrist_coeff.kFF = frc::SmartDashboard::GetNumber("Wrist FF Gain", 0.0);
    m_wrist_coeff.kMinOutput = frc::SmartDashboard::GetNumber("Wrist Min Output", -1.0);
    m_wrist_coeff.kMaxOutput = frc::SmartDashboard::GetNumber("Wrist Min Output", 1.0);
 
}
#include <iostream>
#include <Wrist.h>
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>


Wrist::Wrist(){

        WristPIDInit();
        WristDashInit();
        //WristDashRead();
        
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

void Wrist::Down()
{
    m_wrist_motor.Set(0.1);
}

void Wrist::Up()
{
    m_wrist_motor.Set(-0.1);
}

void Wrist::Stop()
{
    m_wrist_motor.Set(0.0);
}

void Wrist::Follow(double arm_pos)
{
    //1.02 + -0.0176x + 2.51E-04x^2 + -1.78E-06x^3 + 4.55E-09x^4
    //m_wrist_PIDController.SetReference(1.1, rev::CANSparkMax::ControlType::kPosition);
    double position = 1.02 - (0.0176*arm_pos) + (2.51e-4*pow(arm_pos,2)) - (1.78e-6*pow(arm_pos,3)) + (4.55e-9*pow(arm_pos,4));
    // if (arm_pos < 31) {
    //     position = .848;
    // }
    // else if (arm_pos >= 31 && arm_pos < 65 ){
    //     position = .665;
    // }
    // else if (arm_pos >= 65 && arm_pos < 110){
    //     position = .586;
    // }
    // else if (arm_pos >= 110 && arm_pos < 130){
    //     position = .430;
    // }
    // else if (arm_pos >= 130){
    //     position = .393;
    // }
    m_wrist_PIDController.SetReference(position, rev::CANSparkMax::ControlType::kPosition);
}

void Wrist::Follow_Flare(double arm_pos)
{
    m_wrist_PIDController.SetReference(arm_pos/360 + CONSTANTS::WRIST::WRIST_ENCODER_OFFSET + CONSTANTS::WRIST::WRIST_FLARE_OFFSET, rev::CANSparkMax::ControlType::kPosition);
}

void Wrist::Test(){
    std::cout << "wrist encoder: " << m_wrist_Encoder.GetPosition() << "\n";
}
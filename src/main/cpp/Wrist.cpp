#include <iostream>
#include <Wrist.h>
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>


Wrist::Wrist(){

    WristPIDInit();
        
    m_wrist_Encoder.SetInverted(false);
    m_wrist_PIDController.SetFeedbackDevice(m_wrist_Encoder);

}

Wrist::~Wrist(){}

void Wrist::WristPIDInit() {
 
    m_wrist_PIDController.SetP(m_wrist_coeff.kP);
    m_wrist_PIDController.SetI(m_wrist_coeff.kI);
    m_wrist_PIDController.SetD(m_wrist_coeff.kD);
    m_wrist_PIDController.SetIZone(m_wrist_coeff.kIz);
    m_wrist_PIDController.SetFF(m_wrist_coeff.kFF);
    m_wrist_PIDController.SetOutputRange(m_wrist_coeff.kMinOutput, m_wrist_coeff.kMaxOutput);
    

}

void Wrist::Follow(double arm_pos)
{
    double position = 1.02 - (0.0176*arm_pos) + (2.51e-4*pow(arm_pos,2)) - (1.78e-6*pow(arm_pos,3)) + (4.55e-9*pow(arm_pos,4));
    m_wrist_PIDController.SetReference(position, rev::CANSparkMax::ControlType::kPosition);
}


void Wrist::test(){
    std::cout << "wrist encoder: " << m_wrist_Encoder.GetPosition() << "\n";
}
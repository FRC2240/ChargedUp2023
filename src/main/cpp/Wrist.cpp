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

void Wrist::pickupFollow(double arm_pos)
{
    if (arm_pos < 140.0){
        position = (-3.11e-3*arm_pos) + 1.36;
    }
    else {
        position = (-2.78e-3*arm_pos) + 1.17;
    }
    m_wrist_PIDController.SetReference(position, rev::CANSparkMax::ControlType::kPosition);
}

void Wrist::Follow(double arm_pos)
{
    position = 1.47 - (4.6e-3*arm_pos) + (4.65e-6*pow(arm_pos,2)) - (2.26e-9*pow(arm_pos,3));
    m_wrist_PIDController.SetReference(position, rev::CANSparkMax::ControlType::kPosition);
}


void Wrist::test(){
    std::cout << "wrist encoder: " << m_wrist_Encoder.GetPosition() << "\n";
}
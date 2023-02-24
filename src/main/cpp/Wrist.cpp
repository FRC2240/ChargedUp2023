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
    double position = (-2.78e-3*arm_pos) + 1.16;
    m_wrist_PIDController.SetReference(position, rev::CANSparkMax::ControlType::kPosition);
}

void Wrist::Follow(double arm_pos)
{
    double position = 1.47 - (4.6e-3*arm_pos) + (4.65e-6*pow(arm_pos,2)) - (2.26e-9*pow(arm_pos,3));
    //double position = -225 + (7.49*arm_pos) - (0.102*pow(arm_pos,2)) + (7.29e-4*pow(arm_pos,3)) - (2.89e-6*pow(arm_pos,4)) + (6.03e-9*pow(arm_pos,5)) - (5.17e-12*pow(arm_pos,6));
    //std::cout << position << std::endl;
    m_wrist_PIDController.SetReference(position, rev::CANSparkMax::ControlType::kPosition);
}


void Wrist::test(){
    std::cout << "wrist encoder: " << m_wrist_Encoder.GetPosition() << "\n";
}
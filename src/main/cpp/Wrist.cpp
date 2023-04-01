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

// void Wrist::pickupFollow(double arm_pos)
// {
//     if (arm_pos < 140.0){
//         position = (-3.11e-3*arm_pos) + 1.36;
//     }
//     else {
//         position = (-2.78e-3*arm_pos) + 1.17;
//     }
//     m_wrist_PIDController.SetReference(position, rev::CANSparkMax::ControlType::kPosition);
// }

void Wrist::Follow(double arm_pos)
{
    if (arm_pos < 72.0) {
        //-2.99 + 0.269x + -5.77E-03x^2 + 3.87E-05x^3
         position = -2.99 + (0.269*arm_pos) + (-5.77e-3*pow(arm_pos,2)) + (3.87e-5*pow(arm_pos,3));
     }
     else {
        //-3.1E-03*x + 1.07
        position = -3.1e-3*arm_pos + 1.07;
    }
    m_wrist_PIDController.SetReference(position, rev::CANSparkMax::ControlType::kPosition);
}

void Wrist::HumanPlayer()
{
    m_wrist_PIDController.SetReference(0.51, rev::CANSparkMax::ControlType::kPosition);
}

void Wrist::Pickup()
{
    m_wrist_PIDController.SetReference(CONSTANTS::WRIST::PICKUP, rev::CANSparkMax::ControlType::kPosition);
}


void Wrist::test()
{
    std::cout << "wrist encoder: " << m_wrist_Encoder.GetPosition() << "\n";
}

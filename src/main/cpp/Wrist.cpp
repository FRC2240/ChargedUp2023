#include <iostream>
#include <Wrist.h>
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>


Wrist::Wrist()
{
    wrist_PID_init();
        
    m_wrist_encoder.SetInverted(false);
    m_wrist_PID_controller.SetFeedbackDevice(m_wrist_encoder);
}

Wrist::~Wrist(){}

void Wrist::wrist_PID_init() 
{ 
    m_wrist_PID_controller.SetP(m_wrist_coeff.kP);
    m_wrist_PID_controller.SetI(m_wrist_coeff.kI);
    m_wrist_PID_controller.SetD(m_wrist_coeff.kD);
    m_wrist_PID_controller.SetIZone(m_wrist_coeff.kIz);
    m_wrist_PID_controller.SetFF(m_wrist_coeff.kFF);
    m_wrist_PID_controller.SetOutputRange(m_wrist_coeff.kMinOutput, m_wrist_coeff.kMaxOutput);
}

void Wrist::follow(double arm_pos)
{
    if (arm_pos < 72.0) 
    {
        position = -2.99 + (0.269 * arm_pos) + (-5.77e-3 * pow(arm_pos, 2)) + (3.87e-5 * pow(arm_pos, 3));
    }
     else 
    {
        position = -3.1e-3 * arm_pos + 1.07;
    }
    m_wrist_PID_controller.SetReference(position, rev::CANSparkMax::ControlType::kPosition);
}

void Wrist::human_player()
{
    m_wrist_PID_controller.SetReference(CONSTANTS::WRIST::HP, rev::CANSparkMax::ControlType::kPosition);
}

void Wrist::pickup()
{
    m_wrist_PID_controller.SetReference(CONSTANTS::WRIST::PICKUP, rev::CANSparkMax::ControlType::kPosition);
}


void Wrist::test()
{
    std::cout << "wrist encoder: " << m_wrist_encoder.GetPosition() << "\n";
}

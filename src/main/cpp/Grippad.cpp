#include "Grippad.h"


void Grippad::deploy()
{
    m_grippad_piston_front_right.Set(frc::Solenoid::Value::kForward);
    m_grippad_piston_beta_left.Set(frc::Solenoid::Value::kForward);
    m_grippad_piston_back_right.Set(frc::Solenoid::Value::kForward);
    m_grippad_piston_back_left.Set(frc::Solenoid::Value::kForward);
}

void Grippad::retract()
{
    m_grippad_piston_front_right.Set(frc::Solenoid::Value::kReverse);
    m_grippad_piston_front_left.Set(frc::Solenoid::Value::kReverse);
    m_grippad_piston_back_right.Set(frc::Solenoid::Value::kReverse);
    m_grippad_piston_back_left.Set(frc::Solenoid::Value::kReverse);
}







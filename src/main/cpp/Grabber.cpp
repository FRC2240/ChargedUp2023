#include <iostream>
#include "Grabber.h"


Grabber::Grabber(){}

void Grabber::close()
{
  grabber_status_bool = false;
  m_grabber_piston.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Grabber::open()
{
  grabber_status_bool = true;
  m_grabber_piston.Set(frc::DoubleSolenoid::Value::kForward);
}

bool Grabber::grabber_status()
{
  return grabber_status_bool;
}
bool Grabber::limit_switch()
{
  frc::SmartDashboard::GetBoolean("grabber/valid", m_tof_sensor.IsRangeValid());
  frc::SmartDashboard::PutNumber("grabber/dist", m_tof_sensor.GetRange());
  if (m_tof_sensor.IsRangeValid() && (m_tof_sensor.GetRange() < CONSTANTS::GRABBER::TOF_MAX))
  {
    return true;
  }
else
  {
    return false;
  }
}

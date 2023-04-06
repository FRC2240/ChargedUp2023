#include <iostream>
#include "Grabber.h"


Grabber::Grabber()
{
  m_tof_sensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 33.0);
}

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
  frc::SmartDashboard::PutNumber("grabber/noise", m_tof_sensor.GetAmbientLightLevel());
  frc::SmartDashboard::PutNumber("grabber/sigma", m_tof_sensor.GetRangeSigma());
  if (m_tof_sensor.IsRangeValid())
  {
  return (m_tof_sensor.GetRange() < CONSTANTS::GRABBER::TOF_MAX);
  }

  else 
  {
    return 0;
  }
}
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
  return m_limit_switch.Get();
}
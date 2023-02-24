#include <iostream>
#include "Grabber.h"


Grabber::Grabber(){}

void Grabber::close()
{
  grabberStatusBool = false;
  m_grabberPiston.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Grabber::open()
{
  m_grabberPiston.Set(frc::DoubleSolenoid::Value::kForward);
  grabberStatusBool = true;
}

bool Grabber::grabberStatus()
{
  return grabberStatusBool;
}

bool Grabber::break_beam(){return m_beam.Get();}


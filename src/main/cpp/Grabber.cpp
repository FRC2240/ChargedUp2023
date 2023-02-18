#include <iostream>
#include "Grabber.h"


Grabber::Grabber(){}

void Grabber::close()
{
  m_grabberPiston.Set(frc::DoubleSolenoid::Value::kForward);
}

void Grabber::open()
{
  m_grabberPiston.Set(frc::DoubleSolenoid::Value::kReverse);
}

bool Grabber::break_beam(){return m_beam.Get();}


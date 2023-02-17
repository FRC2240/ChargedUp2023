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

void Grabber::GrabberLogic(bool arm_bool, bool toggle_raw)
{
  if (toggle_raw)
    {
      toggle = !toggle;
    }

  if (arm_bool || toggle)
    {
      open();
    }
  else
    {
      close();
    }
}

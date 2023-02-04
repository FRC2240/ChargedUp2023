#include <iostream>
#include "Grabber.h"


Grabber::Grabber(){}

//FIXME
// Only handles one case
// functions not used
// Other issues
// DO NOT IGNORE WARNINGS FROM THIS FUNCTION
Grabber::STATES Grabber::Logic(
                               bool intake_button,
                               bool extake_button,
                               bool stow_button
                               )
{
  if (stow_button)
    {
      state = STOWED;
    }
  switch (state)
    {
    case STOWED:
      Grabber::Stop();

      if (intake_button)
        {
          Grabber::In();
        }
      else if (extake_button)
        {
          Grabber::Out();
        }

      break;
    }
}
 
 
void Grabber::In()
{
  m_grabberPiston.Set(frc::DoubleSolenoid::Value::kForward);
  m_motor_Grabber.Set(1.0);
}

void Grabber::Out()
{
  m_grabberPiston.Set(frc::DoubleSolenoid::Value::kReverse);
  m_motor_Grabber.Set(-1.0);
}

 void Grabber::Stop()
 {
   m_motor_Grabber.Set(0.0);
 }

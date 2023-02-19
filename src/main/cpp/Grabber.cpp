#include <iostream>
#include "Grabber.h"


Grabber::Grabber(){}

//FIXME
// Only handles one case
// functions not used
// Other issues
// DO NOT IGNORE WARNINGS FROM THIS FUNCTION 
 
void Grabber::In()
{
  m_grabberPiston.Set(frc::DoubleSolenoid::Value::kForward);
}

void Grabber::Out()
{
  m_grabberPiston.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Grabber::GrabberLogic(bool arm_bool)
{

}
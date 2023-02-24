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
  grabberStatusBool = true;
}

void Grabber::Out()
{
  m_grabberPiston.Set(frc::DoubleSolenoid::Value::kReverse);
  grabberStatusBool = false;
}

bool Grabber::grabberStatus()
{
  std::cout << grabberStatusBool;
  return grabberStatusBool;
}

void Grabber::GrabberLogic(bool arm_bool)
{

}
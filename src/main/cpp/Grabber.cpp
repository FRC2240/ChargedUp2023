#include <iostream>
#include "Grabber.h"


Grabber::Grabber(){
  
}
 
Grabber::STATES Grabber::Logic(
                               bool intake_button,
                               bool extake_button
                            ) {      

   switch (state)
   {
   case STOWED:
   Grabber::Stop();
   
      break;
   case INTAKING:
   Grabber::In();
      break;

   case EXTAKING:
   Grabber::Out();
      break;
   }  
}     
 
 
 void Grabber::In(){

m_grabberPiston.Set(frc::DoubleSolenoid::Value::kForward);
m_motor_Grabber.Set(1.0);
}

 void Grabber::Out(){

m_grabberPiston.Set(frc::DoubleSolenoid::Value::kReverse);
m_motor_Grabber.Set(-1.0);
}
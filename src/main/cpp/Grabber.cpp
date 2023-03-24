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

bool Grabber::ReadSensors() {
  //static int count = 0;

  //++count;

  // Only read the I2C data once per 10 loops
  // However, force a read if it's a toggle to update the ball states
  //if (count < 10) {
  //  return;
  //}

  // Read data
  auto rightColor  = m_rightSensor.GetColor();
  auto leftColor = m_leftSensor.GetColor();
  std::cout << "r = " << rightColor.red << " " << rightColor.green << " " << rightColor.blue << "\n";
  std::cout << "l = " << leftColor.red << " " << leftColor.green << " " << leftColor.blue << "\n";

  //count = 0;

  return (Color(rightColor) || Color(leftColor));
}

// Measurements from REV Color Sensors:
// RED  BALL: R (0.40 - 0.54),  G (0.34 - 0.40), B (0.12 - 0.20)
// BLUE BALL: R (0.14 - 0.20),  G (0.39 - 0.44), B (0.36 - 0.47)
//   NO BALL: R (0.25), G (0.47), B (0.28)
bool Grabber::Color(frc::Color color) {
  // match cone
  if ((color.red > 0.37) && (color.blue < 0.23)) {
    return true;
  }

  // match cube
  if ((color.red < 0.23) && (color.blue > 0.33)) {
    return true;
  }

  return false;
}

#ifndef GRABBER_H_
#define GRABBER_H_

#include "rev/CANSparkMax.h"
#include <frc/Encoder.h>
#include "Constants.h"
#include <iostream>
#include <frc/Solenoid.h>
#include <frc/Compressor.h>

class Grabber 
{

public:
  Grabber();
  ~Grabber();

private:
  //Only needs one motor
  rev::CANSparkMax m_motor_wrist{CONSTANTS::GRABBER::WRIST_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};
};
#endif //GRABBER_H_

#ifndef GRIPAD_H_
#define GRIPAD_H_

#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include "Constants.h"

class Grippad
{

 public:

  Grippad(){};

  ~Grippad(){};

  void deploy();

  void retract();

private:

  // frc::Compressor phCompressor{1, frc::PneumaticsModuleType::REVPH};

  //Grippad has four pistons operating on one solenoid

  frc::DoubleSolenoid m_grippad_pistons{
    frc::PneumaticsModuleType::REVPH,CONSTANTS::GRIPPAD::GRIPPAD_PISTON_ID1,
    CONSTANTS::GRIPPAD::GRIPPAD_PISTON_ID2
    };

};
#endif // GRIPAD_H_

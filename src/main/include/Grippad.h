#ifndef GRIPAD_H_
#define GRIPAD_H_

#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include "Constants.h"

class Grippad
{

 public:

         Grippad(){};

         ~Grippad(){};

         void deploy();

         void retract();

 private:

  frc::Compressor phCompressor{1, frc::PneumaticsModuleType::REVPH};

  //Grippad has four pistons operating on one solenoid
  frc::Solenoid m_grippad_pistons{frc::PneumaticsModuleType::REVPH,
                                  CONSTANTS::GRIPPAD::GRIPPAD_CHANNEL};

};
#endif // GRIPAD_H_

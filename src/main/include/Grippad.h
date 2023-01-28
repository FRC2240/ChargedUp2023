#ifndef GRIPAD_H_
#define GRIPAD_H_

#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include "Constants.h"

class Grippad
{

 public:

         Grippad();

         ~Grippad();

         void deploy();

         void retract();


 private:
        //Grippad has four pistons
         frc::Solenoid m_grippad_piston_front_right{frc::PneumaticsModuleType::CTREPCM,
             CONSTANTS::GRIPPAD::FRONT_RIGHT_CHANNEL};
         frc::Solenoid m_grippad_piston_front_left{frc::PneumaticsModuleType::CTREPCM,
         CONSTANTS::GRIPPAD::FRONT_LEFT_CHANNEL};
         frc::Solenoid m_grippad_piston_back_right{frc::PneumaticsModuleType::CTREPCM,
         CONSTANTS::GRIPPAD::BACK_RIGHT_CHANNEL};
         frc::Solenoid m_grippad_piston_back_left{frc::PneumaticsModuleType::CTREPCM,
         CONSTANTS::GRIPPAD::BACK_LEFT_CHANNEL};
};
#endif // GRIPAD_H_

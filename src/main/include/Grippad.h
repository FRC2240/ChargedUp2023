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
         frc::Solenoid m_grippad_piston_alpha{frc::PneumaticsModuleType::CTREPCM,
             CONSTANTS::GRIPPAD::ALPHA_CHANNEL};
         frc::Solenoid m_grippad_piston_beta{frc::PneumaticsModuleType::CTREPCM,
         CONSTANTS::GRIPPAD::BETA_CHANNEL};
         frc::Solenoid m_grippad_piston_gamma{frc::PneumaticsModuleType::CTREPCM,
         CONSTANTS::GRIPPAD::GAMMA_CHANNEL};
         frc::Solenoid m_grippad_piston_delta{frc::PneumaticsModuleType::CTREPCM,
         CONSTANTS::GRIPPAD::DELTA_CHANNEL};
};
#endif // GRIPAD_H_

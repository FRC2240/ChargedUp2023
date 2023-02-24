#include "Grippad.h"


// frc::Compressor enableCompressorDigital();

void Grippad::deploy()
{
   m_grippad_pistons.Set(false);
}

void Grippad::retract()
{
   m_grippad_pistons.Set(true);
}


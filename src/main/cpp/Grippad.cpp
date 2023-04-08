#include "Grippad.h"


// frc::Compressor enableCompressorDigital();

void Grippad::deploy()
{
   m_grippad_pistons.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Grippad::retract()
{
   m_grippad_pistons.Set(frc::DoubleSolenoid::Value::kForward);
}


#include "Grippad.h"

void Grippad::deploy()
{
   m_grippad_pistons.Set(false);
}

void Grippad::retract()
{
   m_grippad_pistons.Set(true);
}


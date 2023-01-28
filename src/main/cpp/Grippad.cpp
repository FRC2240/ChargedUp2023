#include "Grippad.h"


void Grippad::deploy()
{

    //m_grippad_piston_front_right.Set(0.1);
    m_grippad_piston_front_right.Set(true);
    // m_grippad_piston_back_left.Set(0.1);
    // m_grippad_piston_back_right.Set(0.1);
    // m_grippad_piston_back_left.Set(0.1);
}

void Grippad::retract()
{
    m_grippad_piston_front_right.Set(false);
    // m_grippad_piston_front_left.Set(-0.1);
    // m_grippad_piston_back_right.Set(-0.1);
    // m_grippad_piston_back_left.Set(-0.1);
}


#ifndef BUTTONS_H_
#define BUTTONS_H_

#include <frc/XboxController.h>

namespace BUTTON
{
  inline frc::XboxController m_stick{0};

  // Aux means auxiallry, like second
  inline frc::XboxController m_aux_stick{1};

  namespace  DRIVETRAIN
  {
    inline double LX() {return BUTTON::m_stick.GetLeftX();}
    inline double LY() {return BUTTON::m_stick.GetLeftY();}
    inline double RX() {return BUTTON::m_stick.GetRightX();}
    inline double RY() {return BUTTON::m_stick.GetRightY();}
    inline double FIELD_CENTRIC(){return BUTTON::m_stick.GetRightStickButton();}
  }
}
#endif //BUTTONS_H_

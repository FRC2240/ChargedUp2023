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

  namespace ARM
  {
    inline bool ARM_STORED() {return BUTTON::m_stick.GetAButtonReleased();}
    inline bool ARM_GROUND() {return BUTTON::m_stick.GetXButtonReleased();}
    inline bool ARM_MID() {return BUTTON::m_stick.GetYButtonReleased();}
    inline bool ARM_HIGH() {return BUTTON::m_stick.GetBButtonReleased();}
    inline bool ARM_HP() {return BUTTON::m_stick.GetLeftStickButtonReleased();}
  }

  namespace GRABBER
  {
    inline bool GRABBER_TOGGLE() {return BUTTON::m_stick.GetRightBumperReleased();}
    inline bool GRABBER_STORE() {return BUTTON::m_stick.GetLeftBumperReleased();}
    
  }

  namespace GRIPPADS
  {
    inline bool GRIPPADS_DEPLOY() 
    {
      if (BUTTON::m_stick.GetPOV() <= 225 && BUTTON::m_stick.GetPOV() >= 135)
      {
        return true;
      }
    else
      {
      return false;
      }

    }
  }
}
#endif //BUTTONS_H_

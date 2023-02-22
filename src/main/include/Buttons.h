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
    inline bool ARM_STORED() {return BUTTON::m_stick.GetLeftBumper();}
    inline bool ARM_PICKUP() {return BUTTON::m_stick.GetPOV() == 270;}
    inline bool ARM_LOW() {return BUTTON::m_stick.GetAButton();}
    inline bool ARM_MID() {return BUTTON::m_stick.GetBButton();}
    inline bool ARM_HIGH() {return BUTTON::m_stick.GetYButton();}
    inline bool ARM_HP() {return BUTTON::m_stick.GetXButtonPressed();}
  }

  namespace GRABBER
  {
    inline bool TOGGLE() {return BUTTON::m_stick.GetRightBumperReleased();}
    //inline bool GRABBER_STORE() {return BUTTON::m_stick.GetLeftBumperReleased();}
    
  }

  namespace GRIPPADS
  {
    inline bool GRIPPADS_DEPLOY() {return BUTTON::m_stick.GetPOV() <= 225 && BUTTON::m_stick.GetPOV() >= 135;}
    inline bool GRIPPADS_RETRACT() {return BUTTON::m_stick.GetPOV() == 0;}
    }
  
  namespace CANDLE
  {
    inline bool CANDLE_LEFT() {return BUTTON::m_aux_stick.GetBButton();}
    inline bool CANDLE_RIGHT() {return BUTTON::m_aux_stick.GetXButton();}
    inline bool CANDLE_YELLOW() {return BUTTON::m_aux_stick.GetYButton();}
    inline bool CANDLE_PURPLE() {return BUTTON::m_aux_stick.GetAButton();}
  }
}

#endif //BUTTONS_H_

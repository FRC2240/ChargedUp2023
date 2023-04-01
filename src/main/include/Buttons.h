#ifndef BUTTONS_H_
#define BUTTONS_H_

#include <frc/XboxController.h>
#include <frc/MathUtil.h>
#include "Constants.h"

namespace BUTTON
{
  inline frc::XboxController m_stick{0};

  // Aux means auxiallry, like second
  inline frc::XboxController m_aux_stick{1};

  namespace  DRIVETRAIN
  {
    inline bool TURBO() {return BUTTON::m_stick.GetLeftTriggerAxis() > 0.5;}
    inline double ZERO() {return BUTTON::m_stick.GetStartButtonPressed();}
    inline double LX() {return BUTTON::m_stick.GetLeftX();}
    inline double LY() {return BUTTON::m_stick.GetLeftY();}
    inline double RX() {return BUTTON::m_stick.GetRightX();}
    inline double RY() {return BUTTON::m_stick.GetRightY();}
    inline double FIELD_CENTRIC(){return 0;}

   inline bool ABORT()
    {
      if 
        (
         frc::ApplyDeadband(BUTTON::DRIVETRAIN::LX(), CONSTANTS::DEADBAND) ||
         frc::ApplyDeadband(BUTTON::DRIVETRAIN::LY(), CONSTANTS::DEADBAND) ||
         frc::ApplyDeadband(BUTTON::DRIVETRAIN::RX(), CONSTANTS::DEADBAND) ||
         frc::ApplyDeadband(BUTTON::DRIVETRAIN::RY(), CONSTANTS::DEADBAND)
         )
        {
          return true;
        }
      else 
        {
          return false;
        }
    } 
  }

  namespace ARM
  {
    inline bool ARM_STORED() {return BUTTON::m_stick.GetLeftBumper();}
    inline bool ARM_PICKUP() {return BUTTON::m_stick.GetPOV() == 270;}
    inline bool ARM_LOW() {return BUTTON::m_stick.GetAButtonPressed();}
    inline bool ARM_MID() {return BUTTON::m_stick.GetBButtonPressed();}
    inline bool ARM_HIGH() {return BUTTON::m_stick.GetYButtonPressed();}
    inline bool ARM_HP() {return BUTTON::m_stick.GetXButtonPressed();}
    inline bool TRIGGER_AUTO()
    {
      if (BUTTON::m_stick.GetRightTriggerAxis() > 0.4)
        {
          return true;
        }
      else
        {
          return false;
        }
    }

    namespace OVERIDES
    {
      inline bool ARM_OVERIDE_UP() {return BUTTON::m_aux_stick.GetPOV() == 0;}
      inline bool ARM_OVERIDE_OPEN() {return BUTTON::m_aux_stick.GetLeftBumperPressed();}
      inline bool PICKUP_OVERIDE() {
      if (BUTTON::m_aux_stick.GetRightTriggerAxis() > 0.4)
        {
          return true;
        }
      else
        {
          return false;
        }
    }
    inline bool HP_OVERIDE() {
      if (BUTTON::m_aux_stick.GetLeftTriggerAxis() > 0.4)
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

  namespace GRABBER
  {
    inline bool TOGGLE() {return BUTTON::m_stick.GetRightBumper();}
    inline bool OVERIDE_TOGGLE() {return BUTTON::m_stick.GetRightBumper();}
  }

  namespace GRIPPADS
  {
    inline bool GRIPPADS_DEPLOY() {return BUTTON::m_stick.GetPOV() == 180;}
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

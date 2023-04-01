#include <rev/CANSparkMax.h>
#include "frc/DigitalInput.h"
#include <frc/Timer.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <Constants.h>
#include <frc/DoubleSolenoid.h>

class Grabber {
private:  enum STATES {STOWED, INTAKING, EXTAKING};

public:

  Grabber();

  bool limit_switch();
  bool break_beam();
  void open();
  void close();

  void GrabberLogic(bool arm_bool, bool toggle_raw);

  STATES Logic(bool intake_button, bool extake_button, bool stow_button);

  bool grabberStatus();

  bool grabberStatusBool = false;

  frc::DigitalInput  m_limit_switch{0};
  frc::DigitalInput m_beam{1};
private:

  bool toggle = false;
  STATES state = STOWED;

  frc::DoubleSolenoid m_grabberPiston{frc::PneumaticsModuleType::REVPH,CONSTANTS::GRABBER::GRABBER_PISTON_ID1,CONSTANTS::GRABBER::GRABBER_PISTON_ID2};

  

  
};

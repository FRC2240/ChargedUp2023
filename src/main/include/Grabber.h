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

  void In();
  void Out();
  bool grabberToggle = false;
  

  STATES Logic(bool intake_button, bool extake_button, bool stow_button);


private:

  STATES state = STOWED;

  frc::DoubleSolenoid m_grabberPiston{frc::PneumaticsModuleType::REVPH,CONSTANTS::GRABBER::GRABBER_PISTON_ID1,CONSTANTS::GRABBER::GRABBER_PISTON_ID2};

  
};

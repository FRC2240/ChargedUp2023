#include <rev/CANSparkMax.h>
#include "frc/DigitalInput.h"
#include <frc/Timer.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <Constants.h>
#include <frc/DoubleSolenoid.h>

class Grabber {
private:  enum STATES {STOWED, INTAKING,EXTAKING};

public:

  Grabber();

  void Stop();
  void In();
  void Out();
  

  STATES Logic(bool intake_button, bool extake_button);

private:

  STATES state = STOWED;

  rev::CANSparkMax m_motor_Grabber{CONSTANTS::GRABBER::GRABBER_MOTOR_ID,rev::CANSparkMax::MotorType::kBrushless};
  frc::DoubleSolenoid m_grabberPiston{frc::PneumaticsModuleType::CTREPCM,CONSTANTS::GRABBER::GRABBER_PISTON_ID,CONSTANTS::GRABBER::GRABBER_PISTON_ID};

};

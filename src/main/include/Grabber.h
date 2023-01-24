#include <rev/CANSparkMax.h>
#include "frc/DigitalInput.h"
#include <frc/Timer.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <Constants.h>
#include <frc/DoubleSolenoid.h>

class Grabber {
private:  enum STATES {STOWED, INTAKING, INTAKE_WAIT, INTAKE_STOP, EXTAKING, OVERIDE_WAIT};

public:

  Grabber();

  void Stop();
  void In();
  void Out();
  bool grabberToggle = false;

  STATES Logic(bool intake_button, bool extake_button, bool stow_button);

  rev::SparkMaxRelativeEncoder m_encoder = m_motor_Grabber.GetEncoder();
private:

  STATES state = STOWED;

  rev::CANSparkMax m_motor_Grabber{CONSTANTS::GRABBER::GRABBER_MOTOR_ID,rev::CANSparkMax::MotorType::kBrushless};
  frc::DoubleSolenoid m_grabberPiston{frc::PneumaticsModuleType::CTREPCM,CONSTANTS::GRABBER::GRABBER_PISTON_ID,CONSTANTS::GRABBER::GRABBER_PISTON_ID};

};

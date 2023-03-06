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

  bool break_beam();
  void open();
  void close();
  bool grabber_status();

private:
  frc::DigitalInput  m_beam{0};

  bool grabber_status_bool = false;

  frc::DoubleSolenoid m_grabber_piston{frc::PneumaticsModuleType::REVPH,CONSTANTS::GRABBER::GRABBER_PISTON_ID1,CONSTANTS::GRABBER::GRABBER_PISTON_ID2};
};

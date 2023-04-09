#include <rev/CANSparkMax.h>
#include "frc/DigitalInput.h"
#include <frc/Timer.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <Constants.h>
#include <frc/DoubleSolenoid.h>
#include <TimeOfFlight.h>

class Grabber {
private:  enum STATES {STOWED, INTAKING, EXTAKING};

public:

  Grabber();

  bool limit_switch();
  void open();
  void close();

  void set_sensor(bool triggered);

  bool grabber_status();

  bool grabber_status_bool = false;

  frc::DigitalInput  m_limit_switch{0};
  frc::DigitalInput m_beam{1};
private:

  bool m_sensor_overide = true;
  bool toggle = false;
  STATES state = STOWED;

  frc::TimeOfFlight m_tof_sensor{CONSTANTS::GRABBER::TOF_CAN};
  frc::DoubleSolenoid m_grabber_piston{
    frc::PneumaticsModuleType::REVPH,CONSTANTS::GRABBER::GRABBER_PISTON_ID1,
    CONSTANTS::GRABBER::GRABBER_PISTON_ID2
    };
};

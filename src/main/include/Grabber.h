#include <rev/CANSparkMax.h>
#include "frc/DigitalInput.h"
#include <frc/Timer.h>
#include "frc/smartdashboard/SmartDashboard.h"

class Grabber {
private:  enum STATES {STOWED, INTAKING, INTAKE_WAIT, INTAKE_STOP, EXTAKING, OVERIDE_WAIT};

public:

  Grabber();

  void Stop();
  void In();
  void Out();
  

  STATES Logic(bool intake_button, bool extake_button, bool store_button, bool ignore_button);

private:
};

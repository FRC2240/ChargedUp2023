#ifndef DASH_H
#define DASH_H

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

namespace DASHBOARD {
  void update_botpose(double x)
    {
 frc::Shuffleboard::GetTab("Tomfoolery")
  .Add("botpose", x)
  .GetEntry();
  }
}


#endif // DASH_H

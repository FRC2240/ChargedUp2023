#pragma once

#include <vector>
#include <list>
#include <frc/TimedRobot.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/angle.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>
#include "Vision.h"
#include <iostream>

// more libraries more better
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/trajectory/TrajectoryGenerator.h>
#include "Buttons.h"
#include "Dash.h"
#include "Grabber.h"
#include "Grippad.h"
#include "Candle.h"
#include "Wrist.h"
#include "Arm.h"



#define m_deadband 0.15

class Robot : public frc::TimedRobot
{
public:
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    Robot();

    void RobotInit() override;
    void RobotPeriodic() override;
    
    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    // void DisabledInit() override;
    // void DisabledPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

    void make_test_path();



private:
    //Vision m_camera;

    Arm m_arm;
    int m_cam_counter = 0;

    frc::Trajectory m_trajectory;

    frc::SendableChooser<std::string> m_chooser;
    const std::string LINE = "Line";
    const std::string CIRCLE = "Circle";
    const std::string NON_HOLONOMIC = "Non holonomic";
    const std::string TEST = "Test";

    std::string m_autoSelected;
    bool breakbeam;

    Grabber m_grabber;
    Grippad m_grippad;
    Candle m_candle;
    Wrist m_wrist;
    

    enum autoActions {
        kPickup,
        kDrop,
        k2Piece,
        kTestPath,
        kTerminalPath1,
        kTerminalPath2,
        kTerminalPath3,
        kTerminalPath4,
        kInstantPath,
        kDelayPath,
        kIdle
  };

      enum autoState {
    kDriving,
    kNothing
  };

  std::list<autoActions> *m_autoSequence; 
  std::list<autoActions> m_testSequence{
    kPickup,
    kTestPath,
    kDrop,
    kIdle
  };

autoActions m_autoAction;
autoState m_autoState;

};

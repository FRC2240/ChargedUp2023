#pragma once

#include <list>
#include "Trajectory.hpp"
#include "Vision.h"
#include "Buttons.h"
#include "Dash.h"
#include "Grabber.h"
#include "Grippad.h"
#include "Candle.h"
#include "Wrist.h"
#include "Arm.h"
#include "Constants.h"
#include "Drivetrain.hpp"
#include "RobotState.hpp"
#include "ngr.hpp"
#include "Odometry.hpp"

#include <frc/TimedRobot.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/angle.h>
#include <vector>
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
#include <frc/MathUtil.h>
#include <iostream>
#include <fmt/format.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>


#include "Constants.h"


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
     void DisabledPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

    void traj_fall_back();
    void traj_init(Trajectory::HEIGHT h);
    void make_test_path();



private:

    CONSTANTS::STATES state = CONSTANTS::STATES::STORED;

    bool fall_back_init = false;
    Trajectory::HEIGHT db_last_tgt = Trajectory::HEIGHT::SAFE;

    bool m_is_auto = false;
    std::vector<double> m_test_case = {1,2,3,4,5};
    Vision m_camera;
    int m_cycle = 0;

    Arm m_arm;
    //frc::Trajectory m_trajectory;

    frc::SendableChooser<std::string> m_chooser;
    const std::string LINE = "Line";
    const std::string CIRCLE = "Circle";
    const std::string NON_HOLONOMIC = "Non holonomic";
    const std::string TEST = "Test";

    bool arm_bool;

    
    frc::Timer m_robot_timer;

    std::string m_autoSelected;
    bool breakbeam;

    Grabber m_grabber;
    Grippad m_grippad;
    Candle m_candle;
    Wrist m_wrist;
    pathplanner::PathPlannerTrajectory m_trajectory;
        pathplanner::PathPlannerTrajectory m_back_trajectory;

    

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

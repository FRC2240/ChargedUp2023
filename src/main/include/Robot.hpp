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
#include "autoBalance.h"
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
#include <list>
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
    double getDistance();

private:

    CONSTANTS::STATES state; //= CONSTANTS::STATES::STORED;
    CONSTANTS::STATES last_state;

    bool fall_back_init = false;
    Trajectory::HEIGHT db_last_tgt = Trajectory::HEIGHT::SAFE;

    bool m_is_auto = false;
    std::vector<double> m_test_case = {1,2,3,4,5};
    Vision m_camera;
    int m_cycle = 0;

    Arm m_arm;
    //frc::Trajectory m_trajectory;

    frc::SendableChooser<std::string> m_chooser;
    const std::string AUTO_STATION = "SCORE + STATION";
    const std::string AUTO_LINE = "SCORE + LEAVE";
    const std::string AUTO_NOTHING = "DO NOTHING";
    const std::string HP_LINK = "HUMANPLAYER LINK";
    const std::string HP_CONE = "HUMANPLAYER CONES ONLY";
    const std::string CS = "CHARGE STATION 2 PIECES";
    const std::string CABLE_LINK = "CABLE BUMP LINK";
    const std::string CABLE_CONE = "CABLE BUMP CONES ONLY";

    bool arm_bool;

    double m_force_pos;

    double speed;

    
    frc::Timer m_robot_timer;
    frc::Timer m_robot_timer2;
    
    std::string m_autoSelected;
    bool breakbeam;


    Grabber m_grabber;
    Grippad m_grippad;
    Candle m_candle;
    Wrist m_wrist;
    autoBalance m_auto_balance;
    pathplanner::PathPlannerTrajectory m_trajectory;
    pathplanner::PathPlannerTrajectory m_simp_trajectory;
    pathplanner::PathPlannerTrajectory m_back_trajectory;
    pathplanner::PathPlannerTrajectory m_humanplayer_traj;
//    pathplanner::PathPlannerTrajectory m_path_trajectory;

    /*
      The reason all of these are different is because there is a concern about
      pathplanner not erasing paths on initalization and bits of old path
      getting mixed in with the new path, generaly being a bother.
    */
    pathplanner::PathPlannerTrajectory m_path_trajectory0;
    pathplanner::PathPlannerTrajectory m_path_trajectory1;
    pathplanner::PathPlannerTrajectory m_path_trajectory2;
    pathplanner::PathPlannerTrajectory m_path_trajectory3;
    pathplanner::PathPlannerTrajectory m_path_trajectory4;
    units::meter_t m_fallback_pos;
    units::meter_t m_fallback_pos2;

    enum autoActions{
        kIntake,
        kDelay,
        kScore,
        kScore_periodic,
        kBalance,
        kCSPath1,
        kCSPath2,
        kCSPath3,
        kCableConePath1,
        kCableConePath2,
        kCableConePath3,
        kCableConePath4,
        kHPConePath1,
        kHPConePath2,
        kHPConePath3,
        kHPConePath4,

        kCableLinkPath1,
        kCableLinkPath2,
        kCableLinkPath3,
        kCableLinkPath4,

        kCableLinkPath1_periodic,
        kCableLinkPath2_periodic,
        kCableLinkPath3_periodic,
        kCableLinkPath4_periodic,

        kHPLinkPath1_periodic,
        kHPLinkPath2_periodic,
        kHPLinkPath3_periodic,
        kHPLinkPath4_periodic,

        kHPLinkPath1,
        kHPLinkPath2,
        kHPLinkPath3,
        kHPLinkPath4,
        kIdle,

        kAutoFallback,
    };

    enum autoState {
        kIntaking,
        kBalancing,
        kNothing
    };

    std::list<autoActions> m_HP_link_sequence{
        kScore,
        kHPLinkPath1,
        kDelay,
        kHPLinkPath2,
        kScore,
        kIdle
    };

    std::list<autoActions> *m_autoSequence;
    autoActions m_autoAction;
    autoState m_autoState;
};

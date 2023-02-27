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

    bool arm_bool;

    double m_force_pos;

    
    frc::Timer m_robot_timer;
    frc::Timer m_robot_timer2;

    std::string m_autoSelected;
    bool breakbeam;

    Grabber m_grabber;
    Grippad m_grippad;
    Candle m_candle;
    Wrist m_wrist;
    pathplanner::PathPlannerTrajectory m_trajectory;
    pathplanner::PathPlannerTrajectory m_back_trajectory;

    units::meter_t m_fallback_pos;
};

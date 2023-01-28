#pragma once

#include <frc/TimedRobot.h>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>
#include "Vision.h"

// more libraries more better
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/trajectory/TrajectoryGenerator.h>
#include "Buttons.h"
#include "Dash.h"
#include "Grabber.h"
#include "Grippad.h"



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



private:
    //Vision m_camera;

    int m_cam_counter = 0;

    frc::Trajectory m_trajectory;

    frc::SendableChooser<std::string> m_chooser;
    const std::string LINE = "Line";
    const std::string CIRCLE = "Circle";
    const std::string NON_HOLONOMIC = "Non holonomic";

    std::string m_autoSelected;
    Grabber m_grabber;
    Grippad m_grippad;
};

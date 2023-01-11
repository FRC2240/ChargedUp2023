#pragma once

#include <functional>

// allows functions outside of robot to know if the robot is enabled and more
namespace RobotState
{
    /******************************************************************/
    /*                   Public Variable Definitions                  */
    /******************************************************************/

    inline std::function<bool()> IsEnabled;
    inline std::function<bool()> IsDisabled;
    inline std::function<bool()> IsAutonomous;
    inline std::function<bool()> IsAutonomousEnabled;
    inline std::function<bool()> IsTeleop;
    inline std::function<bool()> IsTeleopEnabled;
    inline std::function<bool()> IsTest;

}
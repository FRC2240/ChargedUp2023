#pragma once

#include "Drivetrain.hpp"

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPoint.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include "Constants.h"
#include <functional>
#include <cmath>
#include "Drivetrain.hpp"
#include "RobotState.hpp"
#include "ngr.hpp"
#include "Odometry.hpp"

#include <frc/controller/HolonomicDriveController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <chrono>
#include <thread>
#include <frc/Timer.h>

using namespace pathplanner; // PathPlanner keeps everything hidden behind 2 sets of namespaces so it's safe to remove the first layer

namespace Trajectory
{
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    enum class HEIGHT {HIGH, MED, GROUND, SAFE};
    enum BIG_TABLE
        {
            LEFT_1,
            LEFT_2,
            LEFT_3,

            CENTER_1,
            CENTER_2,
            CENTER_3,

            RIGHT_1,
            RIGHT_2,
            RIGHT_3
    };
    enum LCR {LEFT, CENTER, /*"co-op"*/ RIGHT};
    enum PIECE {CONE, CUBE}; //"cube"

    struct Target
    {
        HEIGHT height;
        BIG_TABLE table;
        PIECE piece;
    };

    struct TrajDepends
    {
        units::meter_t current_x;
        units::meter_t current_y;
        units::degree_t current_head;
        units::degree_t current_rot;
        units::meter_t desired_x;
        units::meter_t desired_y;
        units::degree_t desired_head;
        units::degree_t desired_rot;
    };

    TrajDepends balance();

    TrajDepends fall_back(bool is_auto = false);

    units::meter_t determine_desired_y();

    TrajDepends determine_desired_traj(HEIGHT h);

    PathPlannerTrajectory generate_live_traj(TrajDepends t);

    //frc::Timer m_trajTimer;

    PathPlannerTrajectory generate_live_traj(units::meter_t current_x,
                                             units::meter_t current_y,
                                             frc::Rotation2d current_head,
                                             frc::Rotation2d current_rot,
                                             units::meter_t desired_x,
                                             units::meter_t desired_y,
                                             frc::Rotation2d desired_head,
                                             frc::Rotation2d desired_rot
                                             );

    PathPlannerTrajectory generate_live_traj(units::meter_t current_x,
                                             units::meter_t current_y,
                                             units::degree_t current_head,
                                             units::degree_t current_rot,
                                             units::meter_t desired_x,
                                             units::meter_t desired_y,
                                             units::degree_t desired_head,
                                             units::degree_t desired_rot
                                             );

    void init_live_traj(PathPlannerTrajectory traj, units::second_t offset = 0.0_s);

    bool follow_live_traj(PathPlannerTrajectory traj);


    void printRobotRelativeSpeeds();

    void printFieldRelativeSpeeds();

    void driveToState(PathPlannerTrajectory::PathPlannerState const &state);

    void follow(std::string const &traj_dir,
                std::function<void(units::second_t time)> const &periodic = nullptr,
                units::meters_per_second_t const &max_vel = Drivetrain::TRAJ_MAX_SPEED,
                units::meters_per_second_squared_t const &max_accl = Drivetrain::TRAJ_MAX_ACCELERATION);

    void testHolonomic(frc::Pose2d const &target_pose,
                       units::velocity::meters_per_second_t const &velocity,
                       frc::Rotation2d const &target_rot);

    /******************************************************************/
    /*                        Public Variables                        */
    /******************************************************************/
    inline bool reverse_trajectory = false;
}

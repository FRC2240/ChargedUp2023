#pragma once

#include "Drivetrain.hpp"

#include <pathplanner/lib/PathPlanner.h>

#include <frc/kinematics/ChassisSpeeds.h>

#include <functional>

using namespace pathplanner; // PathPlanner keeps everything hidden behind 2 sets of namespaces so it's safe to remove the first layer

namespace Trajectory
{
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

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
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

#include <functional>

using namespace pathplanner; // PathPlanner keeps everything hidden behind 2 sets of namespaces so it's safe to remove the first layer

namespace Trajectory
{
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    PathPlannerTrajectory generate_live_traj(units::meter_t current_x,
                                             units::meter_t current_y,
                                             units::degree_t current_head,
                                             units::degree_t current_rot,
                                             units::meter_t desired_x,
                                             units::meter_t desired_y,
                                             units::degree_t desired_head,
                                             units::degree_t desired_rot
                                             );


    void follow_live_traj(PathPlannerTrajectory traj);

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

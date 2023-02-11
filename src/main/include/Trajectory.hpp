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

using namespace pathplanner; // PathPlanner keeps everything hidden behind 2 sets of namespaces so it's safe to remove the first layer

namespace Trajectory
{
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    enum HEIGHT {HIGH, MED, GROUND};
    enum BIG_TABLE
        {
            // Copyright Westly Miller, 2023.
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

    TrajDepends determine_desired_traj(Target tgt);

    PathPlannerTrajectory generate_live_traj(TrajDepends t);


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


void follow_live_traj(PathPlannerTrajectory traj,
                      std::function<void(units::second_t time)> const &periodic = nullptr,
                      units::meters_per_second_t const &max_vel = Drivetrain::TRAJ_MAX_SPEED,
                      units::meters_per_second_squared_t const &max_accl = Drivetrain::TRAJ_MAX_ACCELERATION);


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

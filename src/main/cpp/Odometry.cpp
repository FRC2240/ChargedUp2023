#include "Odometry.hpp"
#include "SwerveModule.hpp"
#include "Drivetrain.hpp"
#include "ngr.hpp"

#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/SmartDashboard.h>

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

extern frc::SwerveDriveKinematics<4> const kinematics;

static frc::SwerveDriveOdometry<4> odometry{
    kinematics,
    frc::Rotation2d{0_deg},
    wpi::array<frc::SwerveModulePosition, 4>{
        frc::SwerveModulePosition{},
        frc::SwerveModulePosition{},
        frc::SwerveModulePosition{},
        frc::SwerveModulePosition{}
        }
    };

frc::Field2d field2d;

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Odometry::putField2d()
{
    frc::SmartDashboard::PutData("Odometry Field", &field2d);
}

void Odometry::update()
{
    frc::Pose2d const pose = odometry.Update(Drivetrain::getCCWHeading(),
                                             Drivetrain::getModulePositions());
    if constexpr (debugging)
        frc::SmartDashboard::PutString("Odometry: ", fmt::format("Pose X: {}, Y: {}, Z (Degrees): {}\n", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()));
}

frc::Pose2d Odometry::getPose() { return odometry.GetPose(); }

frc::ChassisSpeeds const Odometry::getFieldRelativeSpeeds()
{
    // Init for first time
    static frc::Timer speed_timer;
    speed_timer.Start();
    static frc::Pose2d previous_pose{};

    frc::Pose2d const current_pose = odometry.GetPose();

    frc::Pose2d const delta_pose = current_pose.RelativeTo(previous_pose);

    auto const time_elapsed = speed_timer.Get();
    units::meters_per_second_t const X = delta_pose.X() / time_elapsed;

    units::meters_per_second_t const Y = delta_pose.Y() / time_elapsed;

    units::degrees_per_second_t const rot{delta_pose.Rotation().Degrees() / time_elapsed};

    previous_pose = odometry.GetPose(); // Set the previous_pose for the next time this loop is run

    speed_timer.Reset(); // Time how long until next call

    return frc::ChassisSpeeds{X, Y, rot};
}

void Odometry::resetPosition(const frc::Pose2d &pose, const frc::Rotation2d &gyro_angle)
{
    odometry.ResetPosition(gyro_angle, Drivetrain::getModulePositions(), pose);
}

frc::FieldObject2d *Odometry::getField2dObject(std::string_view name)
{
    return field2d.GetObject(name);
}
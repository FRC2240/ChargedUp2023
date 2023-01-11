#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Pose2d.h>

namespace Odometry
{
    void putField2d();

    [[nodiscard]] frc::Pose2d getPose();

    void update();

    void resetPosition(const frc::Pose2d &pose, const frc::Rotation2d &gyroAngle);

    [[nodiscard]] frc::FieldObject2d *getField2dObject(std::string_view name);

    [[nodiscard]] frc::ChassisSpeeds const getFieldRelativeSpeeds();
}
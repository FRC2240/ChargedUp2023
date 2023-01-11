#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Translation2d.h>

#include <ctre/Phoenix.h>

class SwerveModule
{
public:
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    SwerveModule(int const &driver_adr, int const &turner_adr, int const &cancoder_adr, double const &magnet_offset);

    [[nodiscard]] frc::SwerveModuleState getState();

    [[nodiscard]] frc::SwerveModulePosition getPosition();

    [[nodiscard]] units::degree_t getAngle();

    [[nodiscard]] double getDriverTemp();

    [[nodiscard]] double getTurnerTemp();

    void setDesiredState(const frc::SwerveModuleState &state);

    void percentOutputControl(double const &percent_output);

    void manualVelocityContol(double const &velocity_ticks_per_100ms);

    // No copies/moves should be occuring (Talons don't support this)
    SwerveModule(SwerveModule const &) = delete;
    SwerveModule(SwerveModule &&) = delete;

private:
    /******************************************************************/
    /*                        Private Variables                       */
    /******************************************************************/

    WPI_TalonFX driver, turner;
    WPI_CANCoder cancoder;
    double const magnet_offset;
    int turner_addr;
};
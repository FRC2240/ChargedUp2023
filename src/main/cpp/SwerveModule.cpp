#include "SwerveModule.hpp"
#include "ngr.hpp"

//#include <wpi/numbers>
#include <units/angular_velocity.h>
#include <iostream>
#include <fmt/format.h>
#define CAN_BUS_NAME "canivore"

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

constexpr units::meter_t WHEEL_RADIUS = 2.32_in; // measured
constexpr auto MOTOR_ROTATIONS_TO_TALON_ENCODER_TICKS = 2048;
constexpr auto CANCODER_TICKS_PER_ROTATION = 4096;

constexpr auto DRIVER_GEAR_RATIO = 8.16;
constexpr auto TURNER_GEAR_RATIO = 12.8;

constexpr auto TICKS_PER_MOTOR_RADIAN =

    MOTOR_ROTATIONS_TO_TALON_ENCODER_TICKS / (2 * std::numbers::pi); // Number of ticks per radian

constexpr auto TICKS_PER_DRIVER_WHEEL_ROTATION =
    MOTOR_ROTATIONS_TO_TALON_ENCODER_TICKS * DRIVER_GEAR_RATIO;

constexpr auto DRIVER_TICKS_PER_WHEEL_RADIAN =
    TICKS_PER_MOTOR_RADIAN * DRIVER_GEAR_RATIO; // Total amount of ticks per wheel radian

constexpr auto HUNDREDMILLISECONDS_TO_1SECOND = 10; // Ticks / 100 milliseconds * 10 = Ticks / 1 second
constexpr auto ONESECOND_TO_100MILLISECONDS = .1;   // Ticks / second * .1 = Ticks / 100 milliseconds

constexpr auto TICKS_PER_TALON_ENCODER_DEGREE = MOTOR_ROTATIONS_TO_TALON_ENCODER_TICKS / 360;
constexpr auto TICKS_PER_CANCODER_DEGREE = CANCODER_TICKS_PER_ROTATION / 360;

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

// Set Variables
SwerveModule::SwerveModule(int const &driver_adr, int const &turner_adr, int const &cancoder_adr, double const &magnet_offset)
    : driver{driver_adr,CAN_BUS_NAME},
      turner{turner_adr, CAN_BUS_NAME},
      cancoder{cancoder_adr, CAN_BUS_NAME},
      magnet_offset{magnet_offset}
{
    //std::cout << "Swerve constuctor start \n";
    turner_addr = turner_adr;
    // Configure CANCoder
    CANCoderConfiguration cancoder_config{};
    cancoder_config.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    cancoder_config.unitString = "deg";
    cancoder_config.sensorDirection = true;
    cancoder_config.absoluteSensorRange = AbsoluteSensorRange::Signed_PlusMinus180;
    cancoder_config.magnetOffsetDegrees = magnet_offset;
    cancoder.ConfigAllSettings(cancoder_config);

    // Configure Driver
    TalonFXConfiguration driver_config{};
    driver_config.slot0.kP = 0.1;
    driver_config.slot0.kI = 0.002;
    driver_config.slot0.integralZone = 200;
    driver_config.slot0.kD = 10;
    driver_config.slot0.kF = 0.04857549857549857;
    driver_config.closedloopRamp = .2;
    // driver_config.voltageCompSaturation = 12;
    driver.ConfigAllSettings(driver_config);
    driver.SetNeutralMode(NeutralMode::Brake);
    driver.SetInverted(false);

    // Configure Turner
    TalonFXConfiguration turner_config{};
    turner_config.slot0.kP = 1.0;
    turner_config.slot0.kI = 0;
    turner_config.slot0.kD = 0;
    turner_config.slot0.kF = 0;
    turner_config.neutralDeadband = 0.07;
    turner_config.peakOutputForward = .5;
    turner_config.peakOutputReverse = -.5;
    turner_config.remoteFilter0.remoteSensorDeviceID = cancoder.GetDeviceNumber();
    turner_config.remoteFilter0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_CANCoder;
    turner_config.primaryPID.selectedFeedbackSensor = FeedbackDevice::RemoteSensor0;
    turner_config.closedloopRamp = .000;
    turner.ConfigAllSettings(turner_config);
    turner.SetInverted(false);
    //std::cout << "Swerve constuctor end \n";
}

frc::SwerveModuleState SwerveModule::getState()
{
    return {units::meters_per_second_t{(driver.GetSelectedSensorVelocity() / DRIVER_TICKS_PER_WHEEL_RADIAN * WHEEL_RADIUS) / 0.1_s},
            frc::Rotation2d(getAngle())};
}

frc::SwerveModulePosition SwerveModule::getPosition()
{
    return {units::meter_t{driver.GetSelectedSensorPosition() / TICKS_PER_DRIVER_WHEEL_ROTATION * (2 * std::numbers::pi) * WHEEL_RADIUS},
            frc::Rotation2d(getAngle())};
}

units::degree_t SwerveModule::getAngle() { return units::degree_t{cancoder.GetAbsolutePosition()}; }

double SwerveModule::getDriverTemp() { return driver.GetTemperature(); }

double SwerveModule::getTurnerTemp() { return turner.GetTemperature(); }

void SwerveModule::setDesiredState(frc::SwerveModuleState const &desired_state)
{
    frc::Rotation2d const current_rotation = getAngle();

    // Optimize the reference state to avoid spinning further than 90 degrees
    auto const [optimized_speed, optimized_angle] = frc::SwerveModuleState::Optimize(desired_state, current_rotation);

    // Convert speed (m/s) to ticks per 100 milliseconds
    double const desired_driver_velocity_ticks =
        (optimized_speed / WHEEL_RADIUS * DRIVER_TICKS_PER_WHEEL_RADIAN * ONESECOND_TO_100MILLISECONDS).value();

    // Difference between desired angle and current angle
    frc::Rotation2d delta_rotation = optimized_angle - current_rotation;


    // Convert change in angle to change in (cancoder) ticks
    double const delta_ticks = delta_rotation.Degrees().value() * TICKS_PER_CANCODER_DEGREE;

    // Get the current cancoder position
    double const current_ticks = turner.GetSelectedSensorPosition();
    // Can (theoretically) be replaced with current_ticks = current_rotation * TICKS_PER_CANCODER_DEGREE;

    // Finally, calculate what the new tick value should be
    double const desired_turner_pos_ticks = current_ticks + delta_ticks;

    /*std::cout << "desired speed: " << desired_state.speed.value() << "\n" <<
    " Desired Rotation: " << desired_state.angle.Degrees().value() <<  "\n\n" <<
    " Current rotation" << current_rotation.Degrees().value() << "\n" << 
    " Optomised speed " << optimized_angle.Degrees().value() << "\n" <<
    " Optomised angle " << delta_rotation.Degrees().value() << "\n" <<
    " Delta rotation " << delta_rotation.Degrees().value() << "\n" <<
    " delta ticks " << delta_ticks << "\n" <<
    " current ticks " << current_ticks << "\n" <<
    " desired driver " << desired_driver_velocity_ticks << "\n" <<
    " desired tuner " << desired_turner_pos_ticks << "\n";*/

    

    driver.Set(TalonFXControlMode::Velocity, desired_driver_velocity_ticks);

    turner.Set(TalonFXControlMode::Position, desired_turner_pos_ticks);
}

void SwerveModule::percentOutputControl(double const &percent_output)
{
    driver.Set(TalonFXControlMode::PercentOutput, percent_output);
    turner.Set(TalonFXControlMode::Position, turner.GetSelectedSensorPosition());
}

void SwerveModule::manualVelocityContol(double const &velocity_ticks_per_100ms)
{
    driver.Set(TalonFXControlMode::Velocity, velocity_ticks_per_100ms);
    turner.Set(TalonFXControlMode::Position, 0);
}

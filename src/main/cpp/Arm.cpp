#include "Arm.h"
#include <iostream>

constexpr auto CANCODER_TICKS_PER_ROTATION = 4096;
constexpr auto TICKS_PER_CANCODER_DEGREE = CANCODER_TICKS_PER_ROTATION / 360;


Arm::Arm()
{

    m_arm_motor_left.ConfigFactoryDefault();
    m_arm_motor_right.ConfigFactoryDefault();

    arm_cancoder.SetPositionToAbsolute();

    CANCoderConfiguration arm_cancoder_config{};
    TalonFXConfiguration arm_right_config{};

    arm_cancoder_config.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    arm_cancoder_config.unitString = "deg";
    arm_cancoder_config.sensorDirection = false;
    arm_cancoder_config.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;

    arm_right_config.remoteFilter0.remoteSensorDeviceID = arm_cancoder.GetDeviceNumber();
    arm_right_config.remoteFilter0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_CANCoder;
    arm_right_config.primaryPID.selectedFeedbackSensor = FeedbackDevice::RemoteSensor0;
    arm_right_config.slot0.kP = 1.5;
    arm_right_config.slot0.kD = 4.0;
    arm_right_config.slot0.kI = 0.0; //0.0008;

    arm_cancoder.ConfigAllSettings(arm_cancoder_config);
    m_arm_motor_right.ConfigAllSettings(arm_right_config);

    m_arm_motor_right.ConfigMotionCruiseVelocity(225);
    m_arm_motor_right.ConfigMotionAcceleration(225);

    // Follower
    m_arm_motor_left.Follow(m_arm_motor_right);
}

void Arm::move()
{
    double AFF = sin((3.1415/180)*(desired_position-horizontalPoint + 90)) * maxAFF;
    m_arm_motor_right.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::MotionMagic, desired_position * TICKS_PER_CANCODER_DEGREE,
    ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, AFF);
}

double Arm::Read_Position()
{
    position = arm_cancoder.GetAbsolutePosition(); //+ CONSTANTS::ARM::ARM_ENCODER_OFFSET;
    return position;
}

void Arm::force_move(double pos)
{
    double AFF = sin((3.1415/180)*(pos-horizontalPoint + 90)) * maxAFF;
    m_arm_motor_right.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::MotionMagic, pos * TICKS_PER_CANCODER_DEGREE,
    ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, AFF);
   
}
bool Arm::arm_moved(CONSTANTS::STATES state)
{

    switch (state)
    {
        case CONSTANTS::STATES::ABORT:
            desired_position = arm_cancoder.GetAbsolutePosition();
            move();
            break;
            
        case CONSTANTS::STATES::STORED:
            desired_position = CONSTANTS::ARM::MOTORPOSITIONS::STORED;
            move();
            return false;
            break;

        case CONSTANTS::STATES::HUMANPLAYER:
            desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HP;
            move();
            break;

        case CONSTANTS::STATES::PICKUP:
            desired_position = CONSTANTS::ARM::MOTORPOSITIONS::PICKUP;
            move();
            break;

        case CONSTANTS::STATES::LOW:
            desired_position = CONSTANTS::ARM::MOTORPOSITIONS::LOW;
            move();
            break;

        case CONSTANTS::STATES::MED:
            desired_position = CONSTANTS::ARM::MOTORPOSITIONS::MED;
            move();
            break;

        case CONSTANTS::STATES::HIGH:
            desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HIGH;
            move();
            break;

        case CONSTANTS::STATES::O_UP:
            desired_position = CONSTANTS::ARM::MOTORPOSITIONS::UP;
            move();
            break;
        
    }

    if (arm_cancoder.GetAbsolutePosition()/desired_position > CONSTANTS::ARM::MIN_THRESHOLD &&
        arm_cancoder.GetAbsolutePosition()/desired_position < CONSTANTS::ARM::MAX_THRESHOLD)
    {
        return true;
    } 
    else
    {
        return false;
    }

    
}

void Arm::test()
{
    std::cout << "arm: " << arm_cancoder.GetAbsolutePosition() << "\n";
}



Arm::~Arm(){}

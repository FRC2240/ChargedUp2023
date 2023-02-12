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
    arm_cancoder_config.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    arm_cancoder_config.unitString = "deg";
    arm_cancoder_config.sensorDirection = false;
    arm_cancoder_config.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
    arm_cancoder.ConfigAllSettings(arm_cancoder_config);
    TalonFXConfiguration arm_right_config{};
    arm_right_config.remoteFilter0.remoteSensorDeviceID = arm_cancoder.GetDeviceNumber();
    arm_right_config.remoteFilter0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_CANCoder;
    arm_right_config.primaryPID.selectedFeedbackSensor = FeedbackDevice::RemoteSensor0;
    arm_right_config.slot0.kP = 0.7;
    arm_right_config.slot0.kD = 0.0;
    arm_right_config.slot0.kI = 0.0008;
    m_arm_motor_right.ConfigAllSettings(arm_right_config);

    m_arm_motor_right.ConfigMotionCruiseVelocity(1000);
    m_arm_motor_right.ConfigMotionAcceleration(1000);
    
    //m_arm_motor_left.SetInverted(ctre::phoenix::motorcontrol::InvertType::OpposeMaster);

    //m_arm_motor_left.Follow(m_arm_motor_right);
}

void Arm::move()
{
    m_arm_motor_right.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position, desired_position * TICKS_PER_CANCODER_DEGREE);
}

void Arm::arm_pid_init()
{
    //Arm::arm_dash_read();

    m_arm_motor_right.ConfigFactoryDefault();

    TalonFXConfiguration arm_config{};

    arm_config.remoteFilter0.remoteSensorDeviceID = CONSTANTS::ARM::ARM_CANCODER_ID;
    arm_config.remoteFilter0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_CANCoder;
    arm_config.primaryPID.selectedFeedbackSensor = FeedbackDevice::RemoteSensor0;

    m_arm_motor_right.ConfigAllSettings(arm_config);

    m_arm_motor_right.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 0, 0);
    m_arm_motor_right.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 0, 0);

    m_arm_motor_right.ConfigNominalOutputForward(0, 0);
    m_arm_motor_right.ConfigNominalOutputReverse(0, 0);
    m_arm_motor_right.ConfigPeakOutputForward(1.0, 0);
    m_arm_motor_right.ConfigPeakOutputReverse(-1.0, 0);

    m_arm_motor_right.SelectProfileSlot(CONSTANTS::ARM::PID::slotIdx, CONSTANTS::ARM::PID::pidIdx);
    m_arm_motor_right.Config_kF(CONSTANTS::ARM::PID::slotIdx, m_Arm_RightCoeff.kFF, 0);
    m_arm_motor_right.Config_kP(CONSTANTS::ARM::PID::slotIdx, 0.4, 0);
    m_arm_motor_right.Config_kI(CONSTANTS::ARM::PID::slotIdx, m_Arm_RightCoeff.kI, 0);
    m_arm_motor_right.Config_kD(CONSTANTS::ARM::PID::slotIdx, m_Arm_RightCoeff.kD, 0);
}

void Arm::arm_dash_init()
{
    frc::SmartDashboard::PutNumber("Right Arm P Gain", m_Arm_RightCoeff.kP);
    frc::SmartDashboard::PutNumber("Right Arm I Gain", m_Arm_RightCoeff.kI);
    frc::SmartDashboard::PutNumber("Right Arm D Gain", m_Arm_RightCoeff.kD);
    frc::SmartDashboard::PutNumber("Right Arm FF Gain", m_Arm_RightCoeff.kFF);
    frc::SmartDashboard::PutNumber("Right Arm Max Output", m_Arm_RightCoeff.kMaxOutput);
    frc::SmartDashboard::PutNumber("Right Arm Min Output", m_Arm_RightCoeff.kMinOutput);

}

void Arm::arm_dash_read()
{
    m_Arm_RightCoeff.kP = frc::SmartDashboard::GetNumber("Right Arm P Gain", 0.0);
    m_Arm_RightCoeff.kI = frc::SmartDashboard::GetNumber("Right Arm I Gain", 0.0);
    m_Arm_RightCoeff.kD = frc::SmartDashboard::GetNumber("Right Arm D Gain", 0.0);
    m_Arm_RightCoeff.kFF = frc::SmartDashboard::GetNumber("Right Arm FF Gain", 0.0);
    m_Arm_RightCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Right Arm Min Output", -1.0);
    m_Arm_RightCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Right Arm Max Output", 1.0);

}

void Arm::Read_Position()
{
    position = arm_cancoder.GetAbsolutePosition() + CONSTANTS::ARM::ARM_ENCODER_OFFSET;
}

void Arm::Up(){
    m_arm_motor_right.Set(0.1);

    m_arm_motor_left.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
    m_arm_motor_left.Follow(m_arm_motor_right);
    
    // Add 7 degrees to all PID degree values
    //m_arm_motor_right.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position, 200.0 * TICKS_PER_CANCODER_DEGREE );
    //m_arm_motor_right.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput, .07);

    // double AFF = cos((3.1415/180)*(setPoint-horizontalPoint)) * maxAFF;
    // std::cout << AFF << std::endl;
    // m_arm_motor_right.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::MotionMagic, setPoint,
    // ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, AFF);
}

void Arm::Down(){
    m_arm_motor_right.Set(-0.1);
}

void Arm::Stop(){
    m_arm_motor_right.Set(0.0);
}

Arm::STATES Arm::arm_logic(bool store_button, bool low_button, 
                           bool med_button, bool hp_button,
                           bool high_button, bool pickup_button)
{

    switch(state)
    {
        case STORED:

            if(low_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::LOW;
                move();
                state = LOW;
            }
            else if(med_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::MED;
                move(); 
                state = MED;
            }
            else if(hp_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HP;
                move();
                state = HUMANPLAYER;
            }
            else if(high_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HIGH;
                move();
                state = HIGH;
            }
            else if(pickup_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::PICKUP;
                move();
                state = PICKUP;
            }
        break;

        case LOW:
            if(store_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::STORED;
                move();
                state = STORED;
            }
            else if(med_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::MED;
                move(); 
                state = MED;
            }
            else if(hp_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HP;
                move();
                state = HUMANPLAYER;
            }
            else if(high_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HIGH;
                move();
                state = HIGH;
            }
            else if(pickup_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::PICKUP;
                move();
                state = PICKUP;
            }

        break;

        case MED:
            if(store_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::STORED;
                move();
                state = STORED;
            }
            else if(low_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::LOW;
                move(); 
                state = LOW;
            }
            else if(hp_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HP;
                move();
                state = HUMANPLAYER;
            }
            else if(high_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HIGH;
                move();
                state = HIGH;
            }
            else if(pickup_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::PICKUP;
                move();
                state = PICKUP;
            }
            

        break;

        case HUMANPLAYER:
            if(store_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::STORED;
                move();
                state = STORED;
            }
            else if(low_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::LOW;
                move(); 
                state = LOW;
            }
            else if(med_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::MED;
                move();
                state = MED;
            }
            else if(high_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HIGH;
                move();
                state = HIGH;
            }
            else if(pickup_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::PICKUP;
                move();
                state = PICKUP;
            }
        
        break;

        case HIGH:
            if(store_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::STORED;
                move();
                state = STORED;
            }
            else if(low_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::LOW;
                move(); 
                state = LOW;
            }
            else if(med_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::MED;
                move();
                state = MED;
            }
            else if(hp_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HP;
                move();
                state = HUMANPLAYER;
            }
            else if(pickup_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::PICKUP;
                move();
                state = PICKUP;
            }

        break;

        case PICKUP:

            if(store_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::STORED;
                move();
                state = STORED;
            }
            else if(low_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::LOW;
                move(); 
                state = LOW;
            }
            else if(med_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::MED;
                move();
                state = MED;
            }
            else if(high_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HIGH;
                move();
                state = HIGH;
            }
            else if(hp_button)
            {
                desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HP;
                move();
                state = HUMANPLAYER;
            }
            break;

     }
    return state;    
}

void Arm::Test(){
    std::cout << "arm encoder with offset: " << position << std::endl;
}

void Arm::test()
{
    std::cout << "encoder: " << arm_cancoder.GetAbsolutePosition() << "\n";
}



Arm::~Arm(){}
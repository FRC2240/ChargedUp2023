#include "Arm.h"
#include <iostream>

Arm::Arm()
{
    m_arm_motor_left.Follow(m_arm_motor_right);
    Arm::arm_pid_init();
    //Arm::arm_dash_init();

    CANCoderConfiguration arm_cancoder_config{};
    //cancoder_config.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    arm_cancoder_config.sensorDirection = true;
    arm_cancoder.ConfigAllSettings(arm_cancoder_config);
    TalonFXConfiguration arm_turner_config{};
    arm_turner_config.remoteFilter0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_CANCoder;
    arm_turner_config.primaryPID.selectedFeedbackSensor = FeedbackDevice::RemoteSensor0;
    m_arm_motor_right.ConfigAllSettings(arm_turner_config);

    ARM_FLARE_HIGH = CONSTANTS::ARM::ARM_FLARE_HIGH;
    ARM_FLARE_LOW = CONSTANTS::ARM::ARM_FLARE_LOW;
}

void Arm::move()
{
    m_arm_motor_right.SetSelectedSensorPosition(desired_position);
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
    // m_arm_motor_right.Set(0.25);
    // m_arm_motor_left.Set(0.25);
    std::cout << "Arm up wahoo\n";
    m_arm_motor_right.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position, 158.0);
}

void Arm::Down(){
    m_arm_motor_right.Set(-0.25);
    //m_arm_motor_left.Set(-0.25);
}

void Arm::Stop(){
    m_arm_motor_left.Set(0.0);
    m_arm_motor_right.Set(0.0);
}

Arm::STATES Arm::arm_logic(bool store_button, bool low_button, 
                           bool med_button, bool hp_button,
                           bool high_button)
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

        break;

    }
}

void Arm::Test(){
    std::cout << "arm encoder with offset: " << position << std::endl;
}



Arm::~Arm(){}
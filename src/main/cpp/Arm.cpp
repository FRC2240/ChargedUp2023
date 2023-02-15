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


bool Arm::arm_moved(bool store_button_raw, bool low_button_raw, 
                    bool med_button_raw, bool hp_button_raw,
                    bool high_button_raw, bool pickup_button_raw)
{

    m_arm_motor_left.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
    m_arm_motor_left.Follow(m_arm_motor_right);

    if (store_button_raw)
    {
        store_button = !store_button;
    }

    if (pickup_button_raw)
    {
        pickup_button = !pickup_button;
    }

    if (low_button_raw)
    {
        low_button = !low_button;
    }

    if (med_button_raw)
    {
        med_button = !med_button;
    }

    if (hp_button_raw)
    {
        hp_button = !hp_button;
    }

    if (high_button_raw)
    {
        high_button = !high_button;
    }

    if(pickup_button)
    {
        std::cout << "state: " << "pickup" << "\n";
        desired_position = CONSTANTS::ARM::MOTORPOSITIONS::PICKUP;
        move();
        open_grabber = false;
    }
    else if(low_button)
    {
        std::cout << "state: " << "low" << "\n";
        desired_position = CONSTANTS::ARM::MOTORPOSITIONS::LOW;
        move();
    }
    else if(med_button)
    {
        std::cout << "state: " << "med" << "\n";
        desired_position = CONSTANTS::ARM::MOTORPOSITIONS::MED;
        move();
    }
    else if(hp_button)
    {
        std::cout << "state: " << "hp" << "\n";
        desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HP;
        move();
        open_grabber = false;
    }
    else if(high_button)
    {
        std::cout << "state: " << "high" << "\n";
        desired_position = CONSTANTS::ARM::MOTORPOSITIONS::HIGH;
        move();
    }
    else if(store_button)
    {
        std::cout << "state: " << "store" << "\n";
        desired_position = CONSTANTS::ARM::MOTORPOSITIONS::STORED;
        move();
        open_grabber = false;
    }

    if (((arm_cancoder.GetAbsolutePosition()/desired_position > CONSTANTS::ARM::MIN_THRESHOLD) &&
    (arm_cancoder.GetAbsolutePosition()/desired_position < CONSTANTS::ARM::MAX_THRESHOLD) && desired_position != CONSTANTS::ARM::MOTORPOSITIONS::STORED))
    {
        m_arm_timer.Start();
        open_grabber = true;

        if (m_arm_timer.Get() > units::time::second_t(2.0))
        {
            open_grabber = false;
            desired_position = CONSTANTS::ARM::MOTORPOSITIONS::STORED;
            move();
            m_arm_timer.Stop();
            m_arm_timer.Reset();
        }
    
        open_grabber = true;
    }
    open_grabber = false;
}

void Arm::test()
{
    std::cout << "encoder: " << arm_cancoder.GetAbsolutePosition() << "\n";
}



Arm::~Arm(){}
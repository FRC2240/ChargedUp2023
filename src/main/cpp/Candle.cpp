#include "Candle.h"

Candle::Candle(){

    ctre::phoenix::led::CANdleConfiguration candle_config;
    candle_config.stripType = ctre::phoenix::led::LEDStripType::RGB; 
    candle_config.brightnessScalar = 1.0; 
    m_candle.ConfigAllSettings(candle_config);

    m_candle.SetLEDs(0, 0, 0, 0);
    m_candle.ClearAnimation(0);
}

void Candle::candle_logic(bool left_button, bool right_button, 
                          bool yellow_button, bool purple_button, bool grabber_status)
{

    if (grabber_status == false){
        m_candle_timer.Start();
    }

    m_alliance = frc::DriverStation::GetAlliance();
    
    m_candle.ClearAnimation(0);

    previous_state = state;

    if (grabber_status = false && m_candle_timer.Get() < units::time::second_t(0.5)){
        state = GRABBER_FLASH;
    }
    else if (m_alliance == frc::DriverStation::Alliance::kRed)
    {
        m_candle.SetLEDs(255, 0, 0, 0, 8, 53);
        m_candle.SetLEDs(255, 0, 0, 0, 108, 100);
    } 
    else if (m_alliance == frc::DriverStation::Alliance::kBlue)
    {
        m_candle.SetLEDs(0, 0, 255, 0, 8, 53);
        m_candle.SetLEDs(0, 0, 255, 0, 108, 100);
    }
    
    if (left_button) 
    {
       side = true;
    } 
    if (right_button) 
    {
        side = false;
    } 
    if (purple_button)
    {
        color = true;
    } 
    if (yellow_button)
    {
        color = false;
    } 

    if (grabber_status = false && m_candle_timer.Get() < units::time::second_t(0.5)){
        state = GRABBER_FLASH;
    }
    else if (side && color)
    {
        state = PURPLE_LEFT;
    }
    else if (!side && color)
    {
        state = PURPLE_RIGHT;
    }
    else if (side && !color)
    {
        state = YELLOW_LEFT;
    }
    else if (!side && !color)
    {
        state = YELLOW_RIGHT;
    }

    switch (state)
    {
    case PURPLE_LEFT:
        m_candle.SetLEDs(82, 28, 200, 0, 63, 11);
        m_candle.SetLEDs(82, 28, 200, 0, 88, 12);

        break;

    case PURPLE_RIGHT:
        m_candle.SetLEDs(82, 28, 200, 0, 74, 12);
        m_candle.SetLEDs(82, 28, 200, 0, 100, 9);

        break;

    case YELLOW_LEFT:
        m_candle.SetLEDs(254, 162, 1, 0, 63, 11);
        m_candle.SetLEDs(254, 162, 1, 0, 88, 12);

        break;

    case YELLOW_RIGHT:
        m_candle.SetLEDs(254, 162, 1, 0, 74, 12);
        m_candle.SetLEDs(254, 162, 1, 0, 100, 9);

        break;

    case GRABBER_FLASH:
        m_candle.SetLEDs(0, 255, 0, 0);
        break; 
    }

    if (state != previous_state){
        m_candle.SetLEDs(0, 0, 0, 0);
    }

    if (grabber_status == true){
        m_candle_timer.Stop();
        m_candle_timer.Reset();
    }

    std::cout << grabber_status << std::endl;
}

void Candle::RainbowAnim()
{
    m_candle.Animate(rainbow);
}
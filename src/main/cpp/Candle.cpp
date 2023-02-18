#include "Candle.h"

Candle::Candle(){

    ctre::phoenix::led::CANdleConfiguration candle_config;
    candle_config.stripType = ctre::phoenix::led::LEDStripType::RGB; 
    candle_config.brightnessScalar = 1.0; 
    m_candle.ConfigAllSettings(candle_config);

    m_candle.SetLEDs(0, 0, 0, 0);
    m_candle.ClearAnimation(0);
    m_alliance = frc::DriverStation::GetAlliance();
}

Candle::STATES Candle::candle_logic(bool left_button, bool right_button, 
                                    bool yellow_button, bool purple_button)
{

    previous_state = state;
    m_candle.ClearAnimation();
    

    if (m_alliance == frc::DriverStation::Alliance::kRed){
        m_candle.SetLEDs(255, 0, 0, 0, 8, 54);
        m_candle.SetLEDs(255, 0, 0, 0, 108, 100);
    } 
    else if (m_alliance == frc::DriverStation::Alliance::kBlue){
        m_candle.SetLEDs(0, 0, 255, 0, 8, 54);
        m_candle.SetLEDs(255, 0, 255, 0, 108, 100);
    }
    

    if (left_button) {
        if (yellow_button){
            state = YELLOW_LEFT;
        } else if (purple_button){
            state = PURPLE_LEFT;
        }
    } 
    else if (right_button) {
        if (yellow_button){
            state = YELLOW_RIGHT;
        } else if (purple_button){
            state = PURPLE_RIGHT;
        }
    } 
    else if(purple_button){
        if (right_button){
            state = PURPLE_RIGHT;
        } else if (left_button){
            state = PURPLE_LEFT;
        }
    } 
    else if(yellow_button){
        if (right_button){
            state = PURPLE_RIGHT;
        } else if (left_button){
            state = PURPLE_LEFT;
        }
    } 

    switch (state)
    {
    case PURPLE_LEFT:
        m_candle.SetLEDs(82, 28, 200, 0, 63, 11);
        //m_candle.SetLEDs(82, 28, 200, 0);

        break;

    case PURPLE_RIGHT:
        m_candle.SetLEDs(82, 28, 200, 0, 74, 12);
        //m_candle.SetLEDs(82, 28, 200, 0);

        break;

    case YELLOW_LEFT:
        m_candle.SetLEDs(82, 28, 200, 0, 63, 11);
        //m_candle.SetLEDs(82, 28, 200, 0);

        break;

    case YELLOW_RIGHT:
        m_candle.SetLEDs(82, 28, 200, 0, 74, 12);
        //m_candle.SetLEDs(82, 28, 200, 0);

        break;
    
    default:
        m_candle.ClearAnimation(0);
        m_candle.SetLEDs(0, 0, 0, 0);
        break;
    }

    if (state != previous_state){
        m_candle.SetLEDs(0,0,0);
        m_candle.ClearAnimation(0);
    }
    
    return state;
}

void Candle::RainbowAnim()
{
    m_candle.Animate(rainbow1);
}
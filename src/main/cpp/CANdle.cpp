#include "CANdle.h"

CANdle::CANdle(){

    ctre::phoenix::led::CANdleConfiguration config;
    config.stripType = ctre::phoenix::led::LEDStripType::RGB; 
    config.brightnessScalar = 0.5; 
    m_candle.ConfigAllSettings(config);

    m_candle.SetLEDs(0, 0, 0);

}

void CANdle::Purple() {
    m_candle.SetLEDs(82, 28, 200);
}

void CANdle::Yellow() {
    m_candle.SetLEDs(254, 162, 1);
}

void CANdle::Red() {
    m_candle.SetLEDs(255, 0, 0);
}

void CANdle::Blue(){
    m_candle.SetLEDs(0, 0, 255);
}

void CANdle::Rainbow(){
    m_candle.Animate(rainbow);
}

void CANdle::Off(){
    m_candle.SetLEDs(0,0,0);
}
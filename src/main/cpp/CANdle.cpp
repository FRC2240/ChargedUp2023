#include "Candle.h"

Candle::Candle(){

    ctre::phoenix::led::CANdleConfiguration config;
    config.stripType = ctre::phoenix::led::LEDStripType::RGB; 
    config.brightnessScalar = 0.5; 
    m_candle.ConfigAllSettings(config);

    m_candle.SetLEDs(0, 0, 0);

}

void Candle::Purple() {
    m_candle.SetLEDs(82, 28, 200);
}

void Candle::Yellow() {
    m_candle.SetLEDs(254, 162, 1);
}

void Candle::Red() {
    m_candle.SetLEDs(255, 0, 0);
}

void Candle::Blue(){
    m_candle.SetLEDs(0, 0, 255);
}

void Candle::Rainbow(){
    m_candle.Animate(rainbow);
}

void Candle::Off(){
    m_candle.SetLEDs(0,0,0);
}
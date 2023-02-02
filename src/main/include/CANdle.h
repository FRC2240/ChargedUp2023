#include "Constants.h"
#include "ctre/phoenix/led/CANdle.h"
#include "Buttons.h"
#include "ctre/phoenix/led/RainbowAnimation.h"

class CANdle{
    public:

    CANdle();

    void Purple();
    void Yellow();
    void Red();
    void Blue();
    void Rainbow();
    void Off();

    private:
    ctre::phoenix::led::CANdle m_candle {CONSTANTS::CANDLE::CANDLE_ID, "canivore"};
    ctre::phoenix::led::RainbowAnimation rainbow{0.5, 0.5, -1};

};


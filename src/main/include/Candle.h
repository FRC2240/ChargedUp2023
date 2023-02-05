#include "Constants.h"
#include "ctre/phoenix/led/CANdle.h"
#include "Buttons.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include "ctre/phoenix/led/StrobeAnimation.h"

class Candle{
    public:

    Candle();

    void Purple();
    void Yellow();
    void Red();
    void Blue();
    void Green();
    void Rainbow();
    void Off();

    private:
    ctre::phoenix::led::CANdle m_candle {CONSTANTS::CANDLE::CANDLE_ID, "canivore"};
    ctre::phoenix::led::RainbowAnimation rainbow{0.5, 0.5, -1};
};


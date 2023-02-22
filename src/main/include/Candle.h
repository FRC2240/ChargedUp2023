#include "Constants.h"
#include "ctre/phoenix/led/CANdle.h"
#include "Buttons.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include <frc/DriverStation.h>
#include <frc/Timer.h>

class Candle{
    public:

    enum STATES { PURPLE_LEFT, PURPLE_RIGHT, YELLOW_LEFT, YELLOW_RIGHT, GRABBER_FLASH};

    void candle_logic(bool left_button, bool right_button, 
                        bool yellow_button, bool purple_button, bool grabber_status);

    Candle();

    void RainbowAnim();


    private:

    frc::DriverStation::Alliance m_alliance;

    STATES state = PURPLE_LEFT;
    STATES previous_state;

    bool side;
    bool color;

    bool previous_grabber_status;

    frc::Timer m_candle_timer;

    ctre::phoenix::led::CANdle m_candle {CONSTANTS::CANDLE::CANDLE_ID, "canivore"};
    ctre::phoenix::led::RainbowAnimation rainbow{1.0, 0.5, -1, false, 8};
};


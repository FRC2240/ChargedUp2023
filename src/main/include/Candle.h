#include "Constants.h"
#include "ctre/phoenix/led/CANdle.h"
#include "Buttons.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include <ctre/phoenix/led/LarsonAnimation.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <iostream>

class Candle{
    public:

    enum STATES
        {
            PURPLE_LEFT,
            PURPLE_RIGHT,
            YELLOW_LEFT,
            YELLOW_RIGHT,
            GRABBER_FLASH,
            RAINBOW,
            BOUNCE,
        };

    void candle_logic(bool left_button, bool right_button, 
                      bool yellow_button, bool purple_button, bool grabber_status);

    Candle();

    void rainbow_anim();
    void bounce_anim();

    void set_anim(STATES s);

    struct Color
    {
        int r;
        int g;
        int b;
        int w;
    };

    private:

    frc::DriverStation::Alliance m_alliance;

    STATES state = PURPLE_LEFT;
    STATES previous_state;

    bool side;
    bool color;

    bool previous_grabber_status;

    frc::Timer m_candle_timer;

    Color red{225,0,0,100};
    Color blue{0,0,225,100};

    ctre::phoenix::led::CANdle m_candle {CONSTANTS::CANDLE::CANDLE_ID, "canivore"};
    ctre::phoenix::led::RainbowAnimation rainbow{1.0, 0.5, -1, false, 8};
    ctre::phoenix::led::LarsonAnimation bounce{red.r, red.g, red.b, red.w, 0.7};
};

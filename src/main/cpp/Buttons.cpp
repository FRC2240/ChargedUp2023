#include "Buttons.hpp"
#include <fmt/format.h>

/******************************************************************/
/*                   Public Function Defintions                   */
/******************************************************************/

JoystickButton::JoystickButton(frc::GenericHID &stick, int const &button)
    : stick_{stick}, button_{button}
{
    fmt::print("is connected: {}\n", stick.IsConnected());
    if (button > stick.GetButtonCount() || button < 1)
        fmt::print("Invalid Button Assignment: {}\n", button);
}

bool JoystickButton::getRawButton() const
{
    return stick_.GetRawButton(button_);
}

bool JoystickButton::getRawButtonPressed()
{
    return stick_.GetRawButtonPressed(button_);
}

bool JoystickButton::getRawButtonReleased()
{
    return stick_.GetRawButtonReleased(button_);
}
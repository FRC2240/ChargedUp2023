#pragma once

#include <frc/GenericHID.h>
#include <frc/Joystick.h>

/******************************************************************/
/*                  Public Function Declarations                  */
/******************************************************************/
class JoystickButton
{
    frc::GenericHID &stick_;
    int const button_;

public:
    JoystickButton(frc::GenericHID &, int const &button);
    [[nodiscard]] operator bool() const { return stick_.GetRawButton(button_); }
    [[nodiscard]] bool getRawButton() const;
    [[nodiscard]] bool getRawButtonPressed();
    [[nodiscard]] bool getRawButtonReleased();
};

/******************************************************************/
/*                        Public Constants                        */
/******************************************************************/

namespace BUTTON
{
    inline frc::Joystick PS5{0};

    namespace DRIVETRAIN
    {
        inline JoystickButton ROTATE_FRONT{BUTTON::PS5, 4};
        inline JoystickButton ROTATE_BACK{BUTTON::PS5, 2};
        inline JoystickButton ROTATE_TO_CLOSEST{BUTTON::PS5, 1};
        inline JoystickButton TURN_45{BUTTON::PS5, 4};
        inline JoystickButton TURN_neg45{BUTTON::PS5, 2};
        inline JoystickButton TURN_90{BUTTON::PS5, 3};
        inline JoystickButton TURN_neg90{BUTTON::PS5, 1};
        inline JoystickButton ROTATION_MODE{BUTTON::PS5, 9};
        inline JoystickButton FIELD_CENTRIC{BUTTON::PS5, 10};
    }
}
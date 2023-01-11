#include "TempMonitoring.hpp"
#include "Buttons.hpp"

void TempMonitoring::flashWarning(double const &current_temp, double const &max_temp, std::string_view warning_msg, bool &flashing_red)
{
    if (current_temp > max_temp)
    {
        // oscillating between green & red to grab attention
        if (flashing_red)
        {
            frc::SmartDashboard::PutBoolean(warning_msg, true);
            flashing_red = false;
        }
        else
        {
            frc::SmartDashboard::PutBoolean(warning_msg, false);
            flashing_red = true;
        }
        BUTTON::PS5.SetRumble(BUTTON::PS5.kLeftRumble, .5);
        BUTTON::PS5.SetRumble(BUTTON::PS5.kRightRumble, .5);
    }
    else
    {
        frc::SmartDashboard::PutBoolean(warning_msg, true);
        BUTTON::PS5.SetRumble(BUTTON::PS5.kLeftRumble, 0);
        BUTTON::PS5.SetRumble(BUTTON::PS5.kRightRumble, 0);
    }
}

bool TempMonitoring::monitorTemp(double const &current_temp, double const &max_temp,
                                 std::string_view temperature_msg, std::string_view warning_msg,
                                 bool &flashing_red)
{
    frc::SmartDashboard::PutNumber(temperature_msg, current_temp);

    flashWarning(current_temp, max_temp, warning_msg, flashing_red);

    return current_temp > max_temp;
}
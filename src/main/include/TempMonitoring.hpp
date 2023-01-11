#pragma once

#include <wpi/array.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace TempMonitoring
{
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    void flashWarning(double const &current_temp, double const &max_temp,
                      std::string_view warning_msg, bool &flashing_red);
    bool monitorTemp(double const &current_temp, double const &max_temp,
                     std::string_view temperature_msg, std::string_view warning_msg,
                     bool &flashing_red);
    template <size_t t>
    bool monitorTemps(wpi::array<double, t> const &current_temps, double const &max_temp,
                      std::string_view temperature_msg, std::string_view warning_msg,
                      bool &flashing_red)
    {
        frc::SmartDashboard::PutNumberArray(temperature_msg, current_temps);

        double highest_temp = 0;
        for (double const &temp : current_temps)
        {
            if (temp > highest_temp)
                highest_temp = temp;
        }

        flashWarning(highest_temp, max_temp, warning_msg, flashing_red);
        return highest_temp > max_temp;
    }
}
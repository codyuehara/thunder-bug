#pragma once

#include <cmath>
#include <iostream>

class BatteryModel 
{
public:
    /**
     * U_nom: nominal battery (V)
     * R_int: internal resistance (Ohms)
    **/
    BatteryModel(float U_nom = 16.8f, float R_int = 0.05f)
      : U_nom_(U_nom), R_int_(R_int) {
        coeffs_[0] = 1;
        coeffs_[1] = 1;
        coeffs_[2] = 1;
        coeffs_[3] = 1;
        coeffs_[4] = 1;
    }

    float voltage(const std::array<float, 4>& motor_powers)
    {
        float P_total = 0.0f;
        for (auto p: motor_powers) P_total += p;
        float I_total = P_total / U_nom_; //approx current
        float U_bat = U_nom_ - I_total * R_int_;
        return std::max(U_bat, 12.0f); //optional min voltage
    }

    /**
     * steady state motor speed
    **/
    float calculateOmegaSS(float ucmd, float U_bat)
    {
        return coeffs_[0] + coeffs_[1]*U_bat + coeffs_[2]*std::sqrt(ucmd) + coeffs_[3]*ucmd + coeffs_[4]*(U_bat * std::sqrt(ucmd));
    }

    private:
        std::array<float, 5> coeffs_;
        float U_nom_;
        float R_int_;
};

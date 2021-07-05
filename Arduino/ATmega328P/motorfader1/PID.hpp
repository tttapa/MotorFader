#pragma once

#include "EMA.hpp"
#include <stdint.h>

const float maxOutput = 255.0;

class PID {
  public:
    PID(float k_p, float k_i, float k_d, float Ts)
        : k_p(k_p), k_i(k_i), k_d(k_d), Ts(Ts) {}

    float update(int16_t input) {
        int16_t error = setpoint - input;
        int16_t newIntegral = integral + error;
        float output = k_p * error + k_i * newIntegral * Ts +
                       k_d * (input - previousInput) / Ts;

        if (output > maxOutput)
            output = maxOutput;
        else if (output < -maxOutput)
            output = -maxOutput;
        else
            integral = newIntegral;

        previousInput = input;
        return output;
    }

    float k_p;
    float k_i;
    float k_d;
    float Ts;

    int16_t previousInput = 0;
    int16_t integral = 0;

    int16_t setpoint = 0;
};

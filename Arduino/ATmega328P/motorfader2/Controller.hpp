#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr inline float horner_impl(float xa, const float *p, size_t count,
                                   float t) {
    return count == 0 ? p[count] + xa * t
                      : horner_impl(xa, p, count - 1, p[count] + xa * t);
}

template <size_t N>
constexpr inline float horner(float x, float a, const float (&p)[N]) {
    return horner_impl(x - a, p, N - 2, p[N - 1]);
}

class PID {
  public:
    PID(float kp, float ki, float kd, float Ts, float f_c = 0,
        float maxOutput = 255.f)
        : Ts(Ts), maxOutput(maxOutput) {
        setKp(kp);
        setKi(ki);
        setKd(kd);
        setEMACutoff(f_c);
    }

    float update(int16_t input) {
        int16_t error = setpoint - input;
        int16_t newIntegral = integral + error;
        float diff = emaAlpha * input - emaAlpha * previousInput;
        float output = kp * error + ki_Ts * newIntegral + kd_Ts * diff;
        previousInput += diff;

        if (output > maxOutput)
            output = maxOutput;
        else if (output < -maxOutput)
            output = -maxOutput;
        else
            integral = newIntegral;

        return output;
    }

    void setKp(float kp) { this->kp = kp; }
    void setKi(float ki) { this->ki_Ts = ki * this->Ts; }
    void setKd(float kd) { this->kd_Ts = kd / this->Ts; }

    float getKp() const { return kp; }
    float getKi() const { return ki_Ts / Ts; }
    float getKd() const { return kd_Ts * Ts; }

    void setEMACutoff(float f_c) {
        // Taylor coefficients of
        // α(fₙ) = cos(2πfₙ) - 1 + √( cos(2πfₙ)² - 4 cos(2πfₙ) + 3 )
        // at fₙ = 0.25
        constexpr static float coeff[] {
            +7.3205080756887730e-01, +9.7201214975728490e-01,
            -3.7988125051760377e+00, +9.5168450173968860e+00,
            -2.0829320344443730e+01, +3.0074306603814595e+01,
            -1.6446172139457754e+01, -8.0756002564633450e+01,
            +3.2420501524111750e+02, -6.5601870948443250e+02,
        };
        if (f_c == 0) {
            emaAlpha = 1;
        } else {
            float f_n = f_c * Ts; // normalized sampling frequency
            emaAlpha = horner(f_n, 0.25, coeff);
        }
    }

    void setSetpoint(int16_t setpoint) { this->setpoint = setpoint; }
    int16_t getSetpoint() const { return setpoint; }

    void setMaxOutput(float maxOutput) { this->maxOutput = maxOutput; }
    float getMaxOutput() const { return maxOutput; }

  private:
    float Ts;
    float maxOutput;
    float kp;
    float ki_Ts;
    float kd_Ts;

    float emaAlpha = 1;

    float previousInput = 0;
    int16_t integral = 0;

    int16_t setpoint = 0;
};

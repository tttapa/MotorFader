#pragma once

#include <stddef.h>
#include <stdint.h>

/// @see @ref horner(float,float,const float(&)[N])
constexpr inline float horner_impl(float xa, const float *p, size_t count,
                                   float t) {
    return count == 0 ? p[count] + xa * t
                      : horner_impl(xa, p, count - 1, p[count] + xa * t);
}

/// Evaluate a polynomial using 
/// [Horner's method](https://en.wikipedia.org/wiki/Horner%27s_method).
template <size_t N>
constexpr inline float horner(float x, float a, const float (&p)[N]) {
    return horner_impl(x - a, p, N - 2, p[N - 1]);
}

/// Standard PID (proportional, integral, derivative) controller. Derivative
/// component is filtered using an exponential moving average filter.
class PID {
  public:
    /// @param  kp
    ///         Proportional gain
    /// @param  ki
    ///         Integral gain
    /// @param  kd
    ///         Derivative gain
    /// @param  Ts
    ///         Sampling time (seconds)
    /// @param  fc
    ///         Cutoff frequency of derivative EMA filter (Hertz),
    ///         zero to disable the filter entirely
    PID(float kp, float ki, float kd, float Ts, float f_c = 0,
        float maxOutput = 255.f)
        : Ts(Ts), maxOutput(maxOutput) {
        setKp(kp);
        setKi(ki);
        setKd(kd);
        setEMACutoff(f_c);
    }

    /// Update the controller: given the current position, compute the control
    /// action.
    float update(int16_t input) {
        // The error is the difference between the reference (setpoint) and the
        // actual position (input)
        int16_t error = setpoint - input;
        // The integral or sum of current and previous errors
        int16_t newIntegral = integral + error;
        // Compute the difference between the current and the previous input,
        // but compute a weighted average using a factor α ∊ (0,1]
        float diff = emaAlpha * input - emaAlpha * previousInput;
        // Update the average
        previousInput += diff;

        // Standard PID rule
        float output = kp * error + ki_Ts * newIntegral + kd_Ts * diff;

        // Clamp and anti-windup
        if (output > maxOutput)
            output = maxOutput;
        else if (output < -maxOutput)
            output = -maxOutput;
        else
            integral = newIntegral;

        return output;
    }

    void setKp(float kp) { this->kp = kp; }               ///< Proportional gain
    void setKi(float ki) { this->ki_Ts = ki * this->Ts; } ///< Integral gain
    void setKd(float kd) { this->kd_Ts = kd / this->Ts; } ///< Derivative gain

    float getKp() const { return kp; }         ///< Proportional gain
    float getKi() const { return ki_Ts / Ts; } ///< Integral gain
    float getKd() const { return kd_Ts * Ts; } ///< Derivative gain

    /// Set the cutoff frequency (-3 dB point) of the exponential moving average
    /// filter that is applied to the input before taking the difference for
    /// computing the derivative term.
    /// @see https://tttapa.github.io/Pages/Mathematics/Systems-and-Control-Theory/Digital-filters/Exponential%20Moving%20Average/Exponential-Moving-Average.html#cutoff-frequency
    ///      for the formula.
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
        float f_n = f_c * Ts; // normalized sampling frequency
        emaAlpha = f_c == 0 ? 1 : horner(f_n, 0.25, coeff);
    }

    /// Set the reference/target/setpoint of the controller.
    void setSetpoint(int16_t setpoint) { this->setpoint = setpoint; }
    /// @see @ref setSetpoint(int16_t)
    int16_t getSetpoint() const { return setpoint; }

    /// Set the maximum control output magnitude. Default is 255, which clamps
    /// the control output in [-255, +255].
    void setMaxOutput(float maxOutput) { this->maxOutput = maxOutput; }
    /// @see @ref setMaxOutput(float)
    float getMaxOutput() const { return maxOutput; }

  private:
    float Ts;                ///< Sampling time (seconds)
    float maxOutput;         ///< Maximum control output magnitude
    float kp;                ///< Proportional gain
    float ki_Ts;             ///< Integral gain times Ts
    float kd_Ts;             ///< Derivative gain divided by Ts
    float emaAlpha = 1;      ///< Weight factor of derivative EMA filter.
    float previousInput = 0; ///< (Filtered) previous input for derivative.
    int16_t integral = 0;    ///< Sum of previous errors for integral.
    int16_t setpoint = 0;    ///< Position reference.
};

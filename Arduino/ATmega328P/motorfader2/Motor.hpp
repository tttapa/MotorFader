#pragma once
#include "Registers.hpp"
#include <avr/interrupt.h>
#include <util/atomic.h>

/// Configure Timer0 in either phase correct or fast PWM mode with the given
/// prescaler, enable output compare B, and set pins PD4 and PD5 to output mode.
inline void setupMotorTimer(bool phase_correct_pwm, Timer0Prescaler prescaler) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setTimer0WGMode(phase_correct_pwm ? Timer0WGMode::PWM
                                          : Timer0WGMode::FastPWM);
        setTimer0Prescaler(prescaler);
        sbi(TCCR0A, COM0B1); // Table 14-6, 14-7 Compare Output Mode
        sbi(DDRD, 4);        // GPIO output mode
        sbi(DDRD, 5);
    }
}

/// Move the motor forward with the given duty cycle (speed).
inline void motorForward(uint8_t speed) {
    // Fast PWM (Table 14-6):
    //   Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting mode).
    // Phase Correct PWM (Table 14-7):
    //   Clear OC2B on compare match when up-counting. Set OC2B on compare match
    //   when down-counting.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        cbi(TCCR0A, COM0B0);
        cbi(PORTD, 4);
        OCR0B = speed;
    }
}

/// Move the motor backward with the given duty cycle (speed).
inline void motorBackward(uint8_t speed) {
    // Fast PWM (Table 14-6):
    //   Set OC0B on Compare Match, clear OC0B at BOTTOM (inverting mode).
    // Phase Correct PWM (Table 14-7):
    //   Set OC2B on compare match when up-counting. Clear OC2B on compare match
    //   when down-counting.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sbi(TCCR0A, COM0B0);
        sbi(PORTD, 4);
        OCR0B = speed;
    }
}
